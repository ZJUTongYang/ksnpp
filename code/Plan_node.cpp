/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Bhaskara Marthi
 *         David V. Lu!!
 *********************************************************************/
#include <raystar/planner_core.h>
#include <navfn/MakeNavPlan.h>
#include <raystar/MultiGoalPlan.h>
// #include "Definitions.h"
#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#define USE_RAYSTAR_VISUALIZATION 0

namespace cm = costmap_2d;
namespace rm = geometry_msgs;

using std::vector;
using rm::PoseStamped;
using std::string;
using cm::Costmap2D;
using cm::Costmap2DROS;

extern nav_msgs::OccupancyGrid costmap_;
extern int cellsize_x;
extern int cellsize_y;
extern ros::Publisher rays_pub;
extern ros::Publisher roadmap_pub;

namespace raystar {

class PlannerWithCostmap : public RaystarPlanner {
    public:
        PlannerWithCostmap(string name);
        bool multiGoalPlanService(raystar::MultiGoalPlan::Request& req, raystar::MultiGoalPlan::Response& resp);
        void publishMultiPaths(std::vector<nav_msgs::Path> paths);
    private:
        void costmapCb(const nav_msgs::OccupancyGrid::ConstPtr& map)
        {
            boost::mutex::scoped_lock lock(mutex_);
            std::cout << "enter costmapCb" << std::endl;
            ::costmap_ = *map;
            std::cout << "leave costmapCb" << std::endl;
            cellsize_x = ::costmap_.info.width;
            cellsize_y = ::costmap_.info.height;
        }
        
        tf::TransformListener tf_listener_;
        ros::ServiceServer multi_goal_plan_service_;
        ros::Subscriber pose_sub_;
        ros::Publisher paths_pub_;
        ros::Subscriber costmap_sub_;
        ros::Publisher goals_pub_;
        
};

bool PlannerWithCostmap::multiGoalPlanService(raystar::MultiGoalPlan::Request& req, raystar::MultiGoalPlan::Response& resp)
{
    // The global planner enforce all its received starting and goal points are in "map" frame
    // req.start.header.frame_id = "map";
    std::cout << "enter multiGoalPlanService" << std::endl;

    std::cout << "size of requested point: " << req.goals.poses.size() << std::endl;

    if(req.goals.poses.size() == 0)
    {
        return true;
    }

    geometry_msgs::PoseStamped start_temp;

    geometry_msgs::PoseStamped iden_pose;
    iden_pose.header.frame_id = "base_link";
    iden_pose.header.stamp = ros::Time();
    iden_pose.pose.orientation.w = 1;

    tf_listener_.transformPose(costmap_.header.frame_id, iden_pose, start_temp);

// We transform all goal points into the desired frame
    std::vector<geometry_msgs::PoseStamped> goals_temp;
    geometry_msgs::PoseStamped goal_temp;
    for(unsigned int i = 0; i < req.goals.poses.size(); ++i)
    {
        if(req.goals.poses.at(i).header.frame_id != costmap_.header.frame_id)
        {
            // transform the path to the "map" frame
            try{
                tf_listener_.transformPose(::costmap_.header.frame_id, req.goals.poses.at(i), goal_temp);
            }
            catch( tf::TransformException ex)
            {
                ROS_WARN("transfrom exception : %s",ex.what());
                return false;
            }
            goals_temp.push_back(goal_temp);
        }
        else
        {
            goals_temp.push_back(req.goals.poses.at(i));
        }
    }


    pcl::PointCloud<pcl::PointXYZI> frontier_cloud_viz;
    pcl::PointXYZI frontier_point_viz(100);

    BOOST_FOREACH(geometry_msgs::PoseStamped temp, goals_temp){
        //load frontier into visualization poitncloud
        frontier_point_viz.x = temp.pose.position.x;
        frontier_point_viz.y = temp.pose.position.y;
        frontier_cloud_viz.push_back(frontier_point_viz);
    }

    // We visualize all goals to be solved
    sensor_msgs::PointCloud2 frontier_viz_output;
    pcl::toROSMsg(frontier_cloud_viz,frontier_viz_output);
    frontier_viz_output.header.frame_id = costmap_.header.frame_id;
    frontier_viz_output.header.stamp = ros::Time::now();
    goals_pub_.publish(frontier_viz_output);


    // We find paths for all goals simultaneously 
    bool success;

    success = makeMultiGoalPlan(start_temp, goals_temp, resp.multi_paths);

    publishMultiPaths(resp.multi_paths);

    std::cout << "leave multiGoalPlanService" << std::endl;

    return true;
}

void PlannerWithCostmap::publishMultiPaths(std::vector<nav_msgs::Path> paths)
{
    static int index = 0;

    if(paths.size() == 0)
        return ;
        
    visualization_msgs::Marker lines;

    lines.header.stamp = ros::Time::now();
    lines.header.frame_id = "map";

    lines.ns = "exploration_path";
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.pose.orientation.w = 1.0;

    // we first remove the previous visualization
    lines.action = 3;
    paths_pub_.publish(lines);



    lines.id = 0;

    lines.action = 0;
    lines.scale.x = 0.4;
    lines.color.r = 1;
    lines.color.a = 0.5;

    // std::cout << "show path" << std::endl;
    BOOST_FOREACH(nav_msgs::Path path, paths)
    {
        if(path.poses.size() == 0)
            continue;

        lines.points.resize(path.poses.size()*2-2);
        for(unsigned int j = 0; j < path.poses.size()-1; ++j)
        {
            lines.points.at(2*j).x = path.poses.at(j).pose.position.x;
            lines.points.at(2*j).y = path.poses.at(j).pose.position.y;
            lines.points.at(2*j+1).x = path.poses.at(j+1).pose.position.x;
            lines.points.at(2*j+1).y = path.poses.at(j+1).pose.position.y;
        }

        paths_pub_.publish(lines);
        lines.id ++;

    }
    index = lines.id - 1;
}

PlannerWithCostmap::PlannerWithCostmap(string name) :
        RaystarPlanner(name) {
    ros::NodeHandle private_nh("~");
    costmap_sub_ = private_nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap",
            1, boost::bind(&PlannerWithCostmap::costmapCb, this, _1));
    paths_pub_ = private_nh.advertise<visualization_msgs::Marker>("/paths", 1);
    multi_goal_plan_service_ = private_nh.advertiseService("multi_goal_plan", &PlannerWithCostmap::multiGoalPlanService, this);
    
    
    goals_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("frontiers",5);
}

} // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "raystar_for_service");

    // tf2_ros::Buffer buffer(ros::Duration(10));
    // tf2_ros::TransformListener tf(buffer);

    raystar::PlannerWithCostmap pppp("raystar_planner");

    ros::spin();
    return 0;
}

