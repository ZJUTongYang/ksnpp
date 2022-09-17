#include <raystar/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

// #include <raystar/dijkstra.h>
#include <raystar/astar.h>
#include "Atom.h"
#include "Robot.h"
#include <raystar/Node/Node.h>
#include "Definitions.h"
// #include <raystar/gradient_path.h>
#include <raystar/grid_path.h>
#include <raystar/quadratic_calculator.h>
#include <raystar/map_functions/map_functions.h>
#include "Corridor.h"
#include <visualization_msgs/Marker.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(raystar::RaystarPlanner, nav_core::BaseGlobalPlanner)

namespace raystar {



void RaystarPlanner::outlineMap(unsigned char* costarr, unsigned char value) {
    unsigned char* pc = costarr;
    for (int i = 0; i < cellsize_x; i++)
        *pc++ = value;
    pc = costarr + (cellsize_y - 1) * cellsize_x;
    for (int i = 0; i < cellsize_x; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < cellsize_y; i++, pc += cellsize_x)
        *pc = value;
    pc = costarr + cellsize_x - 1;
    for (int i = 0; i < cellsize_y; i++, pc += cellsize_x)
        *pc = value;
}

RaystarPlanner::RaystarPlanner() :
        initialized_(false), allow_unknown_(true),
        p_calc_(NULL), planner_(NULL), path_maker_(NULL), orientation_filter_(NULL),
        potential_array_(NULL) {
}

RaystarPlanner::RaystarPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
        RaystarPlanner() {
    //initialize the planner
    initialize(name);
}

RaystarPlanner::~RaystarPlanner() {
    if (p_calc_)
        delete p_calc_;
    if (planner_)
        delete planner_;
    if (path_maker_)
        delete path_maker_;
}

void RaystarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void RaystarPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) 
{
    // No use now
    initialize(name);
}

void RaystarPlanner::initialize(std::string name)
{
    if (!initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        // frame_id_ = costmap_.header.frame_id;

        cellsize_x = costmap_.info.width;
        cellsize_y = costmap_.info.height;

        convert_offset_ = 0;

        p_calc_ = new QuadraticCalculator(cellsize_x, cellsize_y);


        planner_ = new AStarExpansion(p_calc_, cellsize_x, cellsize_y);

        path_maker_ = new GridPath(p_calc_);

        // orientation_filter_ = new OrientationFilter();

        rays_pub = private_nh.advertise<visualization_msgs::Marker>("/rays", 1);
        roadmap_pub = private_nh.advertise<visualization_msgs::Marker>("/roadmap", 1);

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

        multi_path_pub_ = private_nh.advertise<visualization_msgs::Marker>("/multi_nonhomo_paths", 1);

        costmap_sub_ = private_nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap",
            1, boost::bind(&RaystarPlanner::costmapCb, this, _1));

        private_nh.param("allow_unknown", allow_unknown_, true);
        planner_->setHasUnknown(allow_unknown_);
        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        private_nh.param("default_tolerance", default_tolerance_, 0.0);
        private_nh.param("publish_scale", publish_scale_, 100);
        private_nh.param("outline_map", outline_map_, true);


        initialized_ = true;
    } else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");

}

void RaystarPlanner::costmapCb(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    std::cout << "enter costmapCb in RaystarPlanner callback" << std::endl;
    boost::mutex::scoped_lock lock(mutex_);
    ::costmap_ = *map;
    cellsize_x = ::costmap_.info.width;
    cellsize_y = ::costmap_.info.height;
}

void RaystarPlanner::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be free
    // costmap_.data[my*cellsize_x + mx] = costmap_2d::FREE_SPACE;
    costmap_.data[my*cellsize_x + mx] = CFREE;
}

void RaystarPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_.info.origin.position.x + (mx+convert_offset_) * costmap_.info.resolution;
    wy = costmap_.info.origin.position.y + (my+convert_offset_) * costmap_.info.resolution;
}

bool RaystarPlanner::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) {
    double origin_x = costmap_.info.origin.position.x;
    double origin_y = costmap_.info.origin.position.y;
    double resolution = costmap_.info.resolution;

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < cellsize_x && my < cellsize_y)
        return true;

    return false;
}

bool RaystarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan) {
    return makePlan(start, goal, default_tolerance_, plan);
}

bool RaystarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) {
    // No use now
    // 2021.3.9 wrap it into multi-goal implementation
    std::vector<geometry_msgs::PoseStamped> goals;
    goals.clear();
    goals.push_back(goal);
    std::vector<nav_msgs::Path> pathList;
    pathList.clear();
    bool b = makeMultiGoalPlan(start, goals, pathList);
    if(b && !pathList.empty())
    {
        std::cout << "we copy the result path to the buffer" << std::endl;
        // copy the nav_msgs::Path to vector of poses
        plan.clear();
        auto& path = pathList[0].poses;
        plan.assign(path.begin(), path.end());
    }
    // return !plan.empty();
    return true;
}


bool RaystarPlanner::makeMultiGoalPlan(const geometry_msgs::PoseStamped& start, 
    const std::vector<geometry_msgs::PoseStamped>& goals,
    std::vector<nav_msgs::Path>& plan) 
{
    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }
    
    std::cout << "enter makeMultiGoalPlan" << std::endl;
    

    //clear the plan, just in case
    plan.clear();

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i;
    double start_x, start_y;

    worldToMap(wx, wy, start_x_i, start_y_i);
    
    start_x = start_x_i;
    start_y = start_y_i;

    std::vector<std::pair<int, int> > goals_xy;

    unsigned int goal_x_i, goal_y_i;
    for(unsigned int i = 0; i < goals.size(); ++i)
    {

        wx = goals.at(i).pose.position.x;
        wy = goals.at(i).pose.position.y;

        worldToMap(wx, wy, goal_x_i, goal_y_i);

        if(costmap_.data[goal_y_i*cellsize_x + goal_x_i] <= CFREE || 
            costmap_.data[goal_y_i*cellsize_x + goal_x_i] > OBSTACLE)
        {
            goals_xy.push_back(std::pair<int, int>(goal_x_i, goal_y_i));
        }
        else
        {
            std::cout << "show cost of goal i: " << (int)(costmap_.data[goal_y_i*cellsize_x + goal_x_i]) << std::endl;
        }
    }

    if(goals_xy.empty())
    {
        std::cout << "ERROR: There should be some goals to plan. " << std::endl;
        return false;
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    clearRobotCell(start, start_x_i, start_y_i);    

    outlineMap((unsigned char*)&(costmap_.data[0]), OBSTACLE);

    // std::cout << "test before into ray*" << std::endl;
    // Start Ray* algorithm
    #if 1

        ros::WallTime whole_t0 = ros::WallTime::now();
        // TODO: We should set the footprint and detailed_footprint
        // TODO: we should set lethal_radius
        lethal_radius = 4;//Grid size of the robot's footprint
        computeFootprint();

        // Set goals
        ::goals.assign(goals_xy.begin(), goals_xy.end());

        std::cout << "Set goals successfully" << std::endl;

        // Clear storages
        solutions.clear();
        bestcost.clear();
        solved_goals.clear();
        solved_solutions.clear();
        solved_bestcost.clear();

        multi_solutions.clear();
        nodeindex_for_solutions.clear();


        // Setup trivial cost so that we can directly index different goals
        bestcost.resize(goals_xy.size(), 1000000);
        nodeindex_for_solutions.resize(goals_xy.size(), -1);

        num_of_nonhomo_path = 5;
        
        // Setup empty path to the solution so that we can index them
        // nav_msgs::Path temp;
        // temp.poses.clear();
        // for(unsigned int i = 0; i < ::goals.size(); ++i)
        // {
        //     solutions.push_back(temp);
        // }

        int Rindex = 0;
        int Nindex = 0;

        // Clear the robot at the beginning
        // If we use re-wire strategy, then don't clear them
        TheRobot(Rindex)->clear();

        TheRobot(Rindex)->N_.push_back(Node(Rindex, Nindex, std::pair<int, int>(start_x_i, start_y_i)));        

        TheRobot(Rindex)->N_.at(Nindex).Expand();

        if(!TheRobot(Rindex)->N_.at(Nindex).C_.empty())
        {
            for(int Cindex = 0; Cindex < TheRobot(Rindex)->N_.at(Nindex).C_.size(); ++Cindex)
            {
                if(TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).status_.compare("open") == 0)
                {
                    // No matter whether the corridor has optimality, we record its gap, 
                    // So that it may be used to remove other gaps

                    // Push the corridors into the openlist
                    TheRobot(Rindex)->openlist_.push_back(Candidate(Nindex, Cindex, 
                        TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).ckiFcost_));
                    
                }
            }
        }
        
        tryToFindSolutions(Rindex);

        saveOptimalSolutions(Rindex);

        bool finish = false;
        int index = 1;
        // while(!finish)
        for(index = 1; index < 150;)
        {
            std::cout << "index = " << index << std::endl;
            finish = replanning(Rindex);
            // std::cout << "time to construct node " << index << ": " << (t1-t0)*1000 << "ms." << std::endl;
            index = index + 1;
            // TheRobot(Rindex)->N_.back().showDetails();
            // TheRobot(Rindex)->showDetailOpenlist();
            if(finish)
            {
                std::cout << "\033[31m YT: foud path" << std::endl;
                break;
            }
        }

        std::cout << "outside loop" << std::endl;

        visualizeRoadmap();

        ros::WallTime whole_t1 = ros::WallTime::now();
        std::cout << "Time for whole Ray*: " << (whole_t1 - whole_t0)*1000 << "ms." << std::endl;
    
        std::cout << "size of solved goals: " << solved_goals.size() << std::endl;
        // if(solved_goals.empty())
        //     return false;

        // We have got path, but they are in grid-map. We need to transform them into world frame
        int mx, my;
        for(unsigned int i = 0; i < goals_xy.size(); ++i)
        {
            // We find the solutions one by one
            auto loc = std::find(solved_goals.begin(), solved_goals.end(), goals_xy.at(i));
            if(loc == solved_goals.end())
                continue;

            nav_msgs::Path& p = solved_solutions.at(loc - solved_goals.begin());

            for(unsigned int j = 0; j < p.poses.size(); ++j)
            {
                mx = p.poses.at(j).pose.position.x;
                my = p.poses.at(j).pose.position.y;
                mapToWorld(mx, my, wx, wy);
                p.poses.at(j).header.frame_id = costmap_.header.frame_id;
                p.poses.at(j).header.stamp = ros::Time::now();
                p.poses.at(j).pose.position.x = wx;
                p.poses.at(j).pose.position.y = wy;
                p.poses.at(j).pose.orientation.w = 1;
            }
            plan.push_back(p);
        }
        
        std::cout << "size of result path: " << plan.size() << std::endl;

        std::cout << "publish multiple non-homotopic paths" << std::endl;
        std::cout << "size of multi_paths: " << multi_solutions.size() << std::endl;
        double wxtemp, wytemp;
        for(auto iter1 = multi_solutions.begin(); iter1 != multi_solutions.end(); ++iter1)
        {
            std::cout << "Multi Path " << iter1 - multi_solutions.begin() << ": " << std::endl;
            for(auto iter2 = iter1->poses.begin(); iter2 != iter1->poses.end(); ++iter2)
            {
                mapToWorld(iter2->pose.position.x, iter2->pose.position.y, wxtemp, wytemp);
                iter2->pose.position.x = wxtemp;
                iter2->pose.position.y = wytemp;
                std::cout << "[" << wxtemp << ", " << wytemp << "], ";
            }
            std::cout << std::endl;
        }


        publishMultiPaths(multi_solutions);

        
        if(plan.size() == 1)
            publishPlan(plan[0].poses);

        // ros::Rate r(1);
        // while(ros::ok())
        //     r.sleep();

    #else

        ros::WallTime t0 = ros::WallTime::now();
        // make sure to resize the underlying array that Navfn uses
        p_calc_->setSize(cellsize_x, cellsize_y);
        planner_->setSize(cellsize_x, cellsize_y);
        path_maker_->setSize(cellsize_x, cellsize_y);
        potential_array_ = new float[cellsize_x * cellsize_y];


        std::cout << "enter Dijkstra" << std::endl;
        ros::WallTime dijkstra_t0 = ros::WallTime::now();
        bool found_legal = planner_->calculatePotentials((unsigned char*)&(costmap_.data[0]), start_x, start_y, goals_xy,
                                                        cellsize_x * cellsize_y * 2, potential_array_);
        ros::WallTime dijkstra_t1 = ros::WallTime::now();
        std::cout << "Time for Dijkstra calculating potential: " << (dijkstra_t1-dijkstra_t0)*1000 << "ms." << std::endl;
        std::cout << "leave Dijkstra" << std::endl;

        if(!old_navfn_behavior_)
            planner_->clearEndpoint((unsigned char*)&(costmap_.data[0]), potential_array_, goals_xy, 2);
        // if(publish_potential_)
            publishPotential(potential_array_);

        if (found_legal) {
            //extract the plan
            getPlanFromPotential(start_x, start_y, goals_xy, plan);
            
        }else{
            ROS_ERROR("Failed to get a plan.");
        }

        ros::WallTime t1 = ros::WallTime::now();
            std::cout << "time for multi_goal_dijkstra: " << (t1 - t0)*1000 << "ms" << std::endl;

        //publish the plan for visualization purposes
        if(plan.size() == 1)
            publishPlan(plan[0].poses);
        delete[] potential_array_;

    #endif
    // End Ray* algorithm
    std::cout << "leave makeMultiGoalPlan" << std::endl;

    
    return true;
}


void RaystarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    // gui_path.header.frame_id = frame_id_;
    gui_path.header.frame_id = costmap_.header.frame_id;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}


bool RaystarPlanner::getPlanFromPotential(double start_x, double start_y,
                                        std::vector<std::pair<int, int> >& goals,
                                        std::vector<nav_msgs::Path>& plan_array) 
{
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    // std::string global_frame = frame_id_;
    std::string global_frame = costmap_.header.frame_id;

    //clear the plan, just in case
    plan_array.clear();

    std::vector<std::pair<float, float> > path;
    double goal_x, goal_y;

    for(unsigned int i = 0; i < goals.size(); ++i)
    {
        path.clear();
        goal_x = goals.at(i).first;
        goal_y = goals.at(i).second;
        if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
            ROS_ERROR("NO PATH!");
            
        }
        else
        {
            // We insert path
            nav_msgs::Path nav_path;
            nav_path.header.stamp = ros::Time::now();
            nav_path.header.frame_id = global_frame;
            nav_path.poses.clear();

            ros::Time plan_time = ros::Time::now();
            for (int i = path.size() -1; i>=0; i--) {
                std::pair<float, float> point = path[i];
                //convert the plan to world coordinates
                double world_x, world_y;
                mapToWorld(point.first, point.second, world_x, world_y);

                geometry_msgs::PoseStamped pose;
                pose.header.stamp = plan_time;
                pose.header.frame_id = global_frame;
                pose.pose.position.x = world_x;
                pose.pose.position.y = world_y;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                nav_path.poses.push_back(pose);
            }
            plan_array.push_back(nav_path);
        }
        
    }

    return true;
}

bool RaystarPlanner::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                      const geometry_msgs::PoseStamped& goal,
                                       std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    // std::string global_frame = frame_id_;
    std::string global_frame = costmap_.header.frame_id;

    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float> > path;

    if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
        ROS_ERROR("NO PATH!");
        return false;
    }

    ros::Time plan_time = ros::Time::now();
    for (int i = path.size() -1; i>=0; i--) {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;
        mapToWorld(point.first, point.second, world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }
    if(old_navfn_behavior_){
            plan.push_back(goal);
    }
    return !plan.empty();
}

void RaystarPlanner::publishPotential(float* potential)
{

    double resolution = costmap_.info.resolution;
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    // grid.header.frame_id = frame_id_;
    grid.header.frame_id = costmap_.header.frame_id;

    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = cellsize_x;
    grid.info.height = cellsize_y;

    double wx, wy;
    mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(cellsize_x * cellsize_y);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++) {
        float potential = potential_array_[i];
        if (potential < POT_HIGH) {
            if (potential > max) {
                max = potential;
            }
        }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++) {
        if (potential_array_[i] >= POT_HIGH) {
            grid.data[i] = -1;
        } else
            grid.data[i] = potential_array_[i] * publish_scale_ / max;
    }
    potential_pub_.publish(grid);
}

bool RaystarPlanner::replanning(int Rindex)
{
    if(TheRobot(Rindex)->openlist_.empty())
    {
        // we cannot explore any more
        std::cout << "The openlist is empty." << std::endl;
        return true;
    }

    // The corridor has already contained a collision-free path
    // Setup best parameters
    auto m = std::min_element(TheRobot(Rindex)->openlist_.begin(), 
        TheRobot(Rindex)->openlist_.end(), 
        [](const Candidate& a, const Candidate& b){return a.leastFcost < b.leastFcost;});
    
    

    int Nindex = m->Nindex;
    int Cindex = m->Cindex;
	std::pair<double, double> newseedpoint = TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).cki_; 
	double newGcost = TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).ckiGcost_;
    int fathernodeindex = Nindex;
    int soninfatherindex = Cindex;
	double start_angle = TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).start_angle_;
    double end_angle = TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).end_angle_;

    TheRobot(Rindex)->openlist_.erase(m);

    // close the chosen corridor
    TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).status_ = "expanded";
    
    // Expand a new node
    int n = TheRobot(Rindex)->N_.size();
    TheRobot(Rindex)->N_.push_back(
        Node(Rindex, n, newseedpoint, start_angle, end_angle, newGcost, 
            fathernodeindex, soninfatherindex)
        );

    Nindex = n;

    TheRobot(Rindex)->N_.at(Nindex).Expand();

    if(!TheRobot(Rindex)->N_.at(Nindex).C_.empty())
    {
        for(Cindex = 0; Cindex < TheRobot(Rindex)->N_.at(Nindex).C_.size(); ++Cindex)
        {
            if(TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).status_.compare("open") == 0)
            {
                // no matter whether the corridor has optimality, we record its gap
                // so that it may be used to remove other gaps

                // push the corridors into the openlist
                TheRobot(Rindex)->openlist_.push_back(Candidate(Nindex, Cindex, 
                    TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).ckiFcost_));

            }
        }
    }

    // Check whether some goals are in the newly expanded node
    tryToFindSolutions(Rindex);    

    // Check whether this cost is the optimal
    saveOptimalSolutions(Rindex);

    if(goals.empty())
    {
        std::cout << "No goals to planning." << std::endl;
        return true;
    }
    // We just return and start another cycle
}

void RaystarPlanner::tryToFindSolutions(int Rindex)
{
    // We only check the last node in the tree, 
    // so we don't need to input its node index

    // std::cout << "try to find solutions" << std::endl;
    int Nindex = TheRobot(Rindex)->N_.size() - 1;

    // Check whether some goals are in the newly expanded node
    bool no_goal = true;
    for(unsigned int i = 0; i < goals.size(); ++i)
    {
        // std::cout << "goals[" << i << "] = (" << goals[i].first << ", " << goals[i].second << ")" << std::endl; 
        if(bestcost.at(i) > TheRobot(Rindex)->N_.at(Nindex).Gcost_ + 
            hypot(goals.at(i).first - TheRobot(Rindex)->N_.at(Nindex).seed_.first, 
                goals.at(i).second - TheRobot(Rindex)->N_.at(Nindex).seed_.second)
            )
        {
            // Check whether the goal is in this node 
            // std::cout << "before safe path finding" << std::endl;
            int b;
            std::vector<std::pair<int, int> > current_path;
            b = TheRobot(Rindex)->N_.at(Nindex).safePathFinding(goals.at(i), current_path);
            if(b == -1)
            {
                // It is impossible to find a path for this goal
                bestcost.at(i) = -1;
                std::cout << "ERROR: All goals should have path in our test!" << std::endl;
            }
            else if(b == 1)
            {
                // std::cout << "We've found a path for the " << i << "-th goal" << std::endl;
                no_goal = false;
                double cost = pathLength(current_path);
                if(cost < bestcost.at(i))
                {
                    nav_msgs::Path pathtemp;
                    // solutions.at(i) = current_path;
                    // std::cout << "in try test 1" << std::endl;
                    pathtemp.poses.reserve(current_path.size());
                    pathtemp.poses.clear();
                    // std::cout << "in try test 2" << std::endl;

                    geometry_msgs::PoseStamped temp;
                    for(unsigned int j = 0; j < current_path.size(); ++j)
                    {
                        temp.pose.position.x = current_path.at(j).first;
                        temp.pose.position.y = current_path.at(j).second;
                        pathtemp.poses.push_back(temp);
                    }
                    if(solutions.empty() && i == 0 && bestcost.size() == 1)
                    {
                        solutions.push_back(pathtemp);
                        bestcost.at(i) = cost;
                    }
                    else
                    {
                        std::cout << "error: For multi-homotopic-path application, we need this to be empty." 
                                  << "solutions.size() = " << solutions.size() << ", i = " << i 
                                  << ", bestcost.size() = " << bestcost.size() << std::endl;
                    }
                    

                    nodeindex_for_solutions.at(i) = TheRobot(Rindex)->N_.size() - 1;
                    // std::cout << "finish save current's optimal path" << std::endl;
                }

            }
            else
            {
                // The goal is not in this node, we do nothing
            }
        }
    }

    // if(no_goal && TheRobot(Rindex)->N_.at(Nindex).C_.empty())
    // {
    //     TheRobot(Rindex)->N_.at(Nindex).createBlock();
    // }
}

void RaystarPlanner::saveOptimalSolutions(int Rindex)
{
    // std::cout << "enter saveOptimalSolutions" << std::endl;
    if(goals.empty())
    {
        std::cout << "We should have non-empty goals to find path." << std::endl;
    }

    for(int i = goals.size()-1; i >= 0; --i)
    {
        // std::cout << "i = " << i << std::endl;
        if(bestcost.at(i) == -1 || currentIsOptimal(i, Rindex))
        {
            // std::cout << "test 1" << std::endl;
            // We store the path
            num_of_nonhomo_path = num_of_nonhomo_path - 1;
            std::cout << "size of optimal solution: " << solutions.at(i).poses.size() << ", cost = " << bestcost.at(i) << std::endl;
            multi_solutions.push_back(solutions.at(i));
            solutions.erase(solutions.begin() + i);
            bestcost.at(i) = 1000000;

            // We mark all successors of this node as "expanded", because we think that
            // a non-homotopic path with exactly the same starting point is unnecessary in robotics
            int Nindex = nodeindex_for_solutions.at(i);

            TheRobot(Rindex)->N_.at(Nindex).removeUsage();
// std::cout << "test 2" << std::endl;
            // We remove the gap and sweeper list of the result path
            int fatherindex = TheRobot(Rindex)->N_.at(Nindex).fatherindex_;
            int soninfatherindex = TheRobot(Rindex)->N_.at(Nindex).soninfatherindex_;
            std::vector<int> fatherlist;
            TheRobot(Rindex)->N_.at(fatherindex).C_.at(soninfatherindex).getPredecessors(fatherlist);
// std::cout << "test 3" << std::endl;
            for(int j = TheRobot(Rindex)->sweeper_list_.size()-1; j >= 0; --j)
            {
                if(TheRobot(Rindex)->sweeper_list_.at(j).Nindex == Nindex)
                    TheRobot(Rindex)->sweeper_list_.erase(TheRobot(Rindex)->sweeper_list_.begin() + j);
            }
            for(int j = TheRobot(Rindex)->path_list_.size()-1; j >= 0; --j)
            {
                if(TheRobot(Rindex)->path_list_.at(j).Nindex == Nindex)
                    TheRobot(Rindex)->path_list_.erase(TheRobot(Rindex)->path_list_.begin() + j);
            }
// std::cout << "test 4" << std::endl;
            for(int i = 0; i < fatherlist.size(); i+= 2)
            {
                for(int j = TheRobot(Rindex)->sweeper_list_.size()-1; j >= 0; --j)
                {
                    if(TheRobot(Rindex)->sweeper_list_.at(j).Nindex == fatherlist.at(i) &&
                        TheRobot(Rindex)->sweeper_list_.at(j).Cindex == fatherlist.at(i+1))
                    {
                        TheRobot(Rindex)->sweeper_list_.erase(TheRobot(Rindex)->sweeper_list_.begin() + j);
                        break;                        
                    }
                }
                for(int j = TheRobot(Rindex)->path_list_.size()-1; j >= 0; --j)
                {
                    if(TheRobot(Rindex)->path_list_.at(j).Nindex == fatherlist.at(i) && 
                        TheRobot(Rindex)->path_list_.at(j).Cindex == fatherlist.at(i+1))
                    {
                        TheRobot(Rindex)->path_list_.erase(TheRobot(Rindex)->path_list_.begin() + j);
                    }
                }
            }

// std::cout << "test 5" << std::endl;
            // We re-open all "no_usage" nodes, and insert them into openlist
            for(auto i = TheRobot(Rindex)->N_.begin(); i != TheRobot(Rindex)->N_.end(); ++i)
            {
                for(auto j = i->C_.begin(); j != i->C_.end(); ++j)
                {
                    if(j->status_.compare("no_usage") == 0)
                    {
                        j->status_ = "open";
                        TheRobot(Rindex)->openlist_.push_back(
                            Candidate(i-TheRobot(Rindex)->N_.begin(), j - i->C_.begin(), 
                            j->ckiFcost_));
                    }
                }
            }
// std::cout << "test 6" << std::endl;
            if(num_of_nonhomo_path == 0)
                goals.erase(goals.begin() + i);

        }


    }

    // for(int i = goals.size()-1; i>= 0; --i)
    // {
    //     std::cout << "cost of the current cost: " << bestcost.at(i) << std::endl;
    //     if(bestcost.at(i) == -1 || currentIsOptimal(i, Rindex))
    //     {
    //         // std::cout << "i = " << i << std::endl;
    //         // std::cout << "the solution for goal" << i << "is optimal. " << std::endl;
    //         solved_goals.push_back(goals.at(i));
    //         goals.erase(goals.begin() + i);
    //         // std::cout << "erase goals" << std::endl;
    //         solved_solutions.push_back(solutions.at(i));
    //         solutions.erase(solutions.begin()+i);
    //         // std::cout << "erase solutions" << std::endl;
    //         solved_bestcost.push_back(bestcost.at(i));
    //         bestcost.erase(bestcost.begin()+i);
    //         // std::cout << "erase cost" << std::endl;

    //         // std::cout << "size of openlist: " << TheRobot(Rindex)->openlist_.size() << std::endl;
    //         // We should update the openlist, 
    //         // so that all cost calculation towards this goal are removed. 
    //         if(!TheRobot(Rindex)->openlist_.empty())
    //         {
    //             for(unsigned int j = 0; j < TheRobot(Rindex)->openlist_.size(); ++j)
    //             {
    //                 // std::cout << "check size of heuristic for Candidate: " << TheRobot(Rindex)->openlist_.at(j).Fcost.size() << std::endl;
    //                 TheRobot(Rindex)->openlist_.at(j).removeHeuristic(i);
    //             }
    //         }
    //         // std::cout << "finish remove Heuristic" << std::endl;
    //     }
    // }
}



void RaystarPlanner::publishMultiPaths(std::vector<nav_msgs::Path> paths)
{
    static int index = 0;

    if(paths.size() == 0)
        return ;
        
    visualization_msgs::Marker lines;

    lines.header.stamp = ros::Time::now();
    lines.header.frame_id = "map";

    lines.ns = "multi_nonhomo_paths";
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.pose.orientation.w = 1.0;

    // we first remove the previous visualization
    lines.action = 3;
    multi_path_pub_.publish(lines);



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

        multi_path_pub_.publish(lines);
        lines.id ++;

    }
    index = lines.id - 1;
}

} //end namespace raystar
