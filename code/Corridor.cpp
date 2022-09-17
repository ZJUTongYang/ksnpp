#include "Corridor.h"
#include "Atom.h"
#include "Robot.h"
#include "Definitions.h"
#include "Relative_optimality.h"
#include "Restricted_astar.h"
#include "Map_functions.h"
#include <cmath>
#include <iostream>
#include <algorithm>

#include <corecrt_math_defines.h>


Corridor::Corridor(int Rindex, int Nindex, int Cindex, std::pair<double, double> seed, 
		ObsExit a_cc_exit, double gcost)
{
	Rindex_ = Rindex;
	Nindex_ = Nindex;
	Cindex_ = Cindex;
	pi_ = seed;
	piGcost_ = gcost;
	status_ = "open";

	me_ = a_cc_exit;
	//std::cout << "corridor construction: " << me_.thetasmall << ", " << me_.dissmall << ", " << me_.smallx << ", " << me_.smally 
	//		  << ", " << me_.thetabig << ", " << me_.disbig << ", " << me_.bigx << ", " << me_.bigy << std::endl;

	son_Nindex_ = -1;

	double left, right;
	std::pair<double, double> leftpoint, rightpoint;

	double normleftpoint;
	double normrightpoint;

	// Calculate based on different situation
	// a_cc_exit(1) is strictly less than a_cc_exit(5)
	if(a_cc_exit.dissmall > a_cc_exit.disbig)
	{
		angle_long_ = a_cc_exit.thetasmall;
		angle_short_ = a_cc_exit.thetabig;
		mid_.first = pi_.first + a_cc_exit.disbig*cos(angle_long_);
		mid_.second = pi_.second + a_cc_exit.disbig*sin(angle_long_);

		footprintProjection(angle_long_, 
			left, right, leftpoint, rightpoint);

		normleftpoint = hypot(leftpoint.first, leftpoint.second);
		normrightpoint = hypot(rightpoint.first, rightpoint.second);

		// judge the critical point
		// have 1 threshold

		cki_ = mid_ - leftpoint/normleftpoint*(normleftpoint+0.5);
		cki_mirror_ = mid_ - rightpoint/normrightpoint*(normrightpoint+0.5);
		width_ = std::pair<double, double>(right+0.5, left+0.5);

		// for finding near obstacle
		std::pair<double, double> for_near = pi_ + 
			(a_cc_exit.disbig+0.1)*std::pair<double, double>(cos(a_cc_exit.thetabig), sin(a_cc_exit.thetabig));

		near_obstacle_ = for_near;

		Trans1_ = std::vector<double>{-sin(angle_long_), cos(angle_long_), pi_.first, 
			                          cos(angle_long_), sin(angle_long_), pi_.second, 
			                          0, 0, 1};
	}
	else
	{
		angle_long_ = a_cc_exit.thetabig;
		angle_short_ = a_cc_exit.thetasmall;
		mid_.first = pi_.first+a_cc_exit.dissmall*cos(angle_long_);
		mid_.second = pi_.second+a_cc_exit.dissmall*sin(angle_long_);

		footprintProjection(angle_long_, 
			left, right, leftpoint, rightpoint);

		normleftpoint = hypot(leftpoint.first, leftpoint.second);
		normrightpoint = hypot(rightpoint.first, rightpoint.second);

		// judge the critical point
		cki_ = mid_ - rightpoint/normrightpoint*(normrightpoint+0.5);
		cki_mirror_ = mid_ - leftpoint/normleftpoint*(normleftpoint+0.5);

		width_ = std::pair<double, double>(left+0.5, right+0.5);

		// For finding near obstacle
		// Here the 0.1 must be consistent to the pointRay function
		std::pair<double, double> for_near = pi_ + 
			(a_cc_exit.dissmall+0.1)*std::pair<double, double>(cos(a_cc_exit.thetasmall), sin(a_cc_exit.thetasmall));

		near_obstacle_ = for_near;
	// 	near_obstacle_ = pi_ + a_cc_exit.dissmall*std::pair<double, double>(cos(a_cc_exit.thetasmall), sin(a_cc_exit.thetasmall));


		Trans1_ = std::vector<double>{sin(angle_long_), -cos(angle_long_), pi_.first, 
			                          cos(angle_long_), sin(angle_long_), pi_.second, 
			                          0, 0, 1};
	}

	Trans2_ = std::vector<double>{leftpoint.first, leftpoint.second, pi_.first, 
		                          cos(angle_long_), sin(angle_long_), pi_.second, 
		                          0, 0, 1};
	Trans3_ = std::vector<double>{rightpoint.first, rightpoint.second, pi_.first, 
							   	  cos(angle_long_), sin(angle_long_), pi_.second, 
	                              0, 0, 1};
	pi_fake_.first = pi_.first + 2*cos(angle_long_);
	pi_fake_.second = pi_.second + 2*sin(angle_long_);
	std::vector<std::pair<int, int> > fake_pi_path_temp;
	classicBresenham(pi_.first, pi_.second, pi_fake_.first, pi_fake_.second, fake_pi_path_temp);
	fake_pi_path_.data.clear();
	for (auto iter = fake_pi_path_temp.begin(); iter != fake_pi_path_temp.end(); ++iter)
	{
		fake_pi_path_.data.push_back(std::pair<double, double>(iter->first + 0.5, iter->second + 0.5));
	}

	cki_fake_.first = cki_.first - 2*cos(angle_long_);
	cki_fake_.second = cki_.second - 2*sin(angle_long_);
	std::vector<std::pair<int, int> > fake_cki_path_temp;
	classicBresenham(cki_fake_.first, cki_fake_.second, cki_.first, cki_.second, fake_cki_path_temp);
	fake_cki_path_.data.clear();
	for (auto iter = fake_cki_path_temp.begin(); iter != fake_cki_path_temp.end(); ++iter)
	{
		fake_cki_path_.data.push_back(std::pair<double, double>(iter->first + 0.5, iter->second + 0.5));
	}

}

bool Corridor::basicChecking(std::vector<std::pair<int, int> >& obs_list)
{
	// true: the corridor can pass basic checking
	// false: it cannot
	// The basicChecking is coarse, i.e., it does not guarantee the existance of the result path segment. 
	// However, we will detailedly do the path finding later, so it does not matter

	// The obs_list must be returned, because there may be some other exits that first be overlooked because overlooking an obstacle

	obs_list.clear();

	if(CC(floor(cki_.first), floor(cki_.second)) == -1)
	{
		getDetailedCCHittingObstacle(floor(cki_.first), floor(cki_.second), obs_list);
		if(obs_list.empty())
		{
			std::cout << "YT: error, we should find some missing obstacles for point: ("
				<< floor(cki_.first) << ", " << floor(cki_.second) << "), whose map cost is"
				<< (int)(costmap_.data[(int)(floor(cki_.second)*cellsize_x + floor(cki_.first))]) << std::endl;
			std::cout << "check size of detailed footprint: " << detailed_footprint.size() << std::endl;
		}
		return false;
	}
	return true;
}

double Corridor::gapSweeper()
{
	// std::cout << "in gapSweeper" << std::endl;
	std::vector<std::pair<int, int> > hitting_obstacles;
	std::pair<double, double> seed = cki_;
	double theta = angle_long_;
	double dis;//no usage
	bool infinite;// no usage

	bool forGapSweeper = true;
	double hit_x, hit_y;

	efficientCCRayHittingPhysicalObstacle(seed, theta, 0, dis, sweep_dis_, 
		infinite, hit_x, hit_y, forGapSweeper);

	sweep_dis_ += 0.1;
	////////////////////////////////////////////////////////////////////////
	std::pair<double, double> o;
	o.first = cki_.first + sweep_dis_ * cos(theta);
	o.second = cki_.second + sweep_dis_ * sin(theta);

	getDetailedCCHittingObstacle(o.first, o.second, hitting_obstacles);

	if (!EXHAUSTIVE)
	{
		Sweeper my(Rindex_, Nindex_, Cindex_, cki_, o, ckiGcost_);

		// We check all intersections of gap sweepers at here
		for (auto iter = TheRobot(Rindex_)->sweeper_list_.begin(); iter != TheRobot(Rindex_)->sweeper_list_.end(); ++iter)
		{
			std::pair<double, double> q;
			bool b = findIntersection(my, *iter, q);
			if (b)
			{
				newSweeperTakeOverSweeper(Rindex_, iter->Nindex, iter->Cindex, Nindex_, Cindex_, q.first, q.second);
			}
		}

		auto* Rtemp = TheRobot(0);

		for (auto iter = TheRobot(Rindex_)->path_list_.begin(); iter != TheRobot(Rindex_)->path_list_.end(); ++iter)
		{
			std::pair<double, double> q;
			int loc;
			bool b = findIntersection(my, *iter, q, loc);

			if (b)
			{
				newPathTakeOverSweeper(Rindex_, Nindex_, Cindex_, iter->Nindex, iter->Cindex, q.first, q.second, loc);
			}

		}

		TheRobot(Rindex_)->sweeper_list_.push_back(my);
	}


	// Continue some registration
	
//	sweep_hitting_obstacles_ = hitting_obstacles[0];
	sweep_hitting_obstacles_.first = hitting_obstacles[0].first;
	sweep_hitting_obstacles_.second = hitting_obstacles[0].second;


	// Setup some angles
	// 2021.1.12 we find the hitting angle
	std::pair<double, double> hitting_diff = sweep_hitting_obstacles_ + std::pair<double, double>(0.5, 0.5) - pi_;
	double hitting_angle = atan2(hitting_diff.second, hitting_diff.first);

	std::pair<double, double> neardiff = near_obstacle_ - cki_;
	double anglenear = atan2(neardiff.second, neardiff.first);

	// Here we are different from MATLAB code on 2020.11.12, 
	// because we direct give value to sweep_hitting_obstacles_, 
	// so it is impossible to be empty
	std::pair<double, double> fardiff = sweep_hitting_obstacles_ + std::pair<double, double>(0.5, 0.5) - cki_;
	double anglefar = atan2(fardiff.second, fardiff.first);

	// Judge the start_angle and the end_angle
	if(normalize_angle(anglefar - anglenear) > 0)
	{
		start_angle_ = anglenear;
		double anglediff = normalize_angle_positive(anglefar - anglenear);
		end_angle_ = start_angle_ + anglediff;
	}
	else
	{
		start_angle_ = anglefar;
		double anglediff = normalize_angle_positive(anglenear - anglefar);
		end_angle_ = start_angle_ + anglediff;
	}

	// modify the obsexit
	if(isLeftOrRight())
	{
		me_.thetasmall = angle_short_ - normalize_angle_positive(angle_short_ - hitting_angle);
		me_.smallx = sweep_hitting_obstacles_.first + 0.5;
		me_.smally = sweep_hitting_obstacles_.second + 0.5;
		me_.dissmall = hypot(pi_.first - me_.smallx, pi_.second - me_.smally); 
	}
	else
	{
		me_.thetabig = angle_short_ + normalize_angle_positive(hitting_angle - angle_short_);
		me_.smallx = sweep_hitting_obstacles_.first + 0.5;
		me_.smally = sweep_hitting_obstacles_.second + 0.5;
		me_.dissmall = hypot(pi_.first - me_.smallx, pi_.second - me_.smally); 
	}

	return hitting_angle;
}

void Corridor::getPredecessors(std::vector<int>& fatherlink)
{
	// Compared to the MATLAB code, we create the vector inversely (and reverse it in the end)
	fatherlink.clear();
	// std::cout << "in getPredecessor: " << Cindex_ << ", " << Nindex_ << std::endl;
	fatherlink.push_back(Cindex_);
	fatherlink.push_back(Nindex_);
	while(fatherlink.back() != 0)
	{
		int temp = fatherlink.back();
		fatherlink.push_back(TheRobot(Rindex_)->N_.at(temp).soninfatherindex_);
		fatherlink.push_back(TheRobot(Rindex_)->N_.at(temp).fatherindex_);
		
	}

	std::reverse(fatherlink.begin(), fatherlink.end());
}

bool Corridor::isInT(const std::pair<double, double> query_point)
{
	// true: the point is in the T region
	// false: the point is not in the T region

	std::pair<double, double> diff = query_point - cki_;
	std::pair<double, double> obs_near = near_obstacle_ - cki_;
	double near_angle = atan2(obs_near.second, obs_near.first);
	double proj_near = diff.first * cos(near_angle) + diff.second * sin(near_angle);

	if(proj_near >= 1)
	{
		double proj_align = diff.first * cos(angle_long_) + diff.second * sin(angle_long_);
		if(proj_align >= 1)
			return true;
	}
	return false;
}

bool Corridor::isLeftOrRight()
{
	// true: left gap (turning left) (disbig is shorter than dissmall)
	// false: right gap (turning right) (disbig is longer than dissmall)
	double angle_diff = normalize_angle(angle_long_ - angle_short_);
	if(angle_diff > 0)
		return false;
	else
		return true;
}

void Corridor::pathSweeper()
{
	if(myPath_.data.empty())
		return ;


	Path my(Rindex_, Nindex_, Cindex_, pi_, cki_, piGcost_);
	// std::cout << "size of sweeper_list: " << TheRobot(Rindex_)->sweeper_list_.size() << std::endl;

	for(auto iter = TheRobot(Rindex_)->sweeper_list_.begin(); iter != TheRobot(Rindex_)->sweeper_list_.end(); ++iter)
	{
		std::pair<double, double> q;
		int loc;
		bool b = findIntersection(*iter, my, q, loc);
		if(b)
		{			 
			newPathTakeOverSweeper(Rindex_, iter->Nindex, iter->Cindex, Nindex_, Cindex_, q.first, q.second, loc);
		}
	}

	TheRobot(Rindex_)->path_list_.push_back(my);

}

void Corridor::removeOptimality()
{
	if(status_.compare("no_usage") == 0)
		return ;
	else if(status_.compare("open") == 0)
	{
		// removeCandidate(TheRobot(Rindex_)->openlist_, Nindex_, Cindex_);
		TheRobot(Rindex_)->openlist_.erase(
			std::remove_if(TheRobot(Rindex_)->openlist_.begin(), 
				TheRobot(Rindex_)->openlist_.end(), 
				[&](const Candidate& a){return a.Nindex == Nindex_ && a.Cindex == Cindex_;}), 
				TheRobot(Rindex_)->openlist_.end());
		status_ = "no_usage";
	}
	else if(status_.compare("expanded") == 0)
	{
		if(son_Nindex_ != -1)
			TheRobot(Rindex_)->N_.at(son_Nindex_).removeOptimality();
	}
}

void Corridor::removeUsage()
{
	if(status_.compare("no_usage") == 0)
	{
		status_ = "expanded";
	}
	else if(status_.compare("open") == 0)
	{
		status_ = "expanded";
		for(int i = TheRobot(Rindex_)->openlist_.size()-1; i >= 0; --i)
		{
			if(TheRobot(Rindex_)->openlist_.at(i).Nindex == Nindex_ && 
				TheRobot(Rindex_)->openlist_.at(i).Cindex == Cindex_)
			{
				TheRobot(Rindex_)->openlist_.erase(TheRobot(Rindex_)->openlist_.begin() + i);
				break;
			}
		}
	}
	else if(status_.compare("expanded") == 0)
	{
		TheRobot(Rindex_)->N_.at(son_Nindex_).removeUsage();
	}
}

bool Corridor::safePathFindingInWorld(std::vector<std::pair<int, int> >& phy_obstacle_list)
{
	// Find path within the given narrow corridor
	// If the safe path can be found, we save it
	// If the safe path cannot be found, return all hitting points


	// Before into detailed planning, we try straight path
	if(straightPath(pi_, cki_))
	{
		std::vector<std::pair<int, int> > P;
		classicBresenham(pi_.first, pi_.second, cki_.first, cki_.second, P);
		
		phy_obstacle_list.clear();
//		setPath(P);
		Path2D Ptemp;
		for (auto iter = P.begin(); iter != P.end(); ++iter)
		{
			Ptemp.data.push_back(std::pair<double, double>(iter->first + 0.5, iter->second + 0.5));
		}
		setPath(Ptemp);
		return true;
	}

	// Before into detailed planning, we try broken path
	if(straightPath(pi_fake_, cki_fake_))
	{
		std::vector<std::pair<int, int> > P;
		classicBresenham(pi_fake_.first, pi_fake_.second, cki_fake_.first, cki_fake_.second, P);

		Path2D Ptemp;
		Ptemp.data.reserve(P.size() + fake_pi_path_.data.size() + fake_cki_path_.data.size());
		for (auto iter = P.begin(); iter != P.end(); ++iter)
		{
			Ptemp.data.push_back(std::pair<double, double>(iter->first + 0.5, iter->second + 0.5));
		}
		Ptemp.data.insert(Ptemp.data.begin(), fake_pi_path_.data.begin(), fake_pi_path_.data.end());
		Ptemp.data.insert(Ptemp.data.end(), fake_cki_path_.data.begin(), fake_cki_path_.data.end());
		setPath(Ptemp);

		phy_obstacle_list.clear();
		return true;
	}

	double left, right;
	std::pair<double, double> leftpoint, rightpoint;
	// // set up affine frame
	footprintProjection(angle_long_, left, right, leftpoint, rightpoint);


	// new frame (not orthonormal frame)
	std::pair<double, double> dy(cos(angle_long_), sin(angle_long_));
	std::pair<double, double> dx, n_dx;
	double proj_dis;
	if(angle_long_ > angle_short_)
	{
		dx = std::pair<double, double>(cos(angle_long_ + M_PI / 2), sin(angle_long_ + M_PI / 2));
		proj_dis = (cki_- mid_).first*dx.first + (cki_- mid_).second*dx.second;
		n_dx = dx;
		dx = (cki_ - mid_)/proj_dis;
	}
	else
	{
		dx = std::pair<double, double>(cos(angle_long_-M_PI/2), sin(angle_long_-M_PI/2));
		proj_dis = (cki_ - mid_).first*dx.first + (cki_-mid_).second*dx.second;
		n_dx = dx;
		dx = (cki_ - mid_)/proj_dis;
	}

	double xposmax, xnegmax, ymax;
	// The size of the costmap must be judged based on distance, not affine frame
	if(angle_long_ > angle_short_)
	{
		xposmax = ceil(right)+1;
		xnegmax = ceil(left)+1;
		// xposmax = ceil(right);
		// xnegmax = ceil(left);
		ymax = ceil(norm(mid_ - pi_))+1;
		xposmax_ = xposmax;
	}
	else
	{
		xposmax = ceil(left)+1;
		xnegmax = ceil(right)+1;
		// xposmax = ceil(left);
		// xnegmax = ceil(right);
		ymax = ceil(norm(mid_ - pi_))+1;
		xposmax_ = xposmax;
	}
	auto origin = pi_;

	// We do some checking to be sure that the path planning can succeed

	//if(!isInNarrowCorridor(floor(pi_), pi_, dx, dy, n_dx, xposmax, xnegmax, ymax))
	//{
	//	// std::cout << "Float-number error: we modify the boundary of region for starting point" << std::endl;

	//	std::pair<double, double> d;
	//	d.first = floor(pi_.first) - origin.first;
	//	d.second = floor(pi_.second) - origin.second;

	//	std::pair<double, double> p;
	//	p.first = d.first*n_dx.first + d.second*n_dx.second;

	//	// check local x coordinate
	//	if(p.first < 0 && fabs(p.first) > xnegmax)
	//	{
	//		xnegmax = fabs(p.first) + 0.1;
	//	}
	//	if(p.first > 0 && fabs(p.first) > xposmax)
	//	{
	//		xposmax = fabs(p.first) + 0.1;
	//	}

	//	std::pair<double, double> newworld_p;
	//	newworld_p.first = floor(pi_.first) - p.first*dx.first - origin.first;
	//	newworld_p.second = floor(pi_.second) - p.first*dx.second - origin.second;

	//	// check local y coordinate
	//	p.second = newworld_p.first*dy.first + newworld_p.second*dy.second;
	//	if(p.second < -1)
	//	{
	//		origin = origin + p.second*dy;
	//		ymax += p.second;
	//		// std::cout << "We cannot change -1" << std::endl;
	//	}
	//	if(p.second > ymax)
	//	{
	//		ymax = p.second + 0.1;
	//	}
	//	

	//	// std::cout << "cki: " << cki_.first << ", " << cki_.second << std::endl;
	//	// std::cout << "pi_: " << pi_.first << ", " << pi_.second << std::endl;
	//	// std::cout << "origin: " << origin.first << ", " << origin.second << std::endl;
	//	// std::cout << "dx: " << dx.first << ", " << dx.second << std::endl;
	//	// std::cout << "dy: " << dy.first << ", " << dy.second << std::endl;
	//	// std::cout << "n_dx: " << n_dx.first << ", " << n_dx.second << std::endl;
	//	// std::cout << "xposmax: " << xposmax << std::endl;
	//	// std::cout << "xnegmax: " << xnegmax << std::endl;
	//	// std::cout << "ymax: " << ymax << std::endl;
	//	// std::cout << "left: " << left << std::endl;
	//	// std::cout << "right: " << right << std::endl;
	//	// std::cout << "dx"
	//	// std::cout << "YT: error, the start is not in the region" << std::endl;
	//}

	if(!isInNarrowCorridor(floor(cki_), pi_, dx, dy, n_dx, xposmax, xnegmax, ymax))
	{
		// std::cout << "YT: error, the goal is not in the region" << std::endl;

		// std::cout << "Float-number error: we modify the boundary of region for ending point" << std::endl;

		std::pair<double, double> d;
		d.first = floor(cki_.first) - origin.first;
		d.second = floor(cki_.second) - origin.second;

		std::pair<double, double> p;
		p.first = d.first*n_dx.first + d.second*n_dx.second;

		// check local x coordinate
		if(p.first < 0 && fabs(p.first) > xnegmax)
		{
			xnegmax = fabs(p.first) + 0.1;
		}
		if(p.first > 0 && fabs(p.first) > xposmax)
		{
			xposmax = fabs(p.first) + 0.1;
		}

		std::pair<double, double> newworld_p;
		newworld_p.first = floor(cki_.first) - p.first*dx.first - origin.first;
		newworld_p.second = floor(cki_.second) - p.first*dx.second - origin.second;

		// check local y coordinate
		p.second = newworld_p.first*dy.first + newworld_p.second*dy.second;
		if(p.second < -1)
		{
			std::cout << "We cannot change -1" << std::endl;
		}
		if(p.second > ymax)
		{
			ymax = p.second + 0.1;
		}
	}
	

	// We create a rectangle area for planning
	// In affine frame, the coordinate will be [-xnegmax, xposmax]*[-0.5, ymax]
	// we find the four corners and inflate
	int xminvalue = std::numeric_limits<int>::max();
	int xmaxvalue = 0;
	int yminvalue = std::numeric_limits<int>::max();
	int ymaxvalue = 0;
	std::pair<double, double> p1 = origin - rightpoint/xnegmax*(xnegmax+0.5);
	if(xminvalue > floor(p1.first+0.5))
		xminvalue = floor(p1.first + 0.5);
	if(yminvalue > floor(p1.second + 0.5))
		yminvalue = floor(p1.second + 0.5);
	if(xmaxvalue < ceil(p1.first + 0.5))
		xmaxvalue = ceil(p1.first + 0.5);
	if(ymaxvalue < ceil(p1.second + 0.5))
		ymaxvalue = ceil(p1.second + 0.5);
	std::pair<double, double> p2 = origin - leftpoint/xposmax*(xposmax+0.5);
	if(xminvalue > floor(p2.first + 0.5))
		xminvalue = floor(p2.first + 0.5);
	if(yminvalue > floor(p2.second + 0.5))
		yminvalue = floor(p2.second + 0.5);
	if(xmaxvalue < ceil(p2.first + 0.5))
		xmaxvalue = ceil(p2.first + 0.5);
	if(ymaxvalue < ceil(p2.second + 0.5))
		ymaxvalue = ceil(p2.second + 0.5);
	std::pair<double, double> p3 = origin - rightpoint/xnegmax*(xnegmax+0.5) + dy*ymax;
	if(xminvalue > floor(p3.first + 0.5))
		xminvalue = floor(p3.first + 0.5);
	if(yminvalue > floor(p3.second + 0.5))
		yminvalue = floor(p3.second + 0.5);
	if(xmaxvalue < ceil(p3.first + 0.5))
		xmaxvalue = ceil(p3.first + 0.5);
	if(ymaxvalue < ceil(p3.second + 0.5))
		ymaxvalue = ceil(p3.second + 0.5);
	std::pair<double, double> p4 = origin - leftpoint/xposmax*(xposmax+0.5) + dy*ymax;
	if(xminvalue > floor(p4.first + 0.5))
		xminvalue = floor(p4.first + 0.5);
	if(yminvalue > floor(p4.second + 0.5))
		yminvalue = floor(p4.second + 0.5);
	if(xmaxvalue < ceil(p4.first + 0.5))
		xmaxvalue = ceil(p4.first + 0.5);
	if(ymaxvalue < ceil(p4.second + 0.5))
		ymaxvalue = ceil(p4.second + 0.5);

	// std::cout << "p1: " << p1.first << ", " << p1.second << std::endl;
	// std::cout << "p2: " << p2.first << ", " << p2.second << std::endl;
	// std::cout << "p3: " << p3.first << ", " << p3.second << std::endl;
	// std::cout << "p4: " << p4.first << ", " << p4.second << std::endl;

	xminvalue -= 1;
	yminvalue -= 1;
	xmaxvalue += 1;
	ymaxvalue += 1;

	int xsize = xmaxvalue - xminvalue;
	int ysize = ymaxvalue - yminvalue;

	std::pair<int, int> offset(xminvalue, yminvalue);

	RestrictedAstar A(offset, xsize, ysize,
		std::pair<int, int>(floor(pi_.first), floor(pi_.second)), 
		std::pair<int, int>(floor(cki_.first), floor(cki_.second)), 
		origin, dx, dy, n_dx, xposmax, xnegmax, ymax);
	// std::cout << "Outside RestrictedAstar" << std::endl;

	phy_obstacle_list.assign(A.newly_found_obstacle_in_global_frame_.begin(), 
		A.newly_found_obstacle_in_global_frame_.end());

	// std::cout << "size of phy_obstacle_list in A*: " << phy_obstacle_list.size() << std::endl;
	// std::cout << "size of path in A*: " << A.path_in_global_frame_.size() << std::endl;

	if(!A.path_in_global_frame_.empty())
	{
//		setPath(A.path_in_global_frame_);
		auto& P = A.path_in_global_frame_;
		Path2D Ptemp;
		for (auto iter = P.begin(); iter != P.end(); ++iter)
		{
			Ptemp.data.push_back(std::pair<double, double>(iter->first + 0.5, iter->second + 0.5));
		}
		setPath(Ptemp);

		return true;
	}
	else
	{
		setPath();
		return false;
	}

}

void Corridor::setCost()
{
	myLength_ = pathLength(myPath_.data);
	ckiGcost_ = piGcost_ + myLength_;

//	ckiFcost_ = ckiGcost_ + hypot((::goals[0]).first - cki_.first, (::goals[0]).second - cki_.second);
	ckiFcost_.resize(::goals.size(), 0);
	for (unsigned int i = 0; i < ::goals.size(); ++i)
	{
//		ckiFcost_[i] = ckiGcost_ + hypot((::goals[i]).first - cki_.first, (::goals[i]).second - cki_.second);
		ckiFcost_[i] = ckiGcost_ + sqrt(((::goals[i]).first - cki_.first)*((::goals[i]).first - cki_.first) + ((::goals[i]).second - cki_.second)*((::goals[i]).second - cki_.second));
	}
}

//void Corridor::setPath(const std::vector<std::pair<int, int> >& newpath)
void Corridor::setPath(const Path2D& newpath)
{
//	myPath_.data.assign(newpath.begin(), newpath.end());
	myPath_.data = newpath.data;
}
void Corridor::setPath()
{
	myPath_.data.clear();
}
void Corridor::tracePath(int untilfather, Path2D& result)
{
	TheRobot(Rindex_)->N_.at(Nindex_).tracePath(result, untilfather);
	result.data.insert(result.data.end(), ++(myPath_.data.begin()), myPath_.data.end());
}