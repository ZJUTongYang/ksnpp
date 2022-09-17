#include "Node.h"
#include "Restricted_astar.h"
#include "Robot.h"

#include <corecrt_math_defines.h>

#define ANGLE_EPS 0.0000001

Node::Node(int Rindex, int nodeindex, std::pair<double, double> seedpoint)
{
	double start_angle = 0;
	double end_angle = 2*M_PI;
	double Gcost = 0;
	int fatherindex = -1;
	int soninfatherindex = -1;

	*this = Node(Rindex, nodeindex, seedpoint, start_angle, end_angle, Gcost, 
		fatherindex, soninfatherindex);
}

Node::Node(int Rindex, int nodeindex, std::pair<double, double> seedpoint, 
		double start_angle, double end_angle, double Gcost, int fatherindex, 
		int soninfatherindex)
{

	Rindex_ = Rindex;
	Nindex_ = nodeindex;
	seed_ = seedpoint;

	point_ori_dis_array_.clear();

	if(nodeindex != 0)	
	{
		start_angle_ = normalize_angle(start_angle);
		end_angle_ = start_angle_ + normalize_angle_positive(end_angle - start_angle);
	}
	else
	{
		start_angle_ = start_angle;
		end_angle_ = end_angle;
	}


	Gcost_ = Gcost;
	fatherindex_ = fatherindex;
	soninfatherindex_ = soninfatherindex;

	if(fatherindex >= 0)
	{
		TheRobot(Rindex)->N_.at(fatherindex).C_.at(soninfatherindex).son_Nindex_ = Nindex_;
		myPath_.data.assign(TheRobot(Rindex)->N_.at(fatherindex).C_.at(soninfatherindex).myPath_.data.begin(), 
			TheRobot(Rindex)->N_.at(fatherindex).C_.at(soninfatherindex).myPath_.data.end());
	}
}


void Node::ccRayExplore()
{
	// In this function, each ray is a line segment in the Cspace
	// Create Basic Shape
	//std::cout << "create basic shapes" << std::endl;
	if(end_angle_ - start_angle_ > M_PI)
	{
		OriDis temp;
		temp.theta = start_angle_;
		point_ori_dis_array_.push_back(temp);
		temp.theta = (start_angle_*3+end_angle_)/4;
		point_ori_dis_array_.push_back(temp);
		temp.theta = (start_angle_+end_angle_)/2;
		point_ori_dis_array_.push_back(temp);
		temp.theta = (start_angle_+end_angle_*3)/4;
		point_ori_dis_array_.push_back(temp);
		temp.theta = end_angle_;
		point_ori_dis_array_.push_back(temp);
	}
	else
	{
		OriDis temp;
		temp.theta = start_angle_;
		point_ori_dis_array_.push_back(temp);
		temp.theta = end_angle_;
		point_ori_dis_array_.push_back(temp);
	}
	//std::cout << "finish create basic shapes" << std::endl;

	// Calculate the length of the rays and calculate the endpoint
	for(auto iter = point_ori_dis_array_.begin(); iter != point_ori_dis_array_.end(); ++iter)
	{
		// 2022.4.30 We consider the difference between C++ and MATLAB
		//std::pair<double, double> seed_temp(seed_.first+0.5, seed_.second+0.5);
		// We use double precision in C++
		std::pair<double, double> seed_temp(seed_.first, seed_.second);


		efficientCCRayHittingPhysicalObstacle(
			seed_temp, iter->theta, 0, iter->dis, 
			iter->Cdis, iter->infinite, iter->endx, iter->endy);

		iter->start_dis = 0;	
	}

	// Refine the rays
	refineCCRay();

	// 2021.3.12 We filter the gaps. If the short ray of two gaps are near enough, we remove all rays in between
	// TODO: maybe there is problem when some rays are labelled as exploited tags. 
	
	//for(auto iter1 = point_ori_dis_array_.begin(); iter1 != std::prev(point_ori_dis_array_.end(), 2); ++iter1)
	//{
	//	auto next1 = std::next(iter1);
	//	if(iter1->dis < next1->dis && normRay(*iter1, *next1) > 2*LETHAL_RADIUS)
	//	{
	//		for(auto iter2 = std::next(iter1, 2); iter2 != point_ori_dis_array_.end(); ++iter2)
	//		{
	//			auto prev2 = std::prev(iter2);
	//			if(iter2->dis < prev2->dis && normRay(*iter2, *prev2) > 2* LETHAL_RADIUS)
	//			{
	//				if(normRay(*iter1, *iter2) < 2* LETHAL_RADIUS)
	//				{
	//					point_ori_dis_array_.erase(next1, iter2);
	//					// stable = false;
	//					iter1 = iter2;
	//					break;
	//				}
	//			}
	//		}
	//		if(iter1 == --point_ori_dis_array_.end())
	//			break;
	//	}
	//}

	auto iter1 = point_ori_dis_array_.begin();
	while(iter1 != std::prev(point_ori_dis_array_.end(), 2))
	{
		auto next1 = std::next(iter1);
		if (iter1->dis < next1->dis && normRay(*iter1, *next1) > 2 * LETHAL_RADIUS)
		{
			for (auto iter2 = std::next(iter1, 2); iter2 != point_ori_dis_array_.end(); ++iter2)
			{
				auto prev2 = std::prev(iter2);
				if (iter2->dis < prev2->dis && normRay(*iter2, *prev2) > 2 * LETHAL_RADIUS)
				{
					if (normRay(*iter1, *iter2) < 2 * LETHAL_RADIUS)
					{
						point_ori_dis_array_.erase(next1, iter2);
//						iter1 = iter2;
						break;
					}
				}
			}
			if (iter1 == std::prev(point_ori_dis_array_.end(), 2))
				break;
		}
		++iter1;
	}


}


std::vector<ObsExit> Node::collectObsExit(std::vector<int>& obs_exit_index)
{

	std::vector<ObsExit> obs_exit;
	obs_exit_index.clear();

	int count = 0;
	for(auto iter = point_ori_dis_array_.begin(); iter != --point_ori_dis_array_.end(); ++iter, ++count )
	{
		if(iter->smallGap == 1)
			continue;

		auto next = std::next(iter);
		if(normRay(*iter, *next) > 2* LETHAL_RADIUS)
		{
			ObsExit temp;
			temp.thetasmall = iter->theta;
			temp.dissmall = iter->dis;
			temp.smallx = iter->endx;
			temp.smally = iter->endy;
			temp.thetabig = next->theta;
			temp.disbig = next->dis;
			temp.bigx = next->endx;
			temp.bigy = next->endy;
			obs_exit.push_back(temp);
			obs_exit_index.push_back(count);
		}
	}

	return obs_exit;
}


void Node::dealWithHiddenObstacle(const std::vector<std::pair<int, int> >& obs_list)
{
	// Based on the newly inserted rays, we find all new exit 
	// TODO: we may need to iteratively run "create rays" and "refind rays"
	// because the new exit may also invalid (i.e., cannot pass basic checking)

	// We make remove redundant obstacles 
	// 2021.1.21 After we mark the obstacles in RestrictedAstar, there should
	// not exist redundant obstacles

	// 2021.3.11 After this function, obs_list should be empty

	if(obs_list.empty())
		return ;

	// we first order all obstacles
	std::vector<double> P;
	for(unsigned int i = 0; i < obs_list.size(); i++)
	{
		double theta = atan2(obs_list.at(i).second + 0.5 - seed_.second, obs_list.at(i).first + 0.5 - seed_.first);
		theta = (theta > start_angle_)? theta : theta + 2*M_PI;

		// we should delete the obstacles which are beyond the node
		if(theta < end_angle_)
			P.push_back(theta);
	}

	if(P.empty())
		return ;

	std::list<OriDis> newrays;
	//if(P.size() >= 2)
	//{
		// sort them based on the angle
		std::sort(P.begin(), P.end());

		OriDis temp;
		temp.dis = -1;
		newrays.resize(P.size(), temp);
		int i = 0; 
		for (auto iter = newrays.begin(); iter != newrays.end(); ++iter)
		{
			iter->theta = P[i++];
		}

	//}
	//else
	//{
	//	OriDis temp;
	//	temp.theta = P[0];
	//	temp.dis = -1;
	//	newrays.push_back(temp);
	//}

	if(P.size() > 1)
		i = newrays.size();

	point_ori_dis_array_.merge(newrays, [](const OriDis& a, const OriDis& b){return a.theta < b.theta;});

	auto iter = point_ori_dis_array_.begin();
	auto next = iter;
	auto prev = iter;
	for(iter = point_ori_dis_array_.begin(); iter != --point_ori_dis_array_.end(); ++iter)
	{
		if(iter->dis > 0)
			continue;

		next = std::next(iter);
		while(next->dis < 0)
			++next;

		prev = std::prev(iter);
		while(prev->dis < 0)
			--prev;
		
		// 2021.3.11 If the ray is inserted into a (checked) gap, we omit it
		if(prev->smallGap != 0)
		{
			point_ori_dis_array_.erase(iter);
			iter = prev;
			continue;
		}

		// We won't insert rays that are too near the existing rays. 
		// It will cause problem when we find the location of the existing gaps in the ray list
		if(iter->theta - prev->theta < ANGLE_EPS || next->theta - iter->theta < ANGLE_EPS)
		{
			point_ori_dis_array_.erase(iter);
			iter = prev;
			continue;
		}

		

		iter->start_dis = calStartDis(prev->theta, 
			prev->dis, prev->Cdis, 
			next->theta, next->dis, 
			next->Cdis, LETHAL_RADIUS);

		// 2022.4.30 We consider the difference between C++ and MATLAB
		//std::pair<double, double> seed_temp(seed_.first + 0.5, seed_.second + 0.5);
		std::pair<double, double> seed_temp(seed_.first, seed_.second);

		efficientCCRayHittingPhysicalObstacle(seed_temp, iter->theta, iter->start_dis, iter->dis, 
			iter->Cdis, iter->infinite, iter->endx, iter->endy);			
	}

	refineCCRay();

}


void Node::Expand()
{

	std::vector<ObsExit> obs_exit;
	std::vector<int> obs_exit_index;
	ccRayExplore();
	while(1)
	{
		//std::cout << "collect obsexit" << std::endl;
		obs_exit = collectObsExit(obs_exit_index);
		if(obs_exit.empty())
		{
			break;
		}

		int near_obs_exit_index = -1; // The position of the nearest gap in the ray list
		ObsExit nearObsExit = chooseOneObsExit(obs_exit, obs_exit_index, near_obs_exit_index);
		if(near_obs_exit_index == -1)
		{
			std::cout << "We cannot locate the nearest ray in the ray list" << std::endl;
			break;
		}

		// std::cout << "Size of C: " << C_.size() << std::endl;

		// mark this 
		markTheIthRay(near_obs_exit_index);

		Corridor newC(Rindex_, Nindex_, -1, seed_, nearObsExit, Gcost_);
		int newCindex = insertCorridor(newC);
		std::vector<std::pair<int, int> > obs_list;
		obs_list.clear();

		if(!C_.at(newCindex).basicChecking(obs_list))
		{
			removeCorridor(newCindex);
			dealWithHiddenObstacle(obs_list);
			obs_list.clear();
		}
		else
		{
			dealWithHiddenObstacle(obs_list);
			obs_list.clear();
			//std::cout << "before safePathFindingInWorld" << std::endl;
			if(C_.at(newCindex).safePathFindingInWorld(obs_list))
			{
				// We unique the obs_list
				std::sort(obs_list.begin(), obs_list.end(), [](auto& a, auto& b)
				{return (a.first < b.first) || (a.first == b.first && a.second < b.second); }
				);
				obs_list.erase(std::unique(obs_list.begin(), obs_list.end(),
					[](auto& a, auto& b) {return a.first == b.first &&a.second == b.second; }), obs_list.end());

				C_.at(newCindex).setCost();
				//std::cout << "before gapsweeper" << std::endl;
				
				auto* Rtemp = TheRobot(0);
				
				double hitting_angle = C_.at(newCindex).gapSweeper();
				
				auto* Rtemp2 = TheRobot(0);

				//std::cout << "before pathsweeper" << std::endl;
				if (!EXHAUSTIVE)
				{
					C_.at(newCindex).pathSweeper();
				}

				

				hitting_angle = start_angle_ + normalize_angle_positive(hitting_angle - start_angle_);

				double angle_near = C_.at(newCindex).angle_short_;
				angle_near = start_angle_ + normalize_angle_positive(angle_near - start_angle_);

				double start_angle = std::min(angle_near, hitting_angle);
				double end_angle = std::max(angle_near, hitting_angle);

				//std::cout << "before removing rays" <<std::endl;

				// mark the gap
				auto iter = point_ori_dis_array_.begin();
				if (iter->theta <= start_angle && std::next(iter)->theta > start_angle)
				{
					iter->smallGap = 1;
					++iter;
				}
				else 
				{
					for (; iter != point_ori_dis_array_.end(); ++iter)
					{
						if (iter->theta < start_angle)
							continue;
						std::prev(iter)->smallGap = 1;
						break;
					}
				}

				for(; iter != point_ori_dis_array_.end();)
				{
					if(iter->theta < start_angle)
					{
						std::cout << "YT: iter->theta should be greater than start_angle" << std::endl;
					}
					if(iter->theta < end_angle)
					{
						point_ori_dis_array_.erase(iter++);
					}
					else
					{
						iter++;
					}
				}
				
				//2021.3.11 obs_list should be empty here

				obs_list.push_back(C_.at(newCindex).sweep_hitting_obstacles_);
				// std::cout << "deal with obstacles after path has been found: " << phy_obs_list.size() << std::endl;
				dealWithHiddenObstacle(obs_list);
				obs_list.clear();
			}
			else
			{
				// We unique the obs_list
				std::sort(obs_list.begin(), obs_list.end(), [](auto& a, auto& b)
				{return (a.first < b.first) || (a.first == b.first && a.second < b.second); }
				);
				obs_list.erase(std::unique(obs_list.begin(), obs_list.end(),
					[](auto& a, auto& b) {return a.first == b.first &&a.second == b.second; }), obs_list.end());


				std::cout << "There is no path for the new corridor " << std::endl;
				if(!obs_list.empty())
				{


					std::cout << "before remove COrridors" << std::endl;
					removeCorridor(newCindex);
					// If there are new corridors added, we must make them following the angle (increasing) order
					dealWithHiddenObstacle(obs_list);
					obs_list.clear();
				}
				else
				{
					std::cout << "YT: There should be some obstacles." << std::endl;
				}
			}
		}
	}
	//visualizeRays(seed_, point_ori_dis_array_, Nindex_);

	//for(auto iter = C_.begin(); iter != C_.end(); iter++)
	//{
	//	std::cout << "corridors: (" << iter->cki_.first  
	//	<< ", " << iter->cki_.second << "), " << iter->start_angle_ << ", " << iter->end_angle_ << std::endl;	
	//}

}


int Node::insertCorridor(const Corridor& newCorridor)
{
	// We needn't change index of any existing corridors
	C_.push_back(newCorridor);
	C_.back().Cindex_ = C_.size()-1;
	return C_.size()-1;
}

bool Node::isInNode(const std::pair<double, double>& query_point, int& Cindex)
{
	// TODO: change to more precise checking function
	std::pair<double, double> diff = query_point - seed_;
	double theta = atan2(diff.second, diff.first);
	double dis = hypot(diff.first, diff.second);
	double angle_diff = normalize_angle_positive(theta - start_angle_);
	theta = start_angle_ + angle_diff;

	// We check the orientation
	double total_diff;
	if (Nindex_ == 0)
	{
		total_diff = 2 * M_PI;
	}
	else
	{
		total_diff = normalize_angle_positive(end_angle_ - start_angle_);
	}

	//std::cout << "isInNode: total_diff = " << total_diff << ", angle_diff = " << angle_diff<< ", dis = " << dis << std::endl;

	if(total_diff < angle_diff && dis > 1)
	{
		// TODO: for resolutional complete, there may be some exceptional case, 
		// Leave for further improvement

		Cindex = -1;
		return false;
	}

	// We check the distance 
	// Assume that the rays are actually dense, we directly compare the length

	auto loc = point_ori_dis_array_.begin();


	for( ; loc != --point_ori_dis_array_.end(); ++loc)
	{
		if(loc->theta < theta && std::next(loc)->theta > theta)
			break;
	}
	if(loc == --point_ori_dis_array_.end())
	{
		std::cout << "error: we cannot locate the query point in the ray list" << std::endl;
		return false;
	}

	auto next = std::next(loc);
	if(loc->dis > dis && next->dis > dis)
	{
		Cindex = -1;
		return true;
	}
	else if(loc->dis < dis && next->dis < dis)
	{
		Cindex = -1;
		return false;
	}
	
	// TODO: precisely check the triangle

	Cindex = -1;
	if(!C_.empty())
	{
		for(unsigned int i = 0; i < C_.size(); ++i)
		{
			if(C_.at(i).isInT(query_point))
			{
				Cindex = i;
				break;
			}
		}
	}

}

void Node::markTheIthRay(int i)
{
	int count = 0;
	for(auto iter = point_ori_dis_array_.begin(); iter != --point_ori_dis_array_.end(); ++iter, ++count)
	{
		if(count != i)
			continue;
		iter->smallGap = 1;
		break;
	}
	if(count != i)
		std::cout << "YT: error, we should mark one ray" << std::endl;
}

void Node::refineCCRay()
{
	refineCCRay(point_ori_dis_array_.begin(), point_ori_dis_array_.end());
}

void Node::refineCCRay(const std::list<OriDis>::iterator& begin, const std::list<OriDis>::iterator& end)
{

	auto iter = begin;
	while(iter != std::prev(end))
	{
		auto next = std::next(iter);
		if (preciseEnough(*iter, *next) || iter->smallGap == 1)
		{
			++iter;
			continue;
		}

	

		if(normRay(*iter, *next) > 2* LETHAL_RADIUS)
		{
				
			OriDis temp;
			temp.theta = (next->theta + iter->theta)/2;
			temp.start_dis = calStartDis(iter->theta, iter->dis, 
				iter->Cdis, next->theta, 
				next->dis, next->Cdis, 
				LETHAL_RADIUS);

			// 2022.4.30 We consider the difference between C++ and MATLAB
			//std::pair<double, double> seed_temp(seed_.first + 0.5, seed_.second + 0.5);
			std::pair<double, double> seed_temp(seed_.first, seed_.second);

			efficientCCRayHittingPhysicalObstacle(seed_temp, temp.theta, temp.start_dis, 
				temp.dis, temp.Cdis, temp.infinite, temp.endx, temp.endy);

			point_ori_dis_array_.insert(next, temp);

		}
		else
		{
			++iter;
		}
		
	}


	//bool stable = false;
	//while(stable == false)
	//{
	//	stable = true;

	//	auto iter = begin;
	//	auto next = std::next(iter);

	//	// for(int i = point_ori_dis_array_.size()-2; i>= 0; --i)
	//	for( ; iter != std::prev(end); ++iter)
	//	{
	//		next = std::next(iter);
	//		if(preciseEnough(*iter, *next) || iter->smallGap == 1)
	//		{
	//			continue;
	//		}
	//		if(normRay(*iter, *next) > 2* LETHAL_RADIUS)
	//		{
	//			
	//			OriDis temp;
	//			temp.theta = (next->theta + iter->theta)/2;
	//			temp.start_dis = calStartDis(iter->theta, iter->dis, 
	//				iter->Cdis, next->theta, 
	//				next->dis, next->Cdis, 
	//				LETHAL_RADIUS);

	//			// 2022.4.30 We consider the difference between C++ and MATLAB
	//			//std::pair<double, double> seed_temp(seed_.first + 0.5, seed_.second + 0.5);
	//			std::pair<double, double> seed_temp(seed_.first, seed_.second);

	//			efficientCCRayHittingPhysicalObstacle(seed_temp, temp.theta, temp.start_dis, 
	//				temp.dis, temp.Cdis, temp.infinite, temp.endx, temp.endy);

	//			point_ori_dis_array_.insert(next, temp);

	//			stable = false;
	//		}
	//	}
	//	
	//}
}

void Node::removeCorridor(int index)
{
	// Remove one corridor and update the index of other corridor
	// Here we cannot use the lazy function, or else we have to update the openlist also
	
	C_.erase(C_.begin() + index);

	if(index == C_.size())
		return ;

	for(int i = index; i < C_.size(); ++i)
	{
		C_.at(i).Cindex_ = i;
	}

}

void Node::removeOptimality()
{
	if(C_.empty())
		return ;

	for(unsigned int i = 0; i < C_.size(); ++i)
	{
		C_.at(i).removeOptimality();
	}
}

void Node::removeUsage()
{
	if(C_.empty())
		return ;
		
	for(unsigned int i = 0; i < C_.size(); ++i)
	{
		C_.at(i).removeUsage();
	}
}

int Node::safePathFinding(std::pair<double, double> goal, Path2D& current_path)
{
	int Cindex;
	bool b = isInNode(goal, Cindex);

	if(b)
	{
		//std::cout << "goal is in node" << std::endl;
		if(Cindex == -1)
		{
			// The goal is in the node, we find the path
			// This is a path from the robot's current position

			// We use straight path trial
			if(straightPath(seed_, goal))
			{
				//std::cout << "straight final path is valid" << std::endl;
				std::vector<std::pair<int, int> > Ptemp;
				classicBresenham(seed_.first, seed_.second, goal.first, goal.second, Ptemp);
				std::vector<std::pair<double, double> > P;
				P.reserve(Ptemp.size());
				for (auto iter = Ptemp.begin(); iter != Ptemp.end(); ++iter)
				{
					P.push_back(std::pair<double, double>(iter->first + 0.5, iter->second + 0.5));
				}

				tracePath(current_path, 0);
				current_path.data.insert(current_path.data.end(), P.begin()+1, P.end());

#if NON_SELFCROSSING
				if(isSelfIntersect(current_path))
				{
					current_path.data.clear();
					return 0;
				}
#endif

				return 1;
			}

			//std::cout << "we do normal A* for final path planning" << std::endl;
			double left, right;
			double theta = atan2(goal.second - seed_.second, goal.first - seed_.first);
			std::pair<double, double> leftpoint, rightpoint;
			// // set up affine frame
			footprintProjection(theta, left, right, leftpoint, rightpoint);

			double xposmax, xnegmax, ymax;
			std::pair<double, double> origin, dx, dy, n_dx;
			constructFakeCorridorParams(seed_, goal, origin, dx, dy, n_dx, xposmax, xnegmax, ymax);


			// Setup the boundary for path planning
			int xminvalue = std::numeric_limits<int>::max();
			int xmaxvalue = 0;
			int yminvalue = std::numeric_limits<int>::max();
			int ymaxvalue = 0;
			std::pair<double, double> p1 = origin - rightpoint/xnegmax*(xnegmax+0.5);
			if(xminvalue > floor(p1.first + 0.5))
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

			xminvalue -= 1;
			yminvalue -= 1;
			xmaxvalue += 1;
			ymaxvalue += 1;


			int xsize = xmaxvalue - xminvalue;
			int ysize = ymaxvalue - yminvalue;
			std::pair<int, int> offset(xminvalue, yminvalue);



			RestrictedAstar A(offset, xsize, ysize, floor(seed_), floor(goal), 
				origin, dx, dy, n_dx, xposmax, xnegmax, ymax);



			if(!A.path_in_global_frame_.empty())
			{
				auto& P = A.path_in_global_frame_;
				Path2D Ptemp;
				for (auto iter = P.begin(); iter != P.end(); ++iter)
				{
					Ptemp.data.push_back(std::pair<double, double>(iter->first + 0.5, iter->second + 0.5));
				}

				tracePath(current_path, 0);
				current_path.data.insert(current_path.data.end(), Ptemp.data.begin(), Ptemp.data.end());
				return 1;
			}
			else
			{
				current_path.data.clear();
				return -1;
			}
		}
		else
		{
			// The goal is in one of the narrow corridor, we just try it in the narrow corridor

			// Current, we do nothing

			// TODO: find path within T(it is tricky, but we would like to do so)
			current_path.data.clear();
			return 0;
		}
	}
	else
	{
		//std::cout << "goal is not in node" << std::endl;
		current_path.data.clear();
		return 0;
	}
}

void Node::tracePath(Path2D& result, int untilfather)
{
	// 2022.6.21 This tracePath function does not make any modification to the path (such as add 0.5)
	result.data.clear();

	if(Nindex_ == 0)
	{
		result.data.push_back(start_location);
		return ;
	}
	
	int i = Nindex_;
	int father, son_in_father;
	while(i != untilfather)
	{
		father = TheRobot(Rindex_)->N_.at(i).fatherindex_;
		son_in_father = TheRobot(Rindex_)->N_.at(i).soninfatherindex_;
		result.data.insert(result.data.begin(),
			TheRobot(Rindex_)->N_.at(father).C_.at(son_in_father).myPath_.data.begin()+1, 
			TheRobot(Rindex_)->N_.at(father).C_.at(son_in_father).myPath_.data.end());
		i = father;
	}
	result.data.insert(result.data.begin(), start_location);
}