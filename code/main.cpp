#include <iostream>
#include "Atom.h"
#include "Definitions.h"
#include "Robot.h"
#include "main.h"
#include <fstream>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include "Kim_node.h"
#include "Kim_alg.h"
#include "optimal_tmv_functions.h"

using namespace std;

int requestMap(int x, int y)
{
	return costmap_.data[y*MAX_X_SIZE + x];
}

void showMap(const Costmap& p, std::string window_name)
{
	// Since this visualization is intrinsic to opencv, we write it here. 
	// Should the costmap be implemented by other interfaces, just discard this function
	
	cv::Mat img(MAX_X_SIZE, MAX_Y_SIZE, CV_8UC1);

	for (unsigned int row = 0; row < MAX_Y_SIZE; ++row)
	{
		uchar *current_row = img.ptr<uchar>(row);
		for (unsigned int col = 0; col < MAX_X_SIZE; ++col)
		{
			*(current_row + col) = p.data[(MAX_Y_SIZE - 1 - row)*MAX_X_SIZE + col]/255.0*100.0+155;
		}
	}
	imshow(window_name, img);
}


void tryToFindSolutions(int Rindex, int goal_index)
{
	// We only check the last node in the tree, 
	// so we don't need to input its node index

	int Nindex = TheRobot(Rindex)->N_.size() - 1;

	// 2022.06.16 If there has been found a resulting path in the parent of the
	// newly expanded node, we need not find it again in the child node.
	// (The parent node and the son node are inevitably overlapped a little)
	if (std::find(multi_goal_multi_nindex_for_solutions[goal_index].begin(),
		multi_goal_multi_nindex_for_solutions[goal_index].end(), TheRobot(Rindex)->N_[Nindex].fatherindex_) != multi_goal_multi_nindex_for_solutions[goal_index].end())
		return;

	// Check whether the goal is in this node 
	int b;
//	std::vector<std::pair<double, double> > current_path;
	Path2D current_path;
	b = TheRobot(0)->N_.at(Nindex).safePathFinding(::goals[goal_index], current_path);
	//if (b == -1)
	//{
	//	// It is impossible to find a path for this goal
	//	std::cout << "ERROR: we do not give such goal: (" 
	//		<< ::goals[goal_index].first << ", " << ::goals[goal_index].second << ") !" << std::endl;
	//}
	//else 
	if (b == 1)
	{
		double cost = pathLength(current_path.data);

#if FINITE_LENGTH > 0
		if(cost > FINITE_LENGTH)
			return;
#endif

		multi_goal_multi_solutions[goal_index].push_back(current_path);
		multi_goal_multi_bestcost[goal_index].push_back(cost);
//		multi_solutions.push_back(current_path);
//		multi_bestcost.push_back(cost);

		multi_goal_multi_nindex_for_solutions[goal_index].push_back(Nindex);
//		multi_nindex_for_solutions.push_back(Nindex);

		// 2021.7.14 marked as solved
		// 2022.6.15 We should not do this, because there may be multiple goals
		//TheRobot(Rindex)->N_.at(Nindex).removeUsage();

		if (!EXHAUSTIVE)
		{
			TheRobot(Rindex)->sweeper_list_.erase(std::remove_if(TheRobot(Rindex)->sweeper_list_.begin(), TheRobot(Rindex)->sweeper_list_.end(),
				[&](const auto& a) {return a.Nindex == Nindex; }), TheRobot(Rindex)->sweeper_list_.end());

			TheRobot(Rindex)->path_list_.erase(std::remove_if(TheRobot(Rindex)->path_list_.begin(), TheRobot(Rindex)->path_list_.end(),
				[&](const auto& a) {return a.Nindex == Nindex; }), TheRobot(Rindex)->path_list_.end());
		}

	}
	else
	{
		// The goal is not in this node, we do nothing
	}

}


bool replanning(int Rindex)
{

	
	if (TheRobot(Rindex)->openlist_.empty())
	{
		// we cannot explore any more
		std::cout << "The openlist is empty." << std::endl;
		return true;
	}



	auto& openlist_temp = TheRobot(Rindex)->openlist_;

	

	if (!EXHAUSTIVE)
	{
		// We check for each goal location
		bool all_goal_finished = true;
		for (unsigned int i = 0; i < ::goals.size(); ++i)
		{
			std::vector<double> C(multi_goal_multi_bestcost[i]);

			std::sort(C.begin(), C.end());

			double current_min_cost_to_go = TheRobot(Rindex)->openlist_[0].Fcost[i];
			for (auto iter = TheRobot(Rindex)->openlist_.begin(); iter != TheRobot(Rindex)->openlist_.end(); ++iter)
			{
				if (iter->Fcost[i] < current_min_cost_to_go)
				{
					current_min_cost_to_go = iter->Fcost[i];
				}
			}

			if (C.size() >= num_of_nonhomo_path && C[num_of_nonhomo_path - 1] < current_min_cost_to_go)
			{
				std::cout << "We have found enough number of paths for goal " << i << "." << std::endl;
				logfile << "We have found enough number of paths for goal " << i << "." << std::endl;
			}
			else
			{
				all_goal_finished = false;
				break;
			}
		}
		if (all_goal_finished)
		{
			return true;
		}

		//std::vector<double> C(multi_bestcost);

		//std::sort(C.begin(), C.end());

		//double current_min_cost_to_go = TheRobot(Rindex)->openlist_[0].Fcost;
		//for (auto iter = TheRobot(Rindex)->openlist_.begin(); iter != TheRobot(Rindex)->openlist_.end(); ++iter)
		//{
		//	if (iter->Fcost < current_min_cost_to_go)
		//	{
		//		current_min_cost_to_go = iter->Fcost;
		//	}
		//}

		//if (C.size() >= num_of_nonhomo_path && C[num_of_nonhomo_path - 1] < current_min_cost_to_go)
		//{
		//	std::cout << "We have found enough number of paths." << std::endl;
		//	logfile << "We have found enough number of paths." << std::endl;
		//	return true;
		//}
	}


	// The corridor has already contained a collision-free path
	// Setup best parameters
	auto m = std::min_element(TheRobot(Rindex)->openlist_.begin(),
		TheRobot(Rindex)->openlist_.end(),
		[](const Candidate& a, const Candidate& b) {return a.leastFcost < b.leastFcost; });

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

	auto* temp = TheRobot(0);

	TheRobot(Rindex)->N_.at(Nindex).Expand();

	auto* temp2 = TheRobot(0);

	if (!TheRobot(Rindex)->N_.at(Nindex).C_.empty())
	{
		for (Cindex = 0; Cindex < TheRobot(Rindex)->N_.at(Nindex).C_.size(); ++Cindex)
		{
			if (TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).status_.compare("open") == 0)
			{
				// no matter whether the corridor has optimality, we record its gap
				// so that it may be used to remove other gaps

				// push the corridors into the openlist
				auto& temp = TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).ckiFcost_;
				double leastcost_temp = *std::min_element(temp.begin(), temp.end());
				if (FINITE_LENGTH > 0 && leastcost_temp > FINITE_LENGTH)
				{
					TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).status_ = "unnecessary";
					continue;
				}

				Path2D pathtemp;
				TheRobot(0)->N_.at(Nindex).C_.at(Cindex).tracePath(0, pathtemp);

				if (NON_SELFCROSSING && isSelfIntersect(pathtemp))
				{
					TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).status_ = "unnecessary";
					continue;
				}

				TheRobot(Rindex)->openlist_.push_back(Candidate(Nindex, Cindex,
					TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).ckiFcost_));

			}
		}
	}

	//std::cout << "check in replanning: " << TheRobot(0)->N_.size() << std::endl;
	// Check whether some goals are in the newly expanded node
	for (unsigned int i = 0; i < ::goals.size(); ++i)
		tryToFindSolutions(Rindex, i);


	if (!EXHAUSTIVE)
	{
		TheRobot(Rindex)->inheritPartialOrder();
		printPartialOrder();

		TheRobot(Rindex)->removeUnnecessaryLeafNode();
	}

	return false;
}

int main()
{
	// This is for ksnpp
//	cv::Mat img = cv::imread("simu.png", 0);

	// This is for tmv
	cv::Mat img = cv::imread("tmv_map.png", 0);
//	cv::Mat img = cv::imread("tmv_map_4obstacle.png", 0);
	if (img.empty())
	{
		return -1;
	}
	int height = img.rows;
	int width = img.cols;
	int ch = img.channels();
//	std::cout << "check map size (should be the same as the #define-d dimension) height: " << height << ", width: " << width << ", ch: " << ch << std::endl;
	cellsize_x = MAX_X_SIZE;
	cellsize_y = MAX_Y_SIZE;

	// We need to store the costmap in Euclidean X-Y coordinate
	for (unsigned int row = 0; row < height; ++row)
	{
		uchar *current_row = img.ptr<uchar>(row);
		for (unsigned int col = 0; col < width; ++col)
		{
			costmap_.data[(height - 1 - row)*width + col] = *(current_row + col);
		}
	}

	//showMap(costmap_, "show the map as image map");

	// We transform the image map (free:255, obstacle:0) to the costmap (free: 0, obstacle: 254, inflation: 127)
	for (unsigned int x = 0; x < MAX_X_SIZE; ++x)
	{
		for (unsigned int y = 0; y < MAX_Y_SIZE; ++y)
		{
			int temp = costmap_.data[y*MAX_X_SIZE + x];
			if (costmap_.data[y*MAX_X_SIZE + x] == 0)
			{
				costmap_.data[y*MAX_X_SIZE + x] = 254;
			}
			else if(costmap_.data[y*MAX_X_SIZE + x] >= 253) // a white grid, which corresponds to the obstacle-free point
			{
				costmap_.data[y*MAX_X_SIZE + x] = 0;
			}
		}
	}

	//showMap(costmap_, "show after image map --> cost map");

	// We add the boundary of the map
	for (unsigned int row = 0; row < height; ++row)
	{
		for (unsigned int col = 0; col < width; ++col)
		{
			if (row == 0 || row == height - 1)
			{
				costmap_.data[row*width + col] = 254;
			}
			else
			{
				if (col == 0 || col == width - 1)
				{
					costmap_.data[row*width + col] = 254;
				}
			}
		}
	}


	num_of_nonhomo_path = 4;




	logfile.open("temp_cpp.txt", ios::out | ios::binary);
	if (!logfile.is_open())
	{
		std::cout << "cannot open the log file" << std::endl;
	}

	computeFootprint(LETHAL_RADIUS);

	// We need to inflate the map 
	preCalculateCspace();

	// To make C++ be aligned with MATLAB, we still use int points for the source point. 
	// But when we create rays, we will use (x+0.5, y+0.5)
	std::pair<double, double> robot_position; 

	// For ksnpp
//	robot_position.first = 299.5;
//	robot_position.second = 349.5;

	// For tmv 9 obstacles
	robot_position.first = 79.5;
	robot_position.second = 29.5;

	// For tmv 4 obstacles
	//robot_position.first = 59.5;
	//robot_position.second = 17.5;


	// We store the base point in a global variable
	start_location = robot_position;

	::goals.clear();

	// For ksnpp
//	::goals.push_back(std::pair<double, double>(136.5, 319.5)); // This is the grid indices in the Euclidean X-Y coordinate

	// For tmv 9 obstacles
	::goals.push_back(std::pair<double, double>(164.5, 186.5));
	::goals.push_back(std::pair<double, double>(30.5, 218.5));
	::goals.push_back(std::pair<double, double>(135.5, 138.5));
	::goals.push_back(std::pair<double, double>(39.5, 131.5));
	::goals.push_back(std::pair<double, double>(229.5, 116.5));
	::goals.push_back(std::pair<double, double>(157.5, 9.5));

	// For tmv 4 obstacles
	//::goals.push_back(std::pair<double, double>(27.5, 142.5));
	//::goals.push_back(std::pair<double, double>(136.5, 25.5));



	std::vector<Path2D> path2d_temp;
	std::vector<double> bestcost_temp;
	multi_goal_multi_solutions.resize(::goals.size(), path2d_temp);
	multi_goal_multi_bestcost.resize(::goals.size(), bestcost_temp);

	std::vector<int> nindices_temp;
	multi_goal_multi_nindex_for_solutions.resize(::goals.size(), nindices_temp);


	clock_t ours_startTime = clock();

	int Rindex = 0;
	int Nindex = 0;

	//showMap(costmap_, "inflation");

	// Clear the robot at the beginning
	// If we use re-wire strategy, then don't clear them
	TheRobot(Rindex)->clear();

	TheRobot(Rindex)->N_.push_back(Node(Rindex, Nindex, robot_position));

	TheRobot(Rindex)->N_.at(Nindex).Expand();

	if (!TheRobot(Rindex)->N_.at(Nindex).C_.empty())
	{
		for (int Cindex = 0; Cindex < TheRobot(Rindex)->N_.at(Nindex).C_.size(); ++Cindex)
		{
			if (TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).status_.compare("open") == 0)
			{
				// No matter whether the corridor has optimality, we record its gap, 
				// So that it may be used to remove other gaps

				// Push the corridors into the openlist
				auto& temp = TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).ckiFcost_;
				double leastcost_temp = *std::min_element(temp.begin(), temp.end());
				if (FINITE_LENGTH > 0 && leastcost_temp > FINITE_LENGTH)
				{
					TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).status_ = "unnecessary";
					continue;
				}

				Path2D pathtemp;
				TheRobot(0)->N_.at(Nindex).C_.at(Cindex).tracePath(0, pathtemp);

				if (NON_SELFCROSSING && isSelfIntersect(pathtemp))
				{
					TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).status_ = "unnecessary";
					continue;
				}

				TheRobot(Rindex)->openlist_.push_back(Candidate(Nindex, Cindex,
					TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).ckiFcost_));

			}
		}
	}

	for(unsigned int i = 0; i < ::goals.size(); ++i)
		tryToFindSolutions(Rindex, i);

	bool finish = false;
	int index = 1;


	while(!finish)
	//for (index = 1; index < 120;)
	{
		//std::cout << "index = " << index << std::endl;

		// We print the seed point of the latest node

		//logfile << "latest seed node: [" << TheRobot(0)->N_.back().seed_.first
		//	<< ", " << TheRobot(0)->N_.back().seed_.second << "]" << std::endl;

		//printOpenlist();
		//printSolutionCost();

		auto* Rtemp = TheRobot(0);
		
		finish = replanning(Rindex);
	//	// std::cout << "time to construct node " << index << ": " << (t1-t0)*1000 << "ms." << std::endl;
		index = index + 1;
	}



	clock_t ours_endTime = clock();

	std::cout << "Time for ours: " << (double)(ours_endTime - ours_startTime)*1000.0 / CLOCKS_PER_SEC << "ms" << std::endl;

	
	for (auto iter = multi_goal_multi_solutions.begin(); iter != multi_goal_multi_solutions.end(); ++iter)
	{
		std::cout << "The number of solutions to visit goal location " << iter - multi_goal_multi_solutions.begin() << "= " << iter->size() << ", path Length = " << std::endl;
		for (auto iter2 = iter->begin(); iter2 != iter->end(); ++iter2)
		{
			std::cout << pathLength(iter2->data) << ", ";
		}
		std::cout << std::endl;
	}

	optimal_tmv();
	//Path2D optimal_tr_path;
	//optimal_tp(multi_goal_multi_solutions[0][0], ::goals[1], optimal_tr_path);

	if (0)
	{
		kim_main(robot_position);
		cv::waitKey(1);
		return 0;
	}

	logfile.close();

	cv::waitKey(1);
	return 0;
}