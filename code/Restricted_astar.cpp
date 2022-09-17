#include "Restricted_astar.h"
#include "Definitions.h"
#include "Map_functions.h"
#include "Atom.h"
#include <queue>

int toIndex(int x, int y, int nx)
{
	return y*nx + x;
}

RestrictedAstar::RestrictedAstar(std::pair<int, int> offset, 
		unsigned int xsize, unsigned int ysize, 
		std::pair<int, int> start_in_global, 
		std::pair<int, int> goal_in_global, 
		std::pair<double, double> origin, std::pair<double, double> dx, 
		std::pair<double, double> dy, 
		std::pair<double, double> n_dx, 
		double xposmax, double xnegmax, double ymax)
{

	offset_ = offset;


	nx_ = xsize;
	ny_ = ysize;
	ns_ = nx_*ny_;

	cell_start_.first = start_in_global.first - offset.first;
	cell_start_.second = start_in_global.second - offset.second;

	cell_goal_.first = goal_in_global.first - offset.first;
	cell_goal_.second = goal_in_global.second - offset.second;

	origin_ = origin;
	dx_ = dx;
	dy_ = dy;
	n_dx_ = n_dx;
	xposmax_ = xposmax;
	xnegmax_ = xnegmax;
	ymax_ = ymax;

	int tempx = abs(cell_goal_.first - cell_start_.first);
	int tempy = abs(cell_goal_.second - cell_start_.second);


	theoretical_lowerbound_ = tempx + tempy - 0.586*std::min(tempx, tempy);

	path_in_global_frame_.clear();
	path_in_global_frame_.reserve(tempx + tempy);
	newly_found_obstacle_in_global_frame_.clear();
	newly_found_obstacle_in_global_frame_.reserve(ceil(xposmax+xnegmax)*ceil(ymax));

	solveAstar();
}

RestrictedAstar::~RestrictedAstar()
{
}

void RestrictedAstar::solveAstar()
{

	if(calculatePotentials())
	{
		// std::cout << "we can calculate potential. now get path" << std::endl;
		getPath();
	}
}

bool RestrictedAstar::calculatePotentials()
{
	auto comp = [](const std::pair<int, double>& a, const std::pair<int, double>& b){return a.second > b.second;};

	potential_.resize(ns_, 10000.0);
	parent_.resize(ns_, -1);
	close_.resize(ns_, false);

	queue_.clear();
	int start_i = toIndex(cell_start_.first, cell_start_.second, nx_);
	queue_.push_back(std::pair<int, double>(start_i, 0.0));
	potential_[start_i] = 0;

	int goal_i = toIndex(cell_goal_.first, cell_goal_.second, nx_);
	while(!queue_.empty())
	{
		auto top = queue_[0];
		std::pop_heap(queue_.begin(), queue_.end(), comp);
		queue_.pop_back();
		
		close_[top.first] = true;
		int x = top.first % nx_;
		int y = (top.first - x) / nx_;

		if(top.first == goal_i)
			return true;

		add(top.first, top.first+1, 1);
		add(top.first, top.first-1, 1);
		add(top.first, top.first+nx_, 1);
		add(top.first, top.first-nx_, 1);
		add(top.first, top.first+1+nx_, 1.414);
		add(top.first, top.first-1-nx_, 1.414);
		add(top.first, top.first+1-nx_, 1.414);
		add(top.first, top.first-1+nx_, 1.414);
	}
	return false;
}

void RestrictedAstar::add(int i, int next_i, double motioncost)
{
	if(next_i < 0 || next_i >= ns_)
		return ;

	if(close_[next_i] == true)
		return ;

	if(potential_[next_i] < 0)
		// This means that we know it hits obstacles, 
		// and we have added it to the obstacle list
		return ;

	int curr_x = i % nx_;
	int curr_y = (i-curr_x)/nx_;

	double prev_potential = potential_[i];
	int x = next_i % nx_;
	int y = (next_i - x) / nx_;

	if(abs(x-curr_x) > 1 || abs(y - curr_y) > 1)
		// When the boundary of the region is not bounded, the index may go cyclicly.
		return ;

	int global_x = x + offset_.first;
	int global_y = y + offset_.second;

	if(CC(global_x, global_y) == -1)
	{
		std::vector<std::pair<int, int> > obs_list;
		getCCHittingObstacle(global_x, global_y, obs_list);
		newly_found_obstacle_in_global_frame_.insert(newly_found_obstacle_in_global_frame_.end(),
			obs_list.begin(), obs_list.end());

		//newly_found_obstacle_in_global_frame_.push_back(std::pair<int, int>(global_x, global_y));
		potential_[next_i] = -1;
		return ;
	}

	if(!isInNarrowCorridor(std::pair<double, double>(global_x, global_y), origin_, dx_, 
		dy_, n_dx_, xposmax_, xnegmax_, ymax_))
	{
		close_[next_i] = true;
		return ;
	}

	// std::cout << "prev_potential: " << prev_potential << ", potential_[next_i] = " << potential_[next_i] << std::endl;
	double newpotential = prev_potential + motioncost;
	// double newpotential = hypot(x - cell_goal_.first, y - cell_goal_.second);
	if(newpotential > potential_[next_i])
		return ;
	
	double distance = hypot(x - cell_goal_.first, y - cell_goal_.second);
	if(potential_[next_i] > 9999)
	{
		// This is a white grid
		potential_[next_i] = newpotential;
		parent_[next_i] = i;
		// We needn't re-build the heap, just insert the new one
		// std::cout << "we insert new points to queue" << std::endl;
		queue_.push_back(std::pair<int, double>(next_i, newpotential + distance));
		std::push_heap(queue_.begin(), queue_.end(), 
			[](const std::pair<int, double>& a, const std::pair<int, double>& b){return a.second > b.second;}
		);
	}
	else
	{
		auto loc = std::find_if(queue_.begin(), queue_.end(), 
			[&](const auto a){return a.first == next_i;}
		);
		// It is a gray grid
		potential_[next_i] = newpotential;
		parent_[next_i] = i;
		queue_.erase(loc);
		queue_.push_back(std::pair<int, double>(next_i, newpotential + distance));
		std::make_heap(queue_.begin(), queue_.end(), 
			[](const std::pair<int, double>& a, const std::pair<int, double>& b){return a.second > b.second;}
			);
		// Since element is removed, we have to re-build the heap
	}
}

void RestrictedAstar::getPath()
{
	std::pair<int, int> current = cell_goal_;
	int start_index = toIndex(cell_start_.first, cell_start_.second, nx_);
	int current_index = toIndex(cell_goal_.first, cell_goal_.second, nx_);

	// double min_val;
	// int min_x, min_y;
	// int x, y, index;
	// path_in_global_frame_.push_back(cell_goal_ + offset_);
	while(1)
	{
		int x = current_index % nx_;
		int y = (current_index - x) / nx_;
		int global_x = x + offset_.first;
		int global_y = y + offset_.second;
		path_in_global_frame_.push_back(std::pair<int, int>(global_x, global_y));
		int p = parent_[current_index];
		if(p == -1)
			break;
		else
		{
			current_index = p;
		}
	}

	std::reverse(path_in_global_frame_.begin(), path_in_global_frame_.end());

	// std::cout << "potential of goal: " << potential_[goal_i] << ", bound: " << theoretical_lowerbound_ +2 << std::endl;
	if(potential_[cell_goal_.second*nx_ + cell_goal_.first] <= theoretical_lowerbound_ + 2)
		newly_found_obstacle_in_global_frame_.clear();
}