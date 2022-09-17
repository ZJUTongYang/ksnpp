#ifndef _RESTRICTEDASTAR_
#define _RESTRICTEDASTAR_

#include <vector>

class RestrictedAstar{
public:

	// the start and the goal are in the related frame
	std::pair<int, int> cell_start_;
	std::pair<int, int> cell_goal_;

	std::pair<int, int> start_in_global_;
	std::pair<int, int> goal_in_global_;

	std::pair<int, int> offset_;

	unsigned int nx_;
	unsigned int ny_;
	unsigned int ns_;

	std::vector<std::pair<int, int> > path_in_global_frame_;
	std::vector<std::pair<int, int> > newly_found_obstacle_in_global_frame_;


	RestrictedAstar(std::pair<int, int> offset, 
		unsigned int xsize, unsigned int ysize, 
		std::pair<int, int> start_in_global, 
		std::pair<int, int> goal_in_global, 
		std::pair<double, double> origin, 
		std::pair<double, double> dx, 
		std::pair<double, double> dy, 
		std::pair<double, double> n_dx, 
		double xposmax, double xnegmax, double ymax);

	~RestrictedAstar();

	void solveAstar();

private:
	std::pair<double, double> origin_;
	std::pair<double, double> dx_;
	std::pair<double, double> dy_;
	std::pair<double, double> n_dx_;
	double xposmax_;
	double xnegmax_;
	double ymax_;

	double theoretical_lowerbound_;

	std::vector<std::pair<int, double> > queue_;

	// double* potential_;
	std::vector<double> potential_;
	std::vector<int> parent_;
	std::vector<bool> close_;

	bool calculatePotentials();
	void getPath();
	void add(int curr_i, int next_i, double motioncost);

};


#endif