#include "Definitions.h"
#include <fstream>

Costmap costmap_;
int cellsize_x;
int cellsize_y;

std::pair<double, double> start_location;

std::vector<std::pair<int, int> > footprint;
std::vector<std::pair<int, int> > detailed_footprint;
std::vector<std::pair<double, double> > goals;

//std::vector<double> bestcost;
//std::vector<Path2D> solutions;

std::vector<std::vector<Path2D> > multi_goal_multi_solutions;
std::vector<std::vector<int> > multi_goal_multi_nindex_for_solutions;
//extern std::vector<Path2D> multi_solutions;
//extern std::vector<int> multi_nindex_for_solutions;

std::vector<std::vector<double> > multi_goal_multi_bestcost;
//std::vector<double> multi_bestcost;

int num_of_nonhomo_path;

std::ofstream logfile;