#include "Kim_alg.h"
#include <ctime>

Kim2013::Kim2013* kim2013_planner;
//
//double pathLength(const std::vector<std::pair<int, int> >& path, int loc)
//{
//    if(path.empty())
//		return std::numeric_limits<double>::infinity();
//
//    if(loc < 0)
//    {
//        loc = path.size() - 1;
//    }
//
//	double L = 0;
//	for(unsigned int i = 0; i < loc; ++i)
//	{
//		L += hypot(path.at(i+1).first - path.at(i).first, path.at(i+1).second - path.at(i).second);
//	}
//	return L;
//}

void kim_main(std::pair<double, double> current_position) 
{
	if (FINITE_LENGTH == 0)
	{
		std::cout << "Error, we should have a maximum tether length constraint to run Kim's algorithm" << std::endl;
		return;
	}

    kim2013_planner = new Kim2013::Kim2013();

    std::cout << "now we wait for map and set goal: " << std::endl;

	kim2013_planner->setStart(current_position);


	///////////////////////////////////////////////////////////////////////////////////////////
            std::cout << "we start precalculation" << std::endl;
            //ros::WallTime t0 = ros::WallTime::now();           
			clock_t startTime = clock();
            kim2013_planner->preCalculateAllConfigurations();
			clock_t endTime = clock();

            std::cout << "Time for Kim2013 homotopic planning precalculation: " << (double)(endTime - startTime)*1000.0/CLOCKS_PER_SEC << "ms" << std::endl;
            

    std::cout << "size of global buffer: " << kim2013_planner->Storage_.size() << std::endl;

	Path2D optimal_tp_path;
	clock_t optimal_tp_startTime = clock();
	kim2013_planner->new_optimal_tp(multi_goal_multi_solutions[0][0], ::goals[1], optimal_tp_path);
	clock_t optimal_tp_endTime = clock();
	std::cout << "Time for optimal TP: " << (double)(optimal_tp_endTime - optimal_tp_startTime)*1000.0 / CLOCKS_PER_SEC << "ms" << std::endl;

	return;
}
