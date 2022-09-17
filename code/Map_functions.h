#ifndef _MAP_FUNCTIONS_
#define _MAP_FUNCTIONS_

#include <string>
#include "Definitions.h"
#include "Atom.h"



extern int CC(int grid_x, int grid_y);
// extern void CCRayHittingPhysicalObstacle(std::pair<double, double> seed, double theta, double start_dis, 
//     double& d, double& Cdis, bool& infinite);

extern void footprintProjection(double theta, double &left, double &right, 
    std::pair<double, double> &leftpoint, std::pair<double, double> &rightpoint);
extern void getCCHittingObstacle(double px, double py, std::vector<std::pair<int, int> >& obs_list);
extern void getDetailedCCHittingObstacle(double px, double py, std::vector<std::pair<int, int> >& obs_list);

extern void efficientCCRayHittingPhysicalObstacle(std::pair<double, double> seed, double theta, double start_dis, 
    double& d, double& Cdis, bool& infinite, double& hit_x, double& hit_y, 
    const bool forGapSweeper = false);

extern void efficientCCRayHittingPhysicalObstacle(std::pair<double, double> seed, double theta, double start_dis, 
    double& d, double& Cdis, bool& infinite, double& hit_x, double& hit_y, 
    std::pair<int, int>& forGapSweeperHittingObstacle, 
    const bool forGapSweeper = false);

#endif