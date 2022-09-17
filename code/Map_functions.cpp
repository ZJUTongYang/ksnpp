#include "Map_functions.h"
#include <iostream>

#include <corecrt_math_defines.h>

int CC(int grid_x, int grid_y)
{
    // -1: known obstacles
    // 0: hitting unknown space, we return false but do not record the value to the CC_map
    // 1: known collision-free 
    if(!INMAP(grid_x, grid_y))
    {
        return -1;
    }
    if(costmap_.data[grid_y * cellsize_x + grid_x] <= CFREE || 
        costmap_.data[grid_y * cellsize_x + grid_x] > OBSTACLE)
    {
        // The unknown grids are automatically set as free by the costmap module, 
        // so we needn't clarify it here
        return 1;
    }
    return -1;

}

void efficientCCRayHittingPhysicalObstacle(std::pair<double, double> seed, double theta, double start_dis, 
    double& dis, double& Cdis, bool& infinite, double& hit_x, double& hit_y, 
    const bool forGapSweeper)
{
    std::pair<int, int> hitting_point_for_gap_sweeper;
    efficientCCRayHittingPhysicalObstacle(seed, theta, start_dis, dis, Cdis, 
        infinite, hit_x, hit_y, hitting_point_for_gap_sweeper, forGapSweeper);
}


void efficientCCRayHittingPhysicalObstacle(std::pair<double, double> seed, double theta, double start_dis, 
    double& dis, double& Cdis, bool& infinite, double& hit_x, double& hit_y, 
    std::pair<int, int>& forGapSweeperHittingObstacle, const bool forGapSweeper)
{
    // Here we do not pre-calculate a long enough ray, because it is unnecessary. 
    // We implement the modified bresenham algorithm in this function. 

    infinite = false;
    double max_length = 1000.0;
    // Here the distance is measured in grid resolution
    Cdis = -1;

    double xi = seed.first + start_dis*cos(theta);
    double yi = seed.second + start_dis*sin(theta);
    double dx, dy, ddis;
    bool main_x;
    if(fabs(cos(theta)) >= fabs(sin(theta)))
    {
        main_x = true;
        dx = cos(theta)/fabs(cos(theta));
        dy = sin(theta)/fabs(cos(theta));
    }
    else
    {
        main_x = false;
        dx = cos(theta)/fabs(sin(theta));
        dy = sin(theta)/fabs(sin(theta));
    }
    ddis = sqrt(dx*dx+dy*dy);

    dis = start_dis;
    double next_xi = xi + dx, next_yi = yi + dy;
    int floorxi = (int)floor(xi), flooryi = (int)floor(yi), floornext_xi = (int)floor(next_xi), floornext_yi = (int)floor(next_yi);
    if(main_x)
    {
        while(dis < max_length)
        {
            if(floornext_yi != flooryi)
            {
                double v;
                //We need to check whether the ray passes [floornext_xi, flooryi] or [floorxi, floornext_yi]
                if(dx > 0)
                {
                    // We check the intersection between ray and "x = floornext_xi" axis
                    v = (next_yi * (floornext_xi - xi) + yi * (next_xi - floornext_xi));
                    hit_x = floornext_xi;
                    hit_y = v;
                }
                else
                {
                    // We check the intersection between ray and "x = floor_xi" axis
                    v = -next_yi * (floorxi - xi) - yi * (next_xi - floorxi);
                    hit_x = floorxi;
                    hit_y = v;
                }

                if(floor(v) == flooryi)
                {
                    // The intermediate grid is [floornext_xi, flooryi]
                    if(costmap_.data[flooryi*cellsize_x + floornext_xi] == OBSTACLE)
                    {
						std::pair<double, double> temp = findHitting(floornext_xi, flooryi, seed, theta);
						hit_x = temp.first;
						hit_y = temp.second;

                        dis = hypot(hit_x - seed.first, hit_y - seed.second);
                        if(Cdis < 0)
                            Cdis = dis;
                        return ;
                    }
                    else if(Cdis < 0 && costmap_.data[flooryi*cellsize_x + floornext_xi] > CFREE && 
                        costmap_.data[flooryi*cellsize_x + floornext_xi] < OBSTACLE)
                    {
						std::pair<double, double> temp = findHitting(floornext_xi, flooryi, seed, theta);
						hit_x = temp.first;
						hit_y = temp.second;

                        Cdis = hypot(hit_x - seed.first, hit_y - seed.second);
                        if(forGapSweeper)
                        {
                            if(Cdis <= 3)
                            {
                                Cdis = -1;
                            }
                            else
                            {
                                return ;
                            }
                        }
                    }
                }
                else if(floor(v) == floornext_yi)
                {
                    // The intermediate grid is [floorxi, floornext_yi]
                    if(costmap_.data[floornext_yi*cellsize_x + floorxi] == OBSTACLE)
                    {

                        std::pair<double, double> temp = findHitting(floorxi, floornext_yi, seed, theta);
                        hit_x = temp.first;
                        hit_y = temp.second;
                        dis = hypot(hit_x - seed.first, hit_y - seed.second);
                        if(Cdis < 0)
                            Cdis = dis;
                        return ;
                    }
                    else if(Cdis < 0 && costmap_.data[floornext_yi*cellsize_x + floorxi] > CFREE && 
                        costmap_.data[floornext_yi*cellsize_x + floorxi] < OBSTACLE)
                    {
                        std::pair<double, double> temp = findHitting(floorxi, floornext_yi, seed, theta);
                        hit_x = temp.first;
                        hit_y = temp.second;
						Cdis = sqrt((hit_x - seed.first)*(hit_x - seed.first) + (hit_y - seed.second)*(hit_y - seed.second));
                        if(forGapSweeper)
                        {
                            if(Cdis <= 3)
                            {
                                Cdis = -1;
                            }
                            else
                            {
                                return ;
                            }
                        }
                    }
                }
                else
                {
                    std::cout << "error, should be either floor(v) == flooryi or floor(v) == floornext_yi" << std::endl;
                    std::cout << "v = " << v << ", floor(v) = " << floor(v) << ", flooryi = " << flooryi 
                              << ", floornext_yi = " << floornext_yi << std::endl;
                }
            }

            // We only need checking one point
            if(costmap_.data[floornext_yi*cellsize_x + floornext_xi] == OBSTACLE)
            {

                std::pair<double, double> xy = findHitting(floornext_xi, floornext_yi, seed, theta);
                hit_x = xy.first;
                hit_y = xy.second;
                dis = hypot(xy.first - seed.first, xy.second - seed.second);
                if(Cdis < 0)
                    Cdis = dis;
                return ;
            }
            else if(Cdis < 0 && costmap_.data[floornext_yi*cellsize_x + floornext_xi] > CFREE && 
                costmap_.data[floornext_yi*cellsize_x + floornext_xi] < OBSTACLE)
            {

                std::pair<double, double> xy = findHitting(floornext_xi, floornext_yi, seed, theta);
                hit_x = xy.first;
                hit_y = xy.second;
                Cdis = hypot(xy.first - seed.first, xy.second - seed.second);
                if(forGapSweeper)
                {
                    if(Cdis <= 3)
                    {
                        Cdis = -1;
                    }
                    else
                    {
                        return ;
                    }
                }
            }

            floorxi = floornext_xi;
            flooryi = floornext_yi;
            xi = next_xi;
            yi = next_yi;
            next_xi = xi + dx;
            next_yi = yi + dy;
            floornext_xi = floor(next_xi);
            floornext_yi = floor(next_yi);        
            dis += ddis;

        }
    }
    else//dy = 1
    {
        while(dis < max_length)
        {
            if(floorxi != floornext_xi)
            {
                double v;
                //We need to check whether the ray passes [floornext_xi, flooryi] or [floorxi, floornext_yi]
                if(dy > 0)
                {
                    // We check the intersection between ray and "y = floornext_yi" axis
                    v = (next_xi * (floornext_yi - yi) + xi * (next_yi - floornext_yi));
                    hit_x = v;
                    hit_y = floornext_yi;
                }
                else
                {
                    // We check the intersection between ray and "y = floor_yi" axis
                    v = -(next_xi * (flooryi - yi) + xi * (next_yi - flooryi));
                    hit_x = v;
                    hit_y = flooryi;
                }

                if(floor(v) == floorxi)
                {
                    // The intermediate grid is [floornext_xi, flooryi]
                    if(costmap_.data[floornext_yi*cellsize_x + floorxi] == OBSTACLE)
                    {
						std::pair<double, double> temp = findHitting(floorxi, floornext_yi, seed, theta);
						hit_x = temp.first;
						hit_y = temp.second; 
						dis = hypot(hit_x - seed.first, hit_y - seed.second);
                        if(Cdis < 0)
                            Cdis = dis;
                        return ;
                    }
                    else if(Cdis < 0 && costmap_.data[floornext_yi*cellsize_x + floorxi] > CFREE && 
                         costmap_.data[floornext_yi*cellsize_x + floorxi] < OBSTACLE)
                    {
						std::pair<double, double> temp = findHitting(floorxi, floornext_yi, seed, theta);
						hit_x = temp.first;
						hit_y = temp.second;
						Cdis = hypot(hit_x - seed.first, hit_y - seed.second);
                        if(forGapSweeper)
                        {
                            if(Cdis <= 3)
                            {
                                Cdis = -1;
                            }
                            else
                            {
                                return ;
                            }
                        }
                    }
                }
                else if(floor(v) == floornext_xi)
                {
                    // The intermediate grid is [floornext_xi, flooryi]
                    if(costmap_.data[flooryi*cellsize_x + floornext_xi] == OBSTACLE)
                    {
                        std::pair<double, double> temp = findHitting(floornext_xi, flooryi, seed, theta);
                        hit_x = temp.first;
                        hit_y = temp.second;
                        dis = hypot(hit_x - seed.first, hit_y - seed.second);
                        Cdis = (Cdis < 0)? dis : Cdis;
                        return ;
                    }
                    else if(Cdis < 0 && costmap_.data[flooryi*cellsize_x + floornext_xi] > CFREE && 
                        costmap_.data[flooryi*cellsize_x + floornext_xi] < OBSTACLE)
                    {
                        std::pair<double, double> temp = findHitting(floornext_xi, flooryi, seed, theta);
                        hit_x = temp.first;
                        hit_y = temp.second;
                        Cdis = hypot(hit_x - seed.first, hit_y - seed.second);
                        if(forGapSweeper)
                        {
                            if(Cdis <= 3)
                            {
                                Cdis = -1;
                            }
                            else
                            {
                                return ;
                            }
                        }
                    }
                }
                else
                {
                    std::cout << "error, should be either floor(v) == floorxi or floor(v) == floornext_xi" << std::endl;
                    std::cout << "v = " << v << ", floor(v) = " << floor(v) << ", floorxi = " << floorxi 
                              << ", floornext_xi = " << floornext_xi << std::endl;
                }
            }

            
            // We only need checking one point
            if(costmap_.data[floornext_yi*cellsize_x + floornext_xi] == OBSTACLE)
            {
                std::pair<double, double> xy = findHitting(floornext_xi, floornext_yi, seed, theta);
                hit_x = xy.first;
                hit_y = xy.second;
                dis = hypot(xy.first - seed.first, xy.second - seed.second);
                Cdis = (Cdis < 0)? dis : Cdis;
                return ;
            }
            else if(Cdis < 0 && costmap_.data[floornext_yi*cellsize_x + floornext_xi] > CFREE && 
                costmap_.data[floornext_yi*cellsize_x + floornext_xi] < OBSTACLE)
            {
                std::pair<double, double> xy = findHitting(floornext_xi, floornext_yi, seed, theta);
                hit_x = xy.first;
                hit_y = xy.second;
                Cdis = hypot(xy.first - seed.first, xy.second - seed.second);
                if(forGapSweeper)
                {
                    if(Cdis <= 3)
                    {
                        Cdis = -1;
                    }
                    else
                    {
                        return ;
                    }
                }
            }

            floorxi = floornext_xi;
            flooryi = floornext_yi;
            xi = next_xi;
            yi = next_yi;
            next_xi = xi + dx;
            next_yi = yi + dy;
            floornext_xi = floor(next_xi);
            floornext_yi = floor(next_yi);        
            dis += ddis;

        }
    }

    std::cout << "efficientCCRayHittingPhysicalObstacle: we should not reach here. " << std::endl;
    
}


void footprintProjection(double theta, double &left, double &right, 
    std::pair<double, double> &leftpoint, std::pair<double, double> &rightpoint)
{
    double cosr = cos(theta - M_PI/2);
    double sinr = sin(theta - M_PI/2);
    double cosl = cos(theta + M_PI/2);
    double sinl = sin(theta + M_PI/2);

    // std::vector<double> L, R;
    // for(unsigned int i =0; i < footprint.size(); i++)
    // {
    //     L.push_back(footprint.at(i).first*cosl + footprint.at(i).second*sinl);
    //     R.push_back(footprint.at(i).first*cosr + footprint.at(i).second*sinr);
    // }

    // auto maxL = std::max_element(L.begin(), L.end());
    // auto maxR = std::max_element(R.begin(), R.end());

    // left = *maxL;
    // right = *maxR;
    // leftpoint = footprint.at(maxL - L.begin());
    // rightpoint = footprint.at(maxR - R.begin());

    left = LETHAL_RADIUS;
    right = LETHAL_RADIUS;
    leftpoint.first = LETHAL_RADIUS *cosl;
    leftpoint.second = LETHAL_RADIUS *sinl;
    rightpoint.first = LETHAL_RADIUS *cosr;
    rightpoint.second = LETHAL_RADIUS *sinr;
}

void getCCHittingObstacle(double px, double py, std::vector<std::pair<int, int> >& obs_list)
{
    int x, y;
    for(unsigned int i = 0; i < footprint.size(); i++)
    {
        x = floor(footprint.at(i).first + px);
        y = floor(footprint.at(i).second + py);
        
        if(!INMAP(x, y))
        {
            continue;
        }
        if(costmap_.data[y*cellsize_x + x] == OBSTACLE)
        {
            obs_list.push_back(std::pair<int, int>(x, y));
        }
    }
}


void getDetailedCCHittingObstacle(double px, double py, std::vector<std::pair<int, int> >& obs_list)
{
    int x, y;
    for(unsigned int i = 0; i < detailed_footprint.size(); i++)
    {
        x = floor(detailed_footprint.at(i).first + px);
        y = floor(detailed_footprint.at(i).second + py);
        if(!INMAP(x, y))
        {
            continue;
        }
        // std::cout << "(x, y, cost) = (" << x << ", " << y << ", " << (int)(costmap_.data[y*cellsize_x + x]) << ")" << std::endl;
        
        if(costmap_.data[y*cellsize_x + x] == OBSTACLE)
        {
            obs_list.push_back(std::pair<int, int>(x, y));
        }
    }
}
