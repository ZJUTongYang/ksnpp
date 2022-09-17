#include "Definitions.h"
#include "Atom.h"
#include "Robot.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <unordered_set>

#include <corecrt_math_defines.h>

bool findIntersection(const std::pair<double, double>& s1, const std::pair<double, double>& e1, 
    const std::pair<double, double>& s2, const std::pair<double, double>& e2, 
    std::pair<double, double>& q);

void bresenham2D(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
    int offset_b, int offset, std::vector<std::pair<int, int> >& result)
{
    // abs_da is always bigger than abs_db
    unsigned int end = abs_da;
    for (unsigned int i = 0; i < end; ++i)
    {
        int x = offset % cellsize_x;
        int y = (offset - x) / cellsize_x;

        result.push_back(std::pair<int, int>(x, y));
        offset += offset_a;
        error_b += abs_db;
        if ((unsigned int)error_b >= abs_da)
        {
            offset += offset_b;
            error_b -= abs_da;
        }
    }
    int x = offset % cellsize_x;
    int y = (offset - x) / cellsize_x;

    result.push_back(std::pair<int, int>(x, y));
}

double calStartDis(double thetasmall, double dis1, double Cdis1, 
	double thetabig, double dis2, double Cdis2, double radius)
{
    // return 0;

    double ang_diff = thetabig - thetasmall;
    double raydis = std::min(dis1, dis2) * cos(ang_diff/2) - 1;
    double Cdis = std::min(radius/tan(ang_diff/2), 
        std::min(Cdis1, Cdis2)*cos(ang_diff/2))-1;
    
    double gridDis = std::max(std::min(0.5/tan(ang_diff/2)-1, raydis), 0.0);
    // because we know that the source of ray is collision-free

    return std::max(std::min(Cdis, raydis), gridDis);
}

ObsExit chooseOneObsExit(std::vector<ObsExit>& obs_exit, const std::vector<int>& obs_exit_index, 
    int& nearest_obs_exit_index)
{
    // We choose the nearest corridor to insert
    ObsExit result;
    if(obs_exit.empty())
    {
        std::cout << "No obs_exit, so cannot find the nearest one" << std::endl;
        return result;
    }

    auto comp = [](const ObsExit& a, const ObsExit& b)
        {return std::min(a.disbig, a.dissmall) < std::min(b.disbig, b.dissmall);};

    int index = std::min_element(obs_exit.begin(), obs_exit.end(), comp) - obs_exit.begin();
    

    nearest_obs_exit_index = obs_exit_index[index];

    return *(obs_exit.begin()+index);
	
}

void classicBresenham(double doublex0, double doubley0, double doublex1, double doubley1, 
    std::vector<std::pair<int, int> >& result)
{
    int x0 = floor(doublex0), x1 = floor(doublex1), y0 = floor(doubley0), y1 = floor(doubley1);
    int dx = x1-x0;
    int dy = y1-y0;
    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);
    int offset_dx = sign(dx);
    int offset_dy = sign(dy)*cellsize_x;

    unsigned int offset = y0*cellsize_x + x0;

    result.reserve(std::max(abs(dx), abs(dy))+1);

    // double dist = hypot(dx, dy);
    double scale = 1;

    if(abs_dx >= abs_dy)
    {
        int error_y = abs_dx / 2;
        bresenham2D(abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, result);
        return ;
    }

    int error_x = abs_dy/2;
    bresenham2D(abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, result);

}


void computeFootprint(int lethal_radius)
{
    // This function should be the same implementation as 
    // the costmap inflation layer in ROS navigation

    // lethal_radius must be set before
    int* mask;
    int n = ceil(lethal_radius) + 1;
    unsigned int d = 2*n+1;
    mask = new int[d*d];
    memset(mask, 0, d*d*sizeof(int));
    int center_i = n*d + n;
    for(unsigned int x = 0; x < d; ++x)
    {
        for(unsigned int y = 0; y < d; ++y)
        {
            if(hypot(x-n, y-n) <= lethal_radius)
                mask[y*d+x] = 1;
        }
    }
    for(unsigned int i = 1; i < d-1; ++i)
    {
        for(unsigned int j = 1; j < d-1; ++j)
        {
            if(mask[(j-1)*d+i]*mask[(j+1)*d+i]*mask[j*d+i+1]*mask[j*d+i-1] == 0)
                footprint.push_back(std::pair<int, int>(i-n, j-n));
            
            detailed_footprint.push_back(std::pair<int, int>(i-n, j-n));
        }
    }
}

void constructFakeCorridorParams(const std::pair<double, double> start, const std::pair<double, double> goal, 
	std::pair<double, double>& origin, std::pair<double, double>& dx, std::pair<double, double>& dy, 
    std::pair<double, double>& n_dx, 
	double& xposmax, double&xnegmax, double& ymax)
{
    // This function only writes for circular robot

    double theta = atan2(goal.second - start.second, goal.first - start.first);
    double left, right;
    std::pair<double, double> leftpoint, rightpoint;
    footprintProjection(theta, left, right, leftpoint, rightpoint);

    // new frame(not orthonormal frame)
    dy.first = cos(theta);
    dy.second = sin(theta);
    dx.first = cos(theta - M_PI/2);
    dx.second = sin(theta - M_PI/2);

    n_dx = dx;
    xposmax = ceil(left) + 1;
    xnegmax = ceil(right) + 1;
    ymax = ceil(hypot( goal.first - start.first, goal.second - start.second))+1;
    origin = start;
}

//
//bool currentIsOptimal(int index, int goal_index)
//{
//    // True: This is the globally optimal path
//    if(TheRobot(0)->openlist_.empty())
//        return true;
//
//	double min_cost_in_openlist = TheRobot(0)->openlist_[0].Fcost[goal_index];
//	for (unsigned int i = 0; i < TheRobot(0)->openlist_.size(); ++i)
//	{
//		if (TheRobot(0)->openlist_[i].Fcost[goal_index] < min_cost_in_openlist)
//			min_cost_in_openlist = TheRobot(0)->openlist_[i].Fcost[goal_index];
//	}
//
//	if (bestcost[index] < min_cost_in_openlist)
//	{
//		return true;
//	}
//	else
//	{
//		return false;
//	}
//	
//}

std::pair<double, double> findHitting(int cell_x, int cell_y, std::pair<double,double> seed, double theta)
{
    // Note that there is precision problem in C++ (no such problem in MATLAB :( )

    if(floor(seed.first) == cell_x && floor(seed.second) == cell_y)
    {
        std::cout <<"YT: there is no need to calculate the hitting points" << std::endl;
        return std::pair<double, double>(cell_x, cell_y);
    }

    if(seed.first == cell_x+0.5)
    {
        double x = cell_x+0.5;
        double y;
        if(seed.second < cell_y)
        {
            y = cell_y;
        }
        else
        {
            y = cell_y + 1;
        }

        return std::pair<double, double>(x, y);
    }

	double k = tan(theta);
	double b = seed.second - seed.first*k;

	double eps = 0.0001;

    if(seed.first < cell_x+0.5)
    {
        if(seed.second < cell_y+0.5)
        {
            // whether hitting on bottom edge
            double y = cell_y;
            double x = (y-b)/k;

			if(x >= cell_x-eps && x <= cell_x + 1+eps)
            {
                return std::pair<double, double>(x, y);
            }
            // whether hitting on left edge
            x = cell_x;
            y = k*x+b;
			if(y >= cell_y-eps && y <= cell_y + 1+eps)
            {
                return std::pair<double, double>(x, y);
            }
        }
        else
        {
            // whether hitting on left edge
            double x = cell_x;
            double y = k*x+b;
			if(y >= cell_y-eps && y <= cell_y + 1+eps)
            {
                return std::pair<double, double>(x, y);
            }
            // whether hitting on top edge
            y = cell_y + 1;
            x = (y-b)/k;
			if(x >= cell_x-eps && x <= cell_x + 1+eps)
            {
                return std::pair<double, double>(x, y);
            }
        }
    }
    else//seed.first > cell_x + 0.5
    {
        if(seed.second < cell_y+0.5)
        {
            // whether hitting on bottom edge
            double y = cell_y;
            double x = (y-b)/k;
			if(x >= cell_x-eps && x <= cell_x + 1+eps)
            {
                return std::pair<double, double>(x, y);
            }
            // whether hitting on right edge
            x = cell_x + 1;
            y = k*x+b;
			if(y >= cell_y-eps && y <= cell_y + 1+eps)
            {
                return std::pair<double, double>(x, y);
            }
        }
        else
        {
            // whether hitting on top edge
            double y = cell_y + 1;
            double x = (y-b)/k;
			if(x >= cell_x-eps && x <= cell_x + 1+eps)
            {
                return std::pair<double, double>(x, y);
            }
            // whether hitting on right edge
            x = cell_x + 1;
            y = k*x+b;
			if(y >= cell_y-eps && y <= cell_y + 1+eps)
            {
                return std::pair<double, double>(x, y);
            }
        }  
    }

	// YT: we shouldn't reach here
	std::cout << "findHitting: error, we shouldn't reach here" << std::endl;
    std::cout << "cell_x, cell_y = " << cell_x << ", " << cell_y 
              << ", seed = (" << seed.first << ", " << seed.second << "), theta = " << theta << std::endl;
	return std::pair<double, double>(-1.0, -1.0);
}


bool findIntersection(const Sweeper& gap, const Path& path, 
    std::pair<double, double>& q, int& loc)
{
    // loc is the index of the hitting point on the path. It is used to calculate path length in subsequent cost comparison

    // We approximate the position 
    const std::vector<std::pair<double, double> >& P = TheRobot(path.Rindex)->N_.at(path.Nindex).C_.at(path.Cindex).myPath_.data;

    if(!findIntersection(P.front(), P.back(), gap.cki, gap.obs, q))
        return false;


    auto iter = P.begin();
    auto next = std::next(iter);
    for(auto iter = P.begin(); iter != --P.end(); ++iter)
    {
        next = std::next(iter);
        bool b = findIntersection(*iter, *next, gap.cki, gap.obs, q);
        if(b)
        {
            loc = iter - P.begin();
            return true;
        }
    }

    return false;

}


bool findIntersection(const Sweeper& gap1, const Sweeper& gap2, 
    std::pair<double, double>& q)
{
    return findIntersection(gap1.cki, gap1.obs, gap2.cki, gap2.obs, q);
}

bool findIntersection(const std::pair<double, double>& s1, 
    const std::pair<double, double>& e1, 
    const std::pair<double, double>& s2, 
    const std::pair<double, double>& e2, 
    std::pair<double, double>& q)
{
    if(std::max(s1.first, e1.first) < std::min(s2.first, e2.first))
        return false;
    if(std::min(s1.first, e1.first) > std::max(s2.first, e2.first))
        return false;
    if(std::max(s1.second, e1.second) < std::min(s2.second, e2.second))
        return false;
    if(std::min(s1.second, e1.second) > std::max(s2.second, e2.second))
        return false;

    //int p0_x = round(s1.first);
    //int p0_y = round(s1.second);
    //int p1_x = round(e1.first);
    //int p1_y = round(e1.second);
    //int p2_x = round(s2.first);
    //int p2_y = round(s2.second);
    //int p3_x = round(e2.first);
    //int p3_y = round(e2.second);


	//// 2022.5.4 YT: just to be the same as MATLAB
	//double p0_x = floor(s1.first)+0.5;
	//double p0_y = floor(s1.second) + 0.5;
	//double p1_x = floor(e1.first) + 0.5;
	//double p1_y = floor(e1.second) + 0.5;
	//double p2_x = floor(s2.first) + 0.5;
	//double p2_y = floor(s2.second) + 0.5;
	//double p3_x = floor(e2.first) + 0.5;
	//double p3_y = floor(e2.second) + 0.5;

	// 2022.5.5 YT: we use the analytic points
	double p0_x = s1.first;
	double p0_y = s1.second;
	double p1_x = e1.first;
	double p1_y = e1.second;
	double p2_x = s2.first;
	double p2_y = s2.second;
	double p3_x = e2.first;
	double p3_y = e2.second;

    if ((((p0_x - p2_x)*(p3_y - p2_y) - (p0_y - p2_y)*(p3_x - p2_x))*
            ((p1_x - p2_x)*(p3_y - p2_y) - (p1_y - p2_y)*(p3_x - p2_x))) > 0 ||
            (((p2_x - p0_x)*(p1_y - p0_y) - (p2_y - p0_y)*(p1_x - p0_x))*
            ((p3_x - p0_x)*(p1_y - p0_y) - (p3_y - p0_y)*(p1_x - p0_x))) > 0)
        return false;

    double s1_x, s1_y, s2_x, s2_y;  
    s1_x = p1_x - p0_x;
    s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;
    s2_y = p3_y - p2_y;

    double s, t;
    t = 1.0 * ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);
    q.first = p0_x + (t * s1_x);
    q.second = p0_y + (t * s1_y);
    return true;

}


bool isGoalFree(const std::vector<std::pair<double, double> >& bd)
{
    // The first element of bd is the same as the last element
    // true: there is no goal in the region
    // false: there is at least one goal in the region
    if(::goals.empty())
        return true;

    // Construct arrays for checking polygons
    int nvert = bd.size();
    float *vertx = new float[nvert];
    float *verty = new float[nvert];
    for(unsigned int i = 0; i < bd.size(); ++i)
    {
        vertx[i] = bd.at(i).first;
        verty[i] = bd.at(i).second;
    }

    float testx, testy;
    for(unsigned int i = 0; i < ::goals.size(); ++i)
    {
        testx = ::goals.at(i).first;
        testy = ::goals.at(i).second;
        if(pnpoly(nvert, vertx, verty, testx, testy))
        {
            delete[] vertx;
            delete[] verty;
            return false;
        }
    }
    delete[] vertx;
    delete[] verty;
    return true;
}

bool isInNarrowCorridor(std::pair<double, double> world_p, std::pair<double, double> origin, 
    std::pair<double, double> dx, std::pair<double, double> dy, 
    std::pair<double, double> n_dx, double xposmax, double xnegmax, double ymax)
{
    // We check whether a given point is in the narrow corridor. 
    // Transform to local frame to see the local position

    std::pair<double, double> d = world_p - origin;

    std::pair<double, double> p;
    p.first = d.first*n_dx.first + d.second*n_dx.second;

    // check local x coordinate
    if((p.first < 0 && p.first < -xnegmax) || (p.first > 0 && p.first > xposmax))

    // check local y coordinate
    p.second = d.first*dy.first + d.second*dy.second;
    if(p.second < -1 || p.second > ymax)
    {
        return false;
    }

    return true;    
}

bool isSelfIntersect(const Path2D& current_path)
{
	std::unordered_set<int> S;
	for (auto iter = current_path.data.begin(); iter != current_path.data.end(); ++iter)
	{
		S.insert(round(iter->first) + round(iter->second)*MAX_X_SIZE);
	}
	if (S.size() < current_path.data.size())
		return true;
	return false;
}



void newbresenham(double x1, double y1, double x2, double y2, std::vector<std::pair<int, int> >& result)
{
	result.clear();

	int floorx1 = floor(x1);
	int floorx2 = floor(x2);
	int floory1 = floor(y1);
	int floory2 = floor(y2);

    result.reserve(abs(floorx2 - floorx1) + 1 + abs(floory2 - floory1) + 1);

	// if they lie vertically (one above another), we cannot calculate k
	// so we direct solve this case
	if(floorx1 == floorx2)
	{
		// std::vector<int> L = intelist(y1, y2);
        if(floory1 < floory2)
        {
            for(int i = floory1; i <= floory2; ++i)
            {
                result.push_back(std::pair<int, int>(floorx1, i));
            }
        }
        else if(floory1 == floory2)
        {
            result.push_back(std::pair<int, int>(floorx1, floory1));
        }
        else
        {
            for(int i = floory1; i >= floory2; --i)
            {
                result.push_back(std::pair<int, int>(floorx1, i));
            }
        }
        return ;
	}

    if(floory1 == floory2)
    {
        if(floorx1 < floorx2)
        {
            for(int i = floorx1; i <= floorx2; ++i)
            {
                result.push_back(std::pair<int, int>(i, floory1));
            }
        }
        else if(floorx1 == floorx2)
        {
            result.push_back(std::pair<int, int>(floorx1, floory1));
        }
        else
        {
            for(int i = floorx1; i >= floorx2; --i)
            {
                result.push_back(std::pair<int, int>(i, floory1));
            }
        }
        return ;
    }

    // we get the function of the ray in y=kx+b
	double k = (y2 - y1)/(x2 - x1);
	double b = y1 - k*x1;

    int flag;
    
    std::vector<std::pair<int, int> > v;
    v.reserve(abs(floorx2 - floorx1));
    if(x1 < x2)
    {
        flag = 1;
        for(int i = floorx1+1; i <= floorx2; ++i)
        {
            v.push_back(std::pair<int, int>(i, floor(k*i+b)));
        }
    }
    else
    {
        flag = -1;
        for(int i = floorx1; i >= floorx2 + 1; --i)
        {
            v.push_back(std::pair<int, int>(i, floor(k*i+b)));
        }
    }
    
    int temp = sign(k)*flag;

    if(floorx2 > floorx1)
    {
        // we insert the start column
        if(floory1 < v.front().second)
        {
            for(int i = floory1; i <= v.front().second; ++i)
            {
                result.push_back(std::pair<int, int>(floorx1, i));
            }
        }
        else
        {
            for(int i = floory1; i >= v.front().second; --i)
            {
                result.push_back(std::pair<int, int>(floorx1, i));
            }
        }
        
        // we insert the intermediate column
        if(abs(floorx2 - floorx1) > 1)
        {

            // if the ray only passes two columns, then there is no intermediate columns
            for(unsigned int i = 0; i < v.size()-1; ++i)
            {
                int xinsert = v.at(i).first;
                int ystart = v.at(i).second;
                int yend = v.at(i+1).second;
                if(ystart < yend)
                {
                    for(int j = ystart; j <= yend; ++j)
                    {
                        result.push_back(std::pair<int, int>(xinsert, j));
                    }
                }
                else
                {
                    for(int j = ystart; j >= yend; --j)
                    {
                        result.push_back(std::pair<int, int>(xinsert, j));
                    }
                }
            }
            
        }
        
        // we insert the final column
        if(floory2 > v.back().second)
        {
            int xinsert = v.back().first;
            for(int i = v.back().second; i <= floory2; ++i)
            {
                result.push_back(std::pair<int, int>(xinsert, i));
            }
        }
        else
        {
            int xinsert = v.back().first;
            for(int i = v.back().second; i >= floory2; --i)
            {
                result.push_back(std::pair<int, int>(xinsert, i));
            }
        }

    }//floorx2 > floorx1
    else // floorx1 is greater than floorx2. We do things backwards
    {
        int xinsert = v.front().first;
        // we insert the start column
        if(floory1 < v.front().second)
        {
            for(int i = floory1; i <= v.front().second; ++i)
            {
                result.push_back(std::pair<int, int>(xinsert, i));
            }
        }
        else
        {
            for(int i = floory1; i >= v.front().second; --i)
            {
                result.push_back(std::pair<int, int>(xinsert, i));
            }
        }
        
        // we insert the intermediate column
        if(abs(floorx2 - floorx1) > 1)
        {

            // if the ray only passes two columns, then there is no intermediate columns
            for(unsigned int i = 0; i < v.size()-1; ++i)
            {
                int xinsert = v.at(i+1).first;//Here is different

                int ystart = v.at(i).second;
                int yend = v.at(i+1).second;
                if(ystart < yend)
                {
                    for(int j = ystart; j <= yend; ++j)
                    {
                        result.push_back(std::pair<int, int>(xinsert, j));
                    }
                }
                else
                {
                    for(int j = ystart; j >= yend; --j)
                    {
                        result.push_back(std::pair<int, int>(xinsert, j));
                    }
                }
            }
            
        }
        
        // we insert the final column
        if(floory2 > v.back().second)
        {
            for(int i = v.back().second; i <= floory2; ++i)
            {
                result.push_back(std::pair<int, int>(floorx2, i));
            }
        }
        else
        {
            for(int i = v.back().second; i >= floory2; --i)
            {
                result.push_back(std::pair<int, int>(floorx2, i));
            }
        }
    }//floorx2 < floorx1
}

double normalize_angle(const double angle)
{
	const double result = fmod(angle + M_PI, 2.0*M_PI);
	if (result <= 0.0) return result + M_PI;
	return result - M_PI;
}

double normalize_angle_positive(const double angle)
{
	const double result = fmod(angle, 2.0*M_PI);
	if (result < 0.0) return result + 2.0*M_PI;
	return result;
}

double normObsExit(const ObsExit& a, const ObsExit& b)
{
    double small = hypot(a.smallx - b.smallx, a.smally - b.smally);
    double big = hypot(a.bigx - b.bigx, a.bigy - b.bigy);
    return hypot(small, big);
}

double normRay(const OriDis& a, const OriDis& b)
{
    return hypot(a.endx - b.endx, a.endy - b.endy);
}

double pathLength(const std::vector<std::pair<int, int> >& path, int loc)
{
	if (path.empty())
		return std::numeric_limits<double>::infinity();

	if (loc < 0)
	{
		loc = path.size() - 1;
	}

	double L = 0;
	for (int i = 0; i < loc; ++i)
	{
		L += hypot(path.at(i + 1).first - path.at(i).first, path.at(i + 1).second - path.at(i).second);
	}
	return L;
}


double pathLength(const std::vector<std::pair<double, double> >& path, int loc)
{
    if(path.empty())
		return std::numeric_limits<double>::infinity();

    if(loc < 0)
    {
        loc = path.size() - 1;
    }

	double L = 0;
	for (int i = 0; i < loc; ++i)
	{
		L += hypot(path.at(i+1).first - path.at(i).first, path.at(i+1).second - path.at(i).second);
	}
	return L;
}

bool pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
{
  int i, j;
  bool c = false;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((verty[i]>testy) != (verty[j]>testy)) &&
     (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
       c = !c;
  }
  return c;
}

void preCalculateCspace()
{
	std::pair<int, int> center;
	center.first = LETHAL_RADIUS + 1; // for C++
	center.second = LETHAL_RADIUS + 1; // for C++

	int mask[(LETHAL_RADIUS + 1) * 2 + 1][(LETHAL_RADIUS + 1) * 2 + 1];

	for (unsigned int i = 0; i < (LETHAL_RADIUS + 1) * 2 + 1; ++i)
	{
		for (unsigned int j = 0; j < (LETHAL_RADIUS + 1) * 2 + 1; ++j)
		{
			if (sqrt((i - center.first)*(i - center.first) + (j - center.second)*(j - center.second)) <= LETHAL_RADIUS)
			{
				mask[i][j] = 127;
			}
			else
			{
				mask[i][j] = 0;
			}
		}
	}

	int data_temp[MAX_X_SIZE*MAX_Y_SIZE];
	memcpy(data_temp, costmap_.data, sizeof(int)*MAX_X_SIZE*MAX_Y_SIZE);
	for (int i = 0; i < MAX_X_SIZE; ++i)
	{
		for (int j = 0; j < MAX_Y_SIZE; ++j)
		{
			if (costmap_.data[j*MAX_X_SIZE + i] == 254) // an obstacle point
			{
				// paste the mask onto the map
				for (int k = 0; k < (LETHAL_RADIUS + 1) * 2 + 1; ++k)
				{
					for (int l = 0; l < (LETHAL_RADIUS + 1) * 2 + 1; ++l)
					{
						int x = i - (LETHAL_RADIUS+1) + k;
						int y = j - (LETHAL_RADIUS+1) + l;
						if (x >= 0 && x < MAX_X_SIZE && y >= 0 && y < MAX_Y_SIZE)
						{
							data_temp[y*MAX_X_SIZE + x] = std::max(data_temp[y*MAX_X_SIZE + x], mask[k][l]);
						}
					}
				}
			}
		}
	}

	memcpy(costmap_.data, data_temp, sizeof(int)*MAX_X_SIZE*MAX_Y_SIZE);

}

bool preciseEnough(const OriDis& a, const OriDis& b)
{
    double nearraylength = std::min(a.dis, b.dis);
    
    return fabs(nearraylength * sin(a.theta - b.theta)) < 0.05;
}

void printOpenlist()
{
	logfile << "Size of Nodelist: " << TheRobot(0)->N_.size() << std::endl;

	logfile << "Printing openlist: " << std::endl;
	for (auto iter = TheRobot(0)->openlist_.begin(); iter != TheRobot(0)->openlist_.end(); ++iter)
	{
		logfile << iter->Nindex << ", " << iter->Cindex << ", "
			<< TheRobot(0)->N_[iter->Nindex].C_[iter->Cindex].ckiGcost_
			<< ", ";
		for (unsigned int i = 0; i < ::goals.size(); ++i)
		{
			logfile << iter->Fcost[i] << ", ";
		}
			
		logfile << TheRobot(0)->N_[iter->Nindex].C_[iter->Cindex].cki_.first 
			<< ", " << TheRobot(0)->N_[iter->Nindex].C_[iter->Cindex].cki_.second
			<< std::endl;
	}
}
void printPartialOrder()
{
	logfile << "Printing partial order:" << std::endl;
	for (auto iter = TheRobot(0)->partial_order_.begin(); iter != TheRobot(0)->partial_order_.end(); ++iter)
	{
		logfile << "[" << iter->smallN << ", " << iter->smallC << ", " << iter->bigN << ", " << iter->bigC << "]" << std::endl;
	}
}
void printSolutionCost()
{
	//logfile << "Cost of existing solutions: [";

	//for (auto iter = multi_bestcost.begin(); iter != multi_bestcost.end(); ++iter)
	//	logfile << *iter << " ";

	//logfile << "]" << std::endl;

	logfile << "Cost of existing solutions: " << std::endl;
	for (unsigned int i = 0; i < ::goals.size(); ++i)
	{
		logfile << "[";
		for (auto iter = multi_goal_multi_bestcost[i].begin(); iter != multi_goal_multi_bestcost[i].end(); ++iter)
			logfile << *iter << " ";
		logfile << "]" << std::endl;
	}

}

void removePoints(std::vector<std::pair<int, int> >& P, int xmin, int ymin,
	int xmax, int ymax)
{
	// remove the points that are out of the map, ensuring subsequent collision checking model receives valid (x, y)
	// xmin, ymin, xmax, ymax are the critical valid indices, such as 0, 0, 239, 179 but not -1, -1, 240, 180
	P.erase(std::remove_if(P.begin(), P.end(), 
		[&](std::pair<int, int> a){return a.first < xmin || a.first > xmax || a.second < ymin || a.second > ymax;}), 
		P.end());
}

bool straightPath(std::pair<double, double> start, std::pair<double, double> goal)
{
    // true: the straight path is collision-free
    std::vector<std::pair<int, int> > P;
    classicBresenham(start.first, start.second, goal.first, goal.second, P);
    for(unsigned int i = 0; i < P.size(); i++)
    {
        if(CC(P.at(i).first, P.at(i).second) == -1)
        {
            return false;
        }
    }
    return true;
}
