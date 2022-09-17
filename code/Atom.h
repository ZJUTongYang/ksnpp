#ifndef _BRESENHAM_
#define _BRESENHAM_

#include <list>
#include <vector>
#include "Map_functions.h"

template<class T>
std::pair<int, int> floor(std::pair<T, T> a)
{
	return std::pair<int, int>(floor(a.first), floor(a.second));
}

template<typename T>
int sign(const T &a){
	return (a > 0) - (a<0);
}

template<class T1, class T2>
std::pair<T1, T2> operator+(std::pair<T1, T2> a, std::pair<T1, T2> b)
{
	return std::pair<T1, T2>(a.first+b.first, a.second+b.second);
}

template<class T1, class T2>
std::pair<T1, T2> operator-(std::pair<T1, T2> a, std::pair<T1, T2> b)
{
	return std::pair<T1, T2>(a.first-b.first, a.second-b.second);
}

template<class T1, class T2>
std::vector<std::pair<T1, T2> > operator+(std::vector<std::pair<T1, T2> > A, std::pair<T1, T2> b)
{
	std::vector<std::pair<T1, T2> > result;
	if(!A.empty())
	{
		for(unsigned int i = 0; i < A.size(); i++)
			result.push_back(A.at(i) + b);
	}
	return result;
}

template<class T1, class T2, class T3, class T4>
std::vector<std::pair<T1, T2> > operator-(std::vector<std::pair<T1, T2> > A, std::pair<T3, T4> b)
{
	std::vector<std::pair<T1, T2> > result;
	if(!A.empty())
	{
		for(unsigned int i = 0; i < A.size(); i++)
		{
			std::pair<T1, T2> temp;
			temp.first = A.at(i).first - b.first;
			temp.second = A.at(i).second - b.second;
			result.push_back(temp);
		}
	}
	return result;
}


template<class T1, class T2>
std::vector<std::pair<double, double> > operator*(double b, std::vector<std::pair<T1, T2> > a)
{
	std::vector<std::pair<double, double> > result;
	for(unsigned int i = 0; i < a.size(); i++)
	{
		result.push_back(std::pair<double, double>(a.at(i).first*b, a.at(i).second*b));
	}
	return result;
}

template<class T1, class T2>
std::pair<double, double> operator*(double b, std::pair<T1, T2> a)
{
	std::pair<double, double> result(a);
	result.first *= b;
	result.second *= b;
	return result; 
}

template<class T1, class T2>
std::pair<double, double> operator*(std::pair<T1, T2> a, double b)
{
	std::pair<double, double> result(a);
	result.first *= b;
	result.second *= b;
	return result; 
}

template<class T1, class T2, class T3>
std::pair<T1, T2> operator/(std::pair<T1, T2> a, T3 b)
{
	std::pair<T1, T2> result(a);
	result.first /= b;
	result.second /= b;
	return result; 
}

template<class T1, class T2>
double norm(std::pair<T1, T2> a)
{
	return sqrt(a.first*a.first+a.second*a.second);
}

//template<class T1, class T2>
//bool operator==(const std::pair<T1, T2>& a, const std::pair<T1, T2>& b)
//{
//	return a.first == b.first && a.second == b.second;
//}


struct Candidate;
struct Sweeper;
struct Path;

double calStartDis(double thetasmall, double dis1, double Cdis1, 
	double thetabig, double dis2, double Cdis2, double radius);

ObsExit chooseOneObsExit(std::vector<ObsExit>& obs_exit, const std::vector<int>& obs_exit_index, 
    int& nearest_obs_exit_index);

void classicBresenham(double x1, double y1, double x2, double y2, std::vector<std::pair<int, int> >& result);

void computeFootprint(int lethal_radius);

void constructFakeCorridorParams(const std::pair<double, double> start, const std::pair<double, double> goal, 
	std::pair<double, double>& origin, std::pair<double, double>& dx, std::pair<double, double>& dy, 
	std::pair<double, double>& n_dx, double& xposmax, double& xnegmax, double& ymax);

//bool currentIsOptimal(int index, int goal_index = 0);

std::pair<double, double> findHitting(int cell_x, int cell_y, 
	std::pair<double,double> seed, double theta);

bool findIntersection(const Sweeper& gap, const Path& path, std::pair<double, double>& q, int& loc);

bool findIntersection(const Sweeper& gap1, const Sweeper& gap2, 
    std::pair<double, double>& q);

// bool hasBeenExploited(const OriDis& small, const OriDis& big);

extern bool isGoalFree(const std::vector<std::pair<double, double> >& bd);
extern bool isInNarrowCorridor(std::pair<double, double> world_p, std::pair<double, double> origin, 
    std::pair<double, double> dx, std::pair<double, double> dy, 
    std::pair<double, double> n_dx, double xposmax, double xnegmax, double ymax);

extern bool isSelfIntersect(const Path2D& current_path);

void newbresenham(double x1, double y1, double x2, double y2, std::vector<std::pair<int, int> >& result);

double normalize_angle(const double theta);

double normalize_angle_positive(const double theta);

double normObsExit(const ObsExit& a, const ObsExit& b);
double normRay(const OriDis& a, const OriDis& b);

double pathLength(const std::vector<std::pair<int, int> >& path, int loc = -1);
double pathLength(const std::vector<std::pair<double, double> >& path, int loc = -1);

bool pnpoly(int nvert, float *vertx, float *verty, float testx, float testy);

void preCalculateCspace();

bool preciseEnough(const OriDis& a, const OriDis& b);

void printOpenlist();
void printPartialOrder();
void printSolutionCost();

void removePoints(std::vector<std::pair<int, int> >& P, int xmin, int ymin,
	int xmax, int ymax);

bool straightPath(std::pair<double, double> start, std::pair<double, double> goal);

// std::vector<int> wiring(int new_Nindex, int new_Cindex, int new_tree_index, int old_tree_index);


#endif