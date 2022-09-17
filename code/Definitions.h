#ifndef _DEFINITIONS_
#define _DEFINITIONS_

#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>

// Claim some pre-definitions
#define OBSTACLE 254
#define COBSTACLE 127
#define CFREE 0

#define MAX_X_SIZE 240
#define MAX_Y_SIZE 240

#define LETHAL_RADIUS 4

#define INMAP(x, y) (x >= 0 && x < MAX_X_SIZE && y >= 0 && y < MAX_Y_SIZE)
#define MODIFYMAP(x, y, cost) costmap_.data[y*MAX_X_SIZE + x] = cost

// Always true
#define MULTI_GOAL true


#define FINITE_LENGTH 250

#if FINITE_LENGTH > 0
	#define EXHAUSTIVE true
#else
	#define EXHAUSTIVE false
#endif

#define NON_SELFCROSSING true

// Claim some structs

struct OriDis
{
	double theta;
	double dis;
	double endx;
	double endy;

	bool infinite;
	double Cdis;
	double start_dis;
	int smallGap;
	// 0: it is not the short ray of a gap
	// 1: it is the small ray of a gap, and the gap has been exploited
	// -1: the distance is greater than 2R, but it is invalid (just for reserve)

	OriDis()
	{
		smallGap = 0;
	}
	OriDis(const OriDis& a)
	{
		theta = a.theta;
		dis = a.dis;
		endx = a.endx;
		endy = a.endy;
		infinite = a.infinite;
		Cdis = a.Cdis;
		start_dis = a.start_dis;
		smallGap = a.smallGap;
	}
};

struct ObsExit{
	double thetasmall;
	double dissmall;
	double smallx;
	double smally;
	double thetabig;
	double disbig;
	double bigx;
	double bigy;

	ObsExit():thetasmall(-1), dissmall(-1), smallx(-1), smally(-1), thetabig(-1), disbig(-1), bigx(-1), bigy(-1)
	{

	}
};

struct Candidate{
	int Nindex;
	int Cindex;
	
//	double Fcost;
	std::vector<double> Fcost;
	double leastFcost;

	Candidate(const Candidate& a)
	{
		Nindex = a.Nindex;
		Cindex = a.Cindex;
//		Fcost = a.Fcost;
		Fcost.assign(a.Fcost.begin(), a.Fcost.end());
		leastFcost = a.leastFcost;
	}
	Candidate(int chart, int corridor, const std::vector<double> cost)
	{
		Nindex = chart;
		Cindex = corridor;

		Fcost.assign(cost.begin(), cost.end());
		leastFcost = *std::min_element(Fcost.begin(), Fcost.end());
	
//		Fcost = cost;
	}

	bool operator()(const Candidate& a, const Candidate& b) const
	{
		return a.leastFcost > b.leastFcost;
//		return a.Fcost > b.Fcost;
	}

};

struct Sweeper{
	int Rindex;//Maybe we don't use it. Just record it
	int Nindex;
	int Cindex; 
	std::pair<double, double> cki;
	std::pair<double, double> obs;
	double ckiGcost;
	Sweeper(int R, int N, int C, const std::pair<double, double>& c, const std::pair<double, double>& o, double g)
	{
		Rindex = R;
		Nindex = N;
		Cindex = C;
		cki = c;
		obs = o;
		ckiGcost = g;
	}
};

struct Path{
	int Rindex;
	int Nindex;
	int Cindex;
	std::pair<double, double> pi;
	std::pair<double, double> cki;
	double piGcost;
	Path(int R, int N, int C, const std::pair<double, double>& p, const std::pair<double, double>& c, double g)
	{
		Rindex = R;
		Nindex = N;
		Cindex = C;
		pi = p;
		cki = c;
		piGcost = g;
	}
};

struct PartialOrder
{
	int smallN;
	int smallC;
	int bigN;
	int bigC;
	PartialOrder(int sn, int sc, int bn, int bc):smallN(sn), smallC(sc), bigN(bn), bigC(bc){}
};

struct Path2D {
	// replace the geometry_msgs::Path in ROS
	std::vector<std::pair<double, double> > data;
	Path2D()
	{
		data.clear();
	}

	Path2D(const std::vector<std::pair<double, double> >& p)
	{
		data.assign(p.begin(), p.end());
	}
	void operator=(const Path2D& p)
	{
		data.assign(p.data.begin(), p.data.end());
	}
};

struct Costmap{
	// replace the costmap_2d in ROS
	int data[MAX_X_SIZE*MAX_Y_SIZE];
	Costmap()
	{
		memset(&data, 0, sizeof(int)*MAX_X_SIZE*MAX_Y_SIZE);
	}
};


// Claim some global variables
extern Costmap costmap_;
extern int cellsize_x;
extern int cellsize_y;

extern std::pair<double, double> start_location;

extern std::vector<std::pair<int, int> > footprint;
extern std::vector<std::pair<int, int> > detailed_footprint;
extern std::vector<std::pair<double, double> > goals;
//extern std::vector<double> bestcost;
//extern std::vector<Path2D> solutions;

extern std::vector<std::vector<Path2D> > multi_goal_multi_solutions;
extern std::vector<std::vector<int> > multi_goal_multi_nindex_for_solutions;
//extern std::vector<Path2D> multi_solutions;
//extern std::vector<int> multi_nindex_for_solutions;

extern std::vector<std::vector<double> > multi_goal_multi_bestcost;
//extern std::vector<double> multi_bestcost;

extern int num_of_nonhomo_path;

extern std::ofstream logfile;

#endif