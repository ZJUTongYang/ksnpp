#ifndef _CORRIDOR_
#define _CORRIDOR_

#include <string>
#include <vector>
#include "Definitions.h"

class Corridor
{
public: 
	int Rindex_;
	int Nindex_;
	int Cindex_;
	std::pair<double, double> pi_;
	
	std::pair<double, double> width_;
	std::pair<double, double> mid_;
	std::pair<double, double> cki_;
	
	std::pair<double, double> cki_mirror_;
	double angle_long_;
	double angle_short_;
	double piGcost_;
	double ckiGcost_;

	std::vector<double> ckiFcost_;
//	double ckiFcost_;

	std::string status_;
	 // -1: of no usage, 0: has explored, 1:open
     // 'no_usage', 'explored', 'open'
     // 'open': Now that we don't use lazy implementation, it means that
     // the corridor has a valid local path, and has been pushed into the
     // openlist
     // 'expanded': After this node has generated a new node, we mark it
     // 'no_usage': If the corridor loses optimality, we mark it as such,
     // and note that if the node has been expanded, then all its
     // predecessors must be also marked as 'no_usage'
     // The difference between 'expanded' and 'no_usage' is: When we
     // rewire an old tree to a new tree, if the corridor is 'no_usage',
     // it should be re-opened, but if it is 'expanded', we should remain
     // this. 
     // 'blocked': Only if all its predecessor are 'only son's, we close
     // them backwardly (see paper for details)

	double sweep_dis_;
	std::pair<double, double> sweep_hitting_obstacles_;
	
	// std::vector<std::pair<int, int> > sweeperGrids_;
	// std::vector<std::pair<int, int> > pathGrids_;
	// Storing the position of points on the gap sweeper. 
	// When we create the corridor, we should record the points here,
	// but not directly draw them in the Robot's map, because we aren't
	// sure whether the index of this corridor will change. 
	// After all corridors of this node have been found path, we draw
	// them in the map through function "newRegisterNewCandidate"

	std::pair<double, double> near_obstacle_;
	double xposmax_;

	// for future exploration
	double start_angle_;
	double end_angle_;

	int son_Nindex_;

	Path2D myPath_;

	double myLength_;

	Corridor(){}
	Corridor(int Rindex, int Nindex, int Cindex, std::pair<double, double> seed, 
		ObsExit a_cc_exit, double gcost);
	Corridor(const Corridor& a)
	{
		Rindex_ = a.Rindex_;
		Nindex_ = a.Nindex_;
		Cindex_ = a.Cindex_;
		pi_ = a.pi_;
		pi_fake_ = a.pi_fake_;
		fake_pi_path_ = a.fake_pi_path_;
		width_ = a.width_;
		mid_ = a.mid_;
		cki_ = a.cki_;
		cki_fake_ = a.cki_fake_;
		fake_cki_path_ = a.fake_cki_path_;
		cki_mirror_ = a.cki_mirror_;
		angle_long_ = a.angle_long_;
		angle_short_ = a.angle_short_;
		piGcost_ = a.piGcost_;
		ckiGcost_ = a.ckiGcost_;
		ckiFcost_ = a.ckiFcost_;

		status_ = a.status_;
		sweep_dis_ = a.sweep_dis_;
		sweep_hitting_obstacles_ = a.sweep_hitting_obstacles_;
		
		near_obstacle_ = a.near_obstacle_;
		xposmax_ = a.xposmax_;
		start_angle_ = a.start_angle_;
		end_angle_ = a.end_angle_;
		son_Nindex_ = a.son_Nindex_;
		myPath_ = a.myPath_;
		myLength_ = a.myLength_;

		Trans1_ = a.Trans1_;
		Trans2_ = a.Trans2_;
		Trans3_ = a.Trans3_;

		me_ = a.me_;
	}

	bool basicChecking(std::vector<std::pair<int, int> >& obs_list);
	double gapSweeper();
	void getPredecessors(std::vector<int>& fatherlink);
	bool isInT(std::pair<double, double> query_point);
	bool isLeftOrRight();
	void pathSweeper();
	void removeOptimality();
	void removeUsage();
	bool safePathFindingInWorld(std::vector<std::pair<int, int> > &phy_obstacle_list);
	void setCost();
	void setPath(const Path2D& newpath);
	void setPath();
	void tracePath(int untilfather, Path2D& result);
private:

	/*
	2020.05.18 we try to adapt three coordinate
	(1) orthonormal frame, y = long ray, x points outside
	(2) left-side affine frame, y = long ray, (not normalized)
	(3) right-side affine frame, y = long ray, (not normalized)
	format: 
	[\bar{x}]   [a, b, px][x]         [x]
	[\bar{y}] = [c, d, py][y] = transx[y]
	[1      ]   [0, 0, 1 ][1]         [1]
	*/

	std::vector<double> Trans1_, Trans2_, Trans3_;

	std::pair<double, double> pi_fake_;
	Path2D fake_pi_path_;

	std::pair<double, double> cki_fake_;
	Path2D fake_cki_path_;

	ObsExit me_;


};

#endif