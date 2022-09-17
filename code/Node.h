#ifndef _NODE_
#define _NODE_

#include <utility>
#include <vector>
#include <string>
#include <list>
#include "Atom.h"
#include "Corridor.h"
#include <iostream>

struct OriDis;
struct ObsExit;
class Corridor;

class Node
{
public:
	int Rindex_;
	int Nindex_;
	std::pair<double, double> seed_;
	double start_angle_;
	double end_angle_;
	std::list<OriDis> point_ori_dis_array_;

	std::vector<ObsExit> obs_exit_;
	std::vector<Corridor> C_;
	double Gcost_;
	int fatherindex_;
	int soninfatherindex_;

//	std::vector<std::pair<int, int> > myPath_;
	Path2D myPath_;

	Node(const Node& a)
	{
		Rindex_ = a.Rindex_;
		Nindex_ = a.Nindex_;
		seed_ = a.seed_;
		start_angle_ = a.start_angle_;
		end_angle_ = a.end_angle_;
		point_ori_dis_array_.assign(a.point_ori_dis_array_.begin(), 
			a.point_ori_dis_array_.end());
		obs_exit_.assign(a.obs_exit_.begin(), a.obs_exit_.end());

		C_.assign(a.C_.begin(), a.C_.end());

		Gcost_ = a.Gcost_;
		fatherindex_ = a.fatherindex_;
		soninfatherindex_ = a.soninfatherindex_;

//		myPath_.assign(a.myPath_.begin(), a.myPath_.end());
		myPath_ = a.myPath_;
	}

	Node(){}
	Node(int Rindex, int nodeindex, std::pair<double, double> seedpoint);
	Node(int Rindex, int nodeindex, std::pair<double, double> seedpoint, 
		double start_angle, double end_angle, double Gcost, int fatherindex, 
		int soninfatherindex);

	inline double adjustAngle(double theta)
	{
		if(theta > start_angle_ && theta < end_angle_)
			return theta;
		if(theta < start_angle_)
		{
			theta = start_angle_ + normalize_angle_positive(theta - start_angle_);
			return theta;
		}
		std::cout << "YT: Angle error, we shouldn't reach here" << std::endl;
	}

	void ccRayExplore();
	std::vector<ObsExit> collectObsExit(std::vector<int>& obs_exit_index);
	//void createBlock();
	void dealWithHiddenObstacle(const std::vector<std::pair<int, int> >& obs_list);
	void Expand();
	int insertCorridor(const Corridor& newC);
	bool isInNode(const std::pair<double, double>& query_point, int& Cindex);

	void markTheIthRay(int i);

	void refineCCRay();
	void refineCCRay(const std::list<OriDis>::iterator& begin, const std::list<OriDis>::iterator& end);
	
	void removeCorridor(int index);
	void removeOptimality();
	void removeUsage();
	
	int safePathFinding(std::pair<double, double> goal, Path2D& current_path);
	void tracePath(Path2D& result, int untilfather = 0);

	void showDetails()
	{
		std::cout << "Node " << Nindex_ 
			<< ": seed position [" << seed_.first << ", " << seed_.second << "]" 
			<< ", start_angle_: " << start_angle_ << ", end_angle_: " << end_angle_ 
			<< ", number of corridors: " << C_.size()
			<< ", fatherindex: " << fatherindex_ 
			<< ", son_in_father_index: " << soninfatherindex_ << std::endl; 
	}

};


#endif