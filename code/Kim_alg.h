#pragma once

#include <vector>
#include <string>
#include "Definitions.h"

namespace Kim2013{

typedef std::vector<int> Indices;

class KimNode
{
public: 
    int x_;
    int y_;
    double g_;
    double f_;
    std::vector<int> hindex_;

    int index_;
    int fatherindex_;

    bool open_;

	std::vector<int> adj_kimnode_;

//	std::vector<int> predecessors_; // This is for non-self-crossing checking. 

    KimNode()
    {
        g_ = 0;
		hindex_.clear();
        fatherindex_ = -1;
        open_ = true;
    }
    KimNode(int x, int y, double g, double f, std::vector<int> hindex, int index, int fatherindex)
    {
        x_ = x;
        y_ = y;
        g_ = g;
        f_ = f;
        hindex_ = hindex;
        


        index_ = index;
        fatherindex_ = fatherindex;

		adj_kimnode_.clear();
        open_ = true;
    }
    KimNode(const KimNode& a)
    {
        x_ = a.x_;
        y_ = a.y_;
        g_ = a.g_;
        f_ = a.f_;

        hindex_ = a.hindex_;

        index_ = a.index_;
        fatherindex_ = a.fatherindex_;

		adj_kimnode_ = a.adj_kimnode_;

        open_ = a.open_;
    }
    void operator=(const KimNode& a)
    {
        x_ = a.x_;
        y_ = a.y_;
        g_ = a.g_;
        f_ = a.f_;

        hindex_ = a.hindex_;

        index_ = a.index_;
        fatherindex_ = a.fatherindex_;

		adj_kimnode_ = a.adj_kimnode_;

        open_ = a.open_;        
    }

    void sons(std::vector<std::pair<int, int> >& A)
    {
        A.clear();
        A.push_back(std::pair<int, int>(x_-1, y_-1));
        A.push_back(std::pair<int, int>(x_-1, y_));
        A.push_back(std::pair<int, int>(x_-1, y_+1));
        A.push_back(std::pair<int, int>(x_, y_-1));
        A.push_back(std::pair<int, int>(x_, y_+1));
        A.push_back(std::pair<int, int>(x_+1, y_-1));
        A.push_back(std::pair<int, int>(x_+1, y_));
        A.push_back(std::pair<int, int>(x_+1, y_+1));
    }
};

class Kim2013{
    public:
        Kim2013();
		~Kim2013() {
			for (unsigned int i = 0; i < MAX_X_SIZE; ++i)
			{
				delete[] I_[i];
			}
			delete[] I_;
		}

        std::vector<int> calSwing(const std::vector<int>& oldH, int oldx, int oldy, int newx, int newy);

        bool C2LPlanning(const KimNode& startnode, const std::pair<int, int>& goalLocation, std::vector<std::pair<int, int> >& result_path, KimNode& goalNode);

        int findEqualHCNode(const std::vector<int>& newHindex, const std::vector<KimNode>& theStorage, Indices** theI, int son_x, int son_y);

        bool isEqualHC(const std::vector<int>& newH, const std::vector<KimNode>& theStorage, Indices** theI, int son_x, int son_y);

        bool isSelfCrossing(const std::vector<std::pair<int, int> >& P);

		void new_optimal_tp(const Path2D& ref_path, std::pair<double, double> goal_location, Path2D& result_path);
		void optimal_tp(const Path2D& ref_path, std::pair<double, double> goal_location, Path2D& result_path);

        void preCalculateAllConfigurations();

        void sedFill(std::vector<std::pair<int, int> >& ObsPosition);

		void setStart(std::pair<double, double> start_location)
		{
			start_point_.first = floor(start_location.first);
			start_point_.second = floor(start_location.second);
		}

        void tracePath(const KimNode& goalnode, std::vector<std::pair<int, int> >& result_path, const std::vector<KimNode>& theStorage,
            const KimNode& startnode);
        std::vector<KimNode> Storage_;

    private:

        std::vector<KimNode> Queue_;
//		Indices I_[MAX_X_SIZE][MAX_Y_SIZE];
		Indices** I_;

        std::pair<int, int>  start_point_;

        std::vector<std::pair<int, int> > g_obs_;
};

} // end namespace Kim2013

