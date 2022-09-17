#ifndef _ROBOT_
#define _ROBOT_

#include <map>
#include <vector>
#include <utility>
#include "Node.h"
#include "Corridor.h"
#include "Atom.h"

struct Candidate;
struct Sweeper;
struct Path;

class Robot{
public:
	static Robot* Instance(int index)
	{
		if(index == 0)
		{
			if(global_ == nullptr)
			{
				global_ = new Robot();
			}
			return global_;
		}
	}

	void clear()
	{
		openlist_.clear();
		N_.clear();
		sweeper_list_.clear();
		path_list_.clear();
	}

	// leaf nodes': nindex, cindex
	std::vector<std::pair<int, int> > getUnsolvedLeafNodes(int Nindex, int Cindex);

	void inheritPartialOrder();

	void insertPartialOrder(const PartialOrder& temp);
	
	bool isLeafNode(int Nindex, int Cindex);

	void removeUnnecessaryLeafNode();

	void showDetailOpenlist();

	bool betterNode(const Candidate& a, const Candidate& b);

	std::vector<std::pair<int, int> > findAllBetterLeafNode(int Nindex, int Cindex);

	std::vector<Candidate> openlist_;

	std::vector<Sweeper> sweeper_list_;
	std::vector<Path> path_list_;

	std::vector<Node> N_;

	std::vector<PartialOrder> partial_order_;

private:
	Robot();

	static Robot* global_;
};

#define TheRobot(x) Robot::Instance(x) 

#endif