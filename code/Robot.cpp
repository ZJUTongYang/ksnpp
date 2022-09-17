#include "Robot.h"
#include <iostream>

Robot* Robot::global_ = nullptr;

Robot::Robot()
{
	this->clear();
}


std::vector<std::pair<int, int> > Robot::findAllBetterLeafNode(int Nindex, int Cindex)
{
	std::vector<std::pair<int, int> > better;
	better.clear();

	std::vector<std::pair<int, int> > bettertemp;
	for (unsigned int i = 0; i < partial_order_.size(); ++i)
	{
		if (partial_order_[i].bigN == Nindex && partial_order_[i].bigC == Cindex)
		{
			bettertemp.push_back(std::pair<int, int>(partial_order_[i].smallN, partial_order_[i].smallC));
		}
	}

	if (bettertemp.empty())
		return better;

	for (unsigned int i = 0; i < bettertemp.size(); ++i)
	{
		std::vector<std::pair<int, int> > sub_better = findAllBetterLeafNode(bettertemp[i].first, bettertemp[i].second);
		better.insert(better.end(), sub_better.begin(), sub_better.end());
	}
}

std::vector<std::pair<int, int> > Robot::getUnsolvedLeafNodes(int Nindex, int Cindex)
{
	std::vector<std::pair<int, int> > sons;
	sons.clear();
	int sonNindex = N_[Nindex].C_[Cindex].son_Nindex_;
	for (unsigned int i = 0; i < N_[sonNindex].C_.size(); ++i)
	{
		if (N_[sonNindex].C_[i].status_ == "solved")
		{
			continue;
		}
		if (N_[sonNindex].C_[i].son_Nindex_ == -1)
		{
			sons.push_back(std::pair<int, int>(sonNindex, i));
		}
		else
		{
			std::vector<std::pair<int, int> > sub_leafnodes = getUnsolvedLeafNodes(sonNindex, i);
			sons.insert(sons.end(), sub_leafnodes.begin(), sub_leafnodes.end());
		}
	}

	return sons;
}

void Robot::inheritPartialOrder()
{
	// If a leaf node is better than a branch node, then it is better than all child nodes of the branch node
	int i = partial_order_.size()-1;
	while (i >= 0)
	{
		auto temp = partial_order_[i];
		if (!isLeafNode(temp.smallN, temp.smallC))
		{
			partial_order_.erase(partial_order_.begin() + i);
			i = i - 1;
			continue;
		}

		if (!isLeafNode(temp.bigN, temp.bigC))
		{
			// Inherit to all its leaf nodes
			// sons: n*2 array, [sonN, sonC]
			std::vector<std::pair<int, int> > sons = getUnsolvedLeafNodes(temp.bigN, temp.bigC);

			// 2021.7.15 YT: Here we should preserve all leaf nodes (including the solved ones), because, 
			// they may perform like the bridge between different partial order relation
			// 2021.7.15 later YT: we should remove the leaf nodes of the solved node, because better or worse than them does not mean anything

			partial_order_.erase(partial_order_.begin()+i);
			for (unsigned int j = 0; j < sons.size(); ++j)
			{
				PartialOrder newtemp(temp.smallN, temp.smallC, sons[j].first, sons[j].second);
				insertPartialOrder(newtemp);
			}
			i = partial_order_.size()-1;
			continue;

		}
		i = i - 1;
	}
}

void Robot::insertPartialOrder(const PartialOrder& temp)
{
	partial_order_.push_back(temp);
}

bool Robot::isLeafNode(int Nindex, int Cindex)
{
	if (N_[Nindex].C_[Cindex].son_Nindex_ == -1)
		return true;
	return false;
}

void Robot::removeUnnecessaryLeafNode()
{
	for (int i = openlist_.size() - 1; i >= 0; --i)
	{
		std::vector<std::pair<int, int> > better = findAllBetterLeafNode(openlist_[i].Nindex, openlist_[i].Cindex);
		if (better.empty())
			continue;

		std::sort(better.begin(), better.end());
		auto iter = std::unique(better.begin(), better.end());
		better.erase(iter, better.end());

		// 2022.06.14 Since we are implementing multi-goal settings, we should use the min number 
		std::vector<int> multi_goal_num_better_solutions;
		multi_goal_num_better_solutions.resize(::goals.size(), 0);
		for (unsigned int k = 0; k < ::goals.size(); ++k)
		{
			double h = openlist_[i].Fcost[k];
			for (unsigned int j = 0; j < multi_goal_multi_bestcost[k].size(); ++j)
			{
				if (multi_goal_multi_bestcost[k][j] < h)
					++multi_goal_num_better_solutions[k];
			}
		}
		int num_better_solutions = *std::min_element(multi_goal_num_better_solutions.begin(), multi_goal_num_better_solutions.end());

		//// For each element, we check how many existing solutions are better than this
		//double h = openlist_[i].Fcost;
		//int num_better_solutions = 0;
		//for (unsigned int j = 0; j < multi_bestcost.size(); ++j)
		//{
		//	if (multi_bestcost[j] < h)
		//		++num_better_solutions;
		//}

		// If there have been enough better results, we remove this. 
		if (better.size() + num_better_solutions >= num_of_nonhomo_path)
		{
			logfile << "[N, C] = [" << openlist_[i].Nindex << ", " << openlist_[i].Cindex 
				<< "] has " << better.size() << "number of better homotopies and " 
				<< num_better_solutions << "better solutions and is removed" << std::endl;


			N_[openlist_[i].Nindex].C_[openlist_[i].Cindex].status_ = "unnecessary";
			openlist_.erase(openlist_.begin() + i);
		}
	}
}


void Robot::showDetailOpenlist()
{
	std::cout << "show openlist: [Nindex, Cindex, piGcost, ckiGcost, Fcost]" << std::endl;
	if (!openlist_.empty())
	{
		for (auto iter = openlist_.begin(); iter != openlist_.end(); ++iter)
		{
			std::cout << "[" << iter->Nindex << ", " << iter->Cindex << ", "
				<< N_.at(iter->Nindex).C_.at(iter->Cindex).piGcost_ << ", "
				<< N_.at(iter->Nindex).C_.at(iter->Cindex).ckiGcost_ << ", ";

			for (auto iter2 = iter->Fcost.begin(); iter2 != iter->Fcost.end(); ++iter2)
				std::cout << *iter2 << ", ";

			std::cout << iter->leastFcost << "]" << std::endl;
		}
	}
}

bool Robot::betterNode(const Candidate& a, const Candidate& b)
{
	for (unsigned int i = 0; i < ::goals.size(); ++i)
	{
		if (a.Fcost[i] >= b.Fcost[i])
			return false;
	}
	return true;
}

