#include "Relative_optimality.h"
#include "Definitions.h"
#include "Robot.h"
#include "Atom.h"
#include <vector>

int greater(int node1, int corridor1, int node2, int corridor2)
{
	//in our old implementation, the corridor with larger angle_long is
	//"bigger" brother
	//t: 1 = father1 > father2
	//   0 = father1 = father2
	//   -1 = father1 < father2
	if (corridor1 == corridor2)
	{
		return 0;
	}

	double angle1 = TheRobot(0)->N_[node1].C_[corridor1].angle_long_;
	double angle2 = TheRobot(0)->N_[node2].C_[corridor2].angle_long_;
	if (normalize_angle(angle1 - angle2) > 0)
	{
		return 1;
	}
	else
	{
		return -1;
	}
}

int Bro(int Rindex, int Nindex1, int Cindex1, int Nindex2, int Cindex2, int& commonfather)
{
    // 1: (N1, C1) is the elder brother of (N2, C2)
    // -1: (N1, C1) is the younger brother of (N2, C2)
    // 0: One is the direct successor of another

    std::vector<int> father1, father2;

    TheRobot(Rindex)->N_.at(Nindex1).C_.at(Cindex1).getPredecessors(father1);
    TheRobot(Rindex)->N_.at(Nindex2).C_.at(Cindex2).getPredecessors(father2);

    int i = 1;// in MATLAB here is 2
    while(i <= father1.size() - 1 && i <= father2.size() - 1)
    {
//        if(father1.at(i) > father2.at(i))
		if(greater(father1[i-1], father1[i], father2[i-1], father2[i]) == 1)
        {
            commonfather = father1.at(i-1);
            return 1;
        }
//        else if(father1.at(i) < father2.at(i))
		else if (greater(father1[i-1], father1[i], father2[i-1], father2[i]) == -1)
		{
            commonfather = father1.at(i-1);
            return -1;
        }
        i += 2;
    }

    // One is the direct predecessor of another
    int n = std::min(father1.size(), father2.size());
    commonfather = father1.at(n-2);
    return 0;
}

void newPathTakeOverPath(int Rindex, int oldNindex, int oldCindex, 
    int Nindex, int Cindex, double hit_x, double hit_y)
{
    if(TheRobot(Rindex)->N_.at(oldNindex).C_.at(oldCindex).status_.compare("open") != 0
        && TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).status_.compare("open") != 0)
        return ;
    
    if(Nindex == oldNindex)
		// Different edges of a same node should not be intersected
        return ;

	int oldloc = std::find_if(TheRobot(Rindex)->N_.at(oldNindex).C_.at(oldCindex).myPath_.data.begin(), 
	    TheRobot(Rindex)->N_.at(oldNindex).C_.at(oldCindex).myPath_.data.end(),
		[&](auto& a) {return fabs(a.first - hit_x) < 0.1 && fabs(a.second - hit_y) < 0.1; })
	    - TheRobot(Rindex)->N_.at(oldNindex).C_.at(oldCindex).myPath_.data.begin();

    //auto oldloc = std::find(TheRobot(Rindex)->N_.at(oldNindex).C_.at(oldCindex).myPath_.data.begin(), 
    //    TheRobot(Rindex)->N_.at(oldNindex).C_.at(oldCindex).myPath_.data.end(),
    //    std::pair<int, int>(hit_x, hit_y)) 
    //    - TheRobot(Rindex)->N_.at(oldNindex).C_.at(oldCindex).myPath_.data.begin();

	int loc = std::find_if(TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).myPath_.data.begin(),
	    TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).myPath_.data.end(),
		[&](auto& a) {return fabs(a.first - hit_x) < 0.1 && fabs(a.second - hit_y) < 0.1; })
	    - TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).myPath_.data.begin();


    //auto loc = std::find(TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).myPath_.data.begin(),
    //    TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).myPath_.data.end(),
    //    std::pair<int, int>(hit_x, hit_y)) 
    //    - TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).myPath_.data.begin();


    double oldcost = TheRobot(0)->N_.at(oldNindex).Gcost_ + 
        pathLength(TheRobot(0)->N_.at(oldNindex).C_.at(oldCindex).myPath_.data, oldloc);
    double cost = TheRobot(0)->N_.at(Nindex).Gcost_ + 
        pathLength(TheRobot(0)->N_.at(Nindex).C_.at(Cindex).myPath_.data, loc);

    if(oldcost < cost)
    {
		PartialOrder temp(oldNindex, oldCindex, Nindex, Cindex);
		TheRobot(0)->insertPartialOrder(temp);
    }
    else if(oldcost > cost)
    {
		PartialOrder temp(Nindex, Cindex, oldNindex, oldCindex);
		TheRobot(0)->insertPartialOrder(temp);
    }
    else
    {
        // We should check what to do when two paths are exactly of the same length
    }
    
}

void newPathTakeOverSweeper(int Rindex, int gapNindex, int gapCindex, 
    int pathNindex, int pathCindex, double hit_x, double hit_y, int loc)
{
	// After a new locally shortest path is found, we check whether the path intersects some Cindex gaps. 
	// If so, we compare the cost of the path going to the endpoints of the Cindex gap (we use direct concatenation)

    auto& gap = TheRobot(Rindex)->N_.at(gapNindex).C_.at(gapCindex);
    auto& path = TheRobot(Rindex)->N_.at(pathNindex).C_.at(pathCindex);


    if(gap.status_.compare("open") != 0 && path.status_.compare("open") != 0)
    {
        return ;
    }

    if(gapNindex == pathNindex)
        return ;// It is almost impossible

    if(TheRobot(Rindex)->N_.at(pathNindex).fatherindex_ == gapNindex )
        return ;
    
	// 2021.7.14 YT: TODO: the parent node is better than the gap sweeper does
	// not mean any of its son must be also greater than the gap sweeper.So
	// there should be a partial order relation, but should contribute nothing
	// to path simplification(maybe used in the future)
    //double pathReplaceCost = TheRobot(Rindex)->N_.at(pathNindex).Gcost_ + 
    //    pathLength(TheRobot(Rindex)->N_.at(pathNindex).C_.at(pathCindex).myPath_, loc)
    //    + hypot(
    //        TheRobot(Rindex)->N_.at(gapNindex).C_.at(gapCindex).cki_.first - hit_x, 
    //        TheRobot(Rindex)->N_.at(gapNindex).C_.at(gapCindex).cki_.second - hit_y
    //    );

    //if(pathReplaceCost < TheRobot(Rindex)->N_.at(gapNindex).C_.at(gapCindex).ckiGcost_)
    //{
    //    // std::cout << "pathtakeoversweeper: the gap loses optimality" << std::endl;
    //    TheRobot(Rindex)->N_.at(gapNindex).C_.at(gapCindex).removeOptimality();
    //    return ;
    //}

	std::vector<std::pair<int, int> > pathtemp;
	classicBresenham(TheRobot(Rindex)->N_.at(gapNindex).C_.at(gapCindex).cki_.first,
		TheRobot(Rindex)->N_.at(gapNindex).C_.at(gapCindex).cki_.second,
		hit_x, hit_y, pathtemp);

	std::vector<std::pair<double, double> >P;
	P.reserve(pathtemp.size());
	for (auto iter = pathtemp.begin(); iter != pathtemp.end(); ++iter)
	{
		P.push_back(std::pair<double, double>(iter->first+0.5, iter->second+0.5));
	}

	double gapReplaceCost = TheRobot(Rindex)->N_.at(gapNindex).C_.at(gapCindex).ckiGcost_ +
		pathLength(TheRobot(Rindex)->N_.at(pathNindex).C_.at(pathCindex).myPath_.data, loc) +
		pathLength(P);

    if(TheRobot(0)->N_.at(pathNindex).Gcost_ > gapReplaceCost)
    {
		PartialOrder temp(gapNindex, gapCindex, pathNindex, pathCindex);
		TheRobot(0)->insertPartialOrder(temp);
        return ;
    }
    
}

void newSweeperTakeOverSweeper(int Rindex, int oldNindex, int oldCindex, 
    int Nindex, int Cindex, double hit_x, double hit_y)
{
    if(TheRobot(0)->N_.at(oldNindex).C_.at(oldCindex).status_.compare("open") != 0
        && TheRobot(0)->N_.at(Nindex).C_.at(Cindex).status_.compare("open") != 0)
        return ;

	// Due to some computational error, the gap sweeper may intersect with its parent which is theoretically impossible. 
	// So we remove this case
     if(oldNindex == TheRobot(0)->N_.at(Nindex).fatherindex_ && 
         oldCindex == TheRobot(0)->N_.at(Nindex).soninfatherindex_)
    {
		std::cout << "We should not merge this case." << std::endl;
        return ;
    }

    if(oldNindex == Nindex)
    {
		std::cout << "We should not merge two gap sweepers belonging to the same node. This is left for future work." << std::endl;
        return ;
    }

    // For two gaps with different types
    // If there is no goal in the surrounded area, one of them may lose optimality

    bool b = TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).isLeftOrRight();
    int commonfather;
    int c = Bro(Rindex, oldNindex, oldCindex, Nindex, Cindex, commonfather);

	Path2D P1temp, P2temp;
    TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).tracePath(commonfather, P1temp);
    TheRobot(Rindex)->N_.at(oldNindex).C_.at(oldCindex).tracePath(commonfather, P2temp);

	std::vector<std::pair<double, double> > P1 = P1temp.data;
	std::vector<std::pair<double, double> > P2 = P2temp.data;


    double oldd = hypot(TheRobot(Rindex)->N_.at(oldNindex).C_.at(oldCindex).cki_.first - hit_x, 
        TheRobot(Rindex)->N_.at(oldNindex).C_.at(oldCindex).cki_.second - hit_y);

    double newd = hypot(TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).cki_.first - hit_x, 
        TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).cki_.second - hit_y);

    std::vector<std::pair<int, int> > bd;
    bd.reserve(2 + P1.size() + P2.size());
    bd.push_back(std::pair<int, int>(hit_x, hit_y));
    std::reverse(P1.begin(), P1.end());
    bd.insert(bd.end(), P1.begin(), P1.end());
    bd.insert(bd.end(), P2.begin(), P2.end());
    bd.push_back(std::pair<int, int>(hit_x, hit_y));

	std::vector<std::pair<double, double> > bd_temp;
	bd_temp.reserve(bd.size());
	for (auto iter = bd.begin(); iter != bd.end(); ++iter)
	{
		bd_temp.push_back(std::pair<double, double>(iter->first + 0.5, iter->second + 0.5));
	}

    bool isGoalFreebd = isGoalFree(bd_temp);
    
	//// 2022.5.5 YT: we comment this
 //   std::vector<int> fatherlink1, fatherlink2;
 //   int sonR, sonN, sonC;
	//int theOtherR, theOtherN, theOtherC;
 //   TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).getPredecessors(fatherlink1);
 //   TheRobot(Rindex)->N_.at(oldNindex).C_.at(oldCindex).getPredecessors(fatherlink2);
 //   if(fatherlink1.size() > fatherlink2.size())
 //   {
 //       sonR = Rindex;
 //       sonN = Nindex;
 //       sonC = Cindex;
	//	theOtherR = Rindex;
	//	theOtherN = oldNindex;
	//	theOtherC = oldCindex;
 //   }
 //   else if(fatherlink1.size() < fatherlink2.size())
 //   {
 //       sonR = Rindex;
 //       sonN = oldNindex;
 //       sonC = oldCindex;
	//	theOtherR = Rindex;
	//	theOtherN = Nindex;
	//	theOtherC = Cindex;
 //   }
 //   else
 //   {
 //       if(c == 0)
 //       {
 //           // Only if we think one is the direct son of another, but they have fatherlink of same length
 //           // we will report bug
 //           std::cout << "ERROR: we should not reach here. Show father 1: " << std::endl; 
 //           for(unsigned int i = 0; i < fatherlink1.size(); ++i)
 //           {
 //               std::cout << fatherlink1.at(i) << ", ";
 //           }
 //           std::cout << "father 2: ";
 //           for(unsigned int i = 0; i < fatherlink2.size(); ++i)
 //           {
 //               std::cout << fatherlink2.at(i) << ", ";
 //           }
 //           std::cout << std::endl;
 //       }
 //   }


    // We filter the old gap so that it must have different gaptype as the newly-found one
    if(TheRobot(Rindex)->N_.at(oldNindex).C_.at(oldCindex).isLeftOrRight() != b)
    {
        
        if(c == 0)
        {
			int sonR, sonN, sonC;
		    int theOtherR, theOtherN, theOtherC;

            // A node is the direct predecessor of another one. We figure out them
            // We want to know which one is the predecessor
		    std::vector<int> fatherlink1, fatherlink2;
		    TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).getPredecessors(fatherlink1);
			TheRobot(Rindex)->N_.at(oldNindex).C_.at(oldCindex).getPredecessors(fatherlink2);
			if(fatherlink1.size() > fatherlink2.size())
			{
				sonR = Rindex;
				sonN = Nindex;
				sonC = Cindex;
				theOtherR = Rindex;
				theOtherN = oldNindex;
				theOtherC = oldCindex;
			}
			else if(fatherlink1.size() < fatherlink2.size())
			{
				sonR = Rindex;
				sonN = oldNindex;
				sonC = oldCindex;
				theOtherR = Rindex;
				theOtherN = Nindex;
				theOtherC = Cindex;
			}
			else
			{
				std::cout << "ERROR: we should not reach here." << std::endl;
			}

            if(TheRobot(sonR)->N_.at(sonN).C_.at(sonC).status_.compare("no_usage") != 0 && isGoalFreebd)
            {
                // The son will be removed
				PartialOrder temp(theOtherN, theOtherC, sonN, sonC);
				TheRobot(0)->insertPartialOrder(temp);
				return;
            }

        }
        else if((b && c == 1)||(!b && c == -1))
        {
            // we get the contour of the closed region
            if(isGoalFreebd)
            {
                double oldCost = TheRobot(0)->N_.at(oldNindex).C_.at(oldCindex).ckiGcost_ + oldd;
                double newCost = TheRobot(0)->N_.at(Nindex).C_.at(Cindex).ckiGcost_ + newd;
                if(newCost < oldCost + 1)
                {
					PartialOrder temp(Nindex, Cindex, oldNindex, oldCindex);
					TheRobot(0)->insertPartialOrder(temp);
                }
                else if(oldCost < newCost + 1)
                {
					PartialOrder temp(oldNindex, oldCindex, Nindex, Cindex);
					TheRobot(0)->insertPartialOrder(temp);
                }
                return ;
            }
        }
        
    }
    else
    {
        // For two gaps with same type, 
        // If there is no goal in the surrounded area, then the gap with higher cost 
        // to the intersection can be safely removed
        if(c == 0)
        {			
			int sonR, sonN, sonC;
			int theOtherR, theOtherN, theOtherC;

			// A node is the direct predecessor of another one. We figure out them
			// We want to know which one is the predecessor
			std::vector<int> fatherlink1, fatherlink2;
			TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).getPredecessors(fatherlink1);
			TheRobot(Rindex)->N_.at(oldNindex).C_.at(oldCindex).getPredecessors(fatherlink2);
			if (fatherlink1.size() > fatherlink2.size())
			{
				sonR = Rindex;
				sonN = Nindex;
				sonC = Cindex;
				theOtherR = Rindex;
				theOtherN = oldNindex;
				theOtherC = oldCindex;
			}
			else if (fatherlink1.size() < fatherlink2.size())
			{
				sonR = Rindex;
				sonN = oldNindex;
				sonC = oldCindex;
				theOtherR = Rindex;
				theOtherN = Nindex;
				theOtherC = Cindex;
			}
			else
			{
				std::cout << "ERROR: we should not reach here." << std::endl;
			}

            if(isGoalFreebd)
            {
                // We need not compare the movement cost, because the predecessor node is a straight path
				PartialOrder temp(theOtherN, theOtherC, sonN, sonC);
				TheRobot(0)->insertPartialOrder(temp);
                return ;
            }
        }
        else
        {
            // The order of nodes: The more "counter clockwise" the node index is, 
            // the bigger index it is
            if(b)
            {
                // Both two gaps are right gap. We should find the younger bro, 
                // verify that the younger bro has less cost to the intersection, 
                // and verify that there is no goal within the surrounded area. 
                // Then the elder bro can be safely removed. 
                if(c == 1 && isGoalFreebd &&
                    TheRobot(Rindex)->N_.at(oldNindex).C_.at(oldCindex).ckiGcost_ + oldd < 
                        TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).ckiGcost_ + newd)
                {
					PartialOrder temp(oldNindex, oldCindex, Nindex, Cindex);
					TheRobot(0)->insertPartialOrder(temp);
                    return ;
                }
                else if(c == -1 && isGoalFreebd &&
                    TheRobot(Rindex)->N_.at(oldNindex).C_.at(oldCindex).ckiGcost_ + oldd > 
                    TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).ckiGcost_ + newd)
                {
					PartialOrder temp(Nindex, Cindex, oldNindex, oldCindex);
					TheRobot(0)->insertPartialOrder(temp);
                    return ;
                }   
            }
            else // if !b
            {
				// Both two gaps are right gap. We should find the younger bro, 
				// verify that the younger bro has less cost to the intersection, 
				// and verify that there is no goal within the surrounded area. 
				// Then the elder bro can be safely removed. 
                if(c == 1 && isGoalFreebd &&
                    TheRobot(Rindex)->N_.at(oldNindex).C_.at(oldCindex).ckiGcost_ + oldd > 
                        TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).ckiGcost_ + newd)
                {
					PartialOrder temp(Nindex, Cindex, oldNindex, oldCindex);
					TheRobot(0)->insertPartialOrder(temp);
                    return ;
                }
                else if(c == -1 && isGoalFreebd &&
                    TheRobot(Rindex)->N_.at(oldNindex).C_.at(oldCindex).ckiGcost_ + oldd < 
                    TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).ckiGcost_ + newd)
                {
					PartialOrder temp(oldNindex, oldCindex, Nindex, Cindex);
					TheRobot(0)->insertPartialOrder(temp);
                    return ;
                }   
            }
            
        }
        
    }

    // We check the intersected gap sweepers
    if(TheRobot(Rindex)->N_.at(oldNindex).C_.at(oldCindex).ckiGcost_ > 
        TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).ckiGcost_ + oldd + newd)
    {
		PartialOrder temp(Nindex, Cindex, oldNindex, oldCindex);
		TheRobot(0)->insertPartialOrder(temp);
    }
    else if(TheRobot(Rindex)->N_.at(Nindex).C_.at(Cindex).ckiGcost_ > 
            TheRobot(Rindex)->N_.at(oldNindex).C_.at(oldCindex).ckiGcost_ + oldd + newd)
    {
		PartialOrder temp(oldNindex, oldCindex, Nindex, Cindex);
		TheRobot(0)->insertPartialOrder(temp);
    }
}
