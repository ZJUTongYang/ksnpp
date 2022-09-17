#include "Kim_alg.h"
#include "Atom.h"
#include <unordered_set>
#include <ctime>

namespace Kim2013 {

template <typename T> int sgn(T val) 
{
    return (T(0) < val) - (val < T(0));
}

Kim2013::Kim2013()
{

    Storage_.clear();
    Queue_.clear();

	I_ = new Indices*[MAX_X_SIZE];
	for (unsigned int i = 0; i < MAX_X_SIZE; ++i)
	{
		I_[i] = new Indices[MAX_Y_SIZE];
		for (unsigned int j = 0; j < MAX_Y_SIZE; ++j)
		{
			I_[i][j].clear();
		}
	}

    g_obs_.clear();

    std::cout << "Finish Kim2013 planner" << std::endl;
}

std::vector<int> Kim2013::calSwing(const std::vector<int>& oldH, int oldx, int oldy, int newx, int newy)
{
    std::vector<int> newH;
    newH = oldH;
    for(int i = 0; i < g_obs_.size(); ++i)
    {
        if(oldy >= g_obs_[i].second + 0.5 && newy >= g_obs_[i].second + 0.5)
        {
            
            if (oldx > g_obs_[i].first + 0.5 && newx < g_obs_[i].first + 0.5)
            {
                if(!oldH.empty() && oldH.back() == i)
                    newH.pop_back();
                else
                    newH.push_back(-i);

                return newH;
            }
            else if(oldx < g_obs_[i].first + 0.5 && newx > g_obs_[i].first + 0.5)
            {
                if (!oldH.empty() && oldH.back() == -i)
                    newH.pop_back();
                else
                    newH.push_back(i);

                return newH;
            }
        }
    }
	return newH;
}
//
//bool Kim2013::C2LPlanning(const KimNode& snode, const std::pair<int, int>& goalLocation, std::vector<std::pair<int, int> >& result_path, KimNode& goalNode)
//{
//
//    // std::cout << "test 1" << std::endl;
//    int sizex = MAX_X_SIZE;
//    int sizey = MAX_Y_SIZE;
//
//    KimNode startnode;
//    startnode.index_ = 0;
//    startnode.x_ = snode.x_;
//    startnode.y_ = snode.y_;
//    startnode.g_ = 0;
//    startnode.f_ = sqrt( (snode.x_ - goalLocation.first)*(snode.x_ - goalLocation.first) + (snode.y_ - goalLocation.second)*(snode.y_ - goalLocation.second) ); // This is meaningless when precalculating the whole space
//    startnode.hindex_ = snode.hindex_;
//    startnode.fatherindex_ = -1;
//
//    // std::cout << "check start location: (" << startnode.x_ << ", " << startnode.y_ << ")" << std::endl;
//
//    // std::cout << "Check goal location: (" << goalLocation.first << ", " << goalLocation.second << ")" << std::endl;
//
//    std::vector<KimNode> private_Storage; // The buffer for only this C2LPlanning
//    private_Storage.clear();
//    private_Storage.reserve(sizex*sizey);
//
//    private_Storage.push_back(startnode);
//
//    //std::vector<std::vector<Indices> > private_I;
//	//Indices private_I[MAX_X_SIZE][MAX_Y_SIZE];
//	Indices** private_I;
//	private_I = new Indices*[MAX_X_SIZE];
//	for (unsigned int i = 0; i < MAX_X_SIZE; ++i)
//	{
//		private_I[i] = new Indices[MAX_Y_SIZE];
//	}
//
//    private_I[startnode.x_][startnode.y_].push_back(startnode.index_);
//
//    std::vector<KimNode> private_Queue;
//    private_Queue.clear();
//    private_Queue.push_back(startnode);
//
//    while(1)
//    {
//        if(private_Queue.empty())
//        {
//            std::cout << "The private_queue is empty" << std::endl;
//            break;
//        }
//
//        // std::cout << "private_Storage[0] = (" << private_Storage[0].x_ << ", " << private_Storage[0].y_ << ")" << std::endl;
//
//        // we choose the best element
//        auto newnode = private_Queue[0]; 
//
//        // std::cout << "check new node: (" << newnode.x_ << ", " << newnode.y_ << "), newnode.index_: " <<  newnode.index_ 
//        //           << ", its parent: (" << private_Storage[newnode.fatherindex_].x_ << ", " << private_Storage[newnode.fatherindex_].y_ << ")"
//        //           << std::endl;
//
//        std::pop_heap(private_Queue.begin(), private_Queue.end(), 
//            [](const KimNode& a, const KimNode& b){
//                return a.f_ > b.f_;
//            });
//
//        private_Queue.pop_back();
//        private_Storage[newnode.index_].open_ = false;
//
//
//        if(newnode.x_ == goalLocation.first && newnode.y_ == goalLocation.second)
//        {
//
//            std::cout << "size of buffer: " << private_Storage.size() << std::endl;
//            std::cout << "YT: we have found one path.\n" << std::endl;
//            std::vector<std::pair<int, int> > result_temp;
//            tracePath(newnode, result_temp, private_Storage, startnode);
//
//            // check self-crossing
//            // if(isSelfCrossing(result_temp))
//            // {
//            //     continue;
//            // }
//
//            result_path.assign(result_temp.begin(), result_temp.end());
//            goalNode = newnode;
//
//            break;
//        }
//
//        std::vector<std::pair<int, int> > son;
//        newnode.sons(son);
//
//        for(unsigned int j = 0; j < son.size(); ++j)
//        {
//            auto son_x = son[j].first;
//            auto son_y = son[j].second;
//
//            if(son_x < 0 || son_x >= sizex || son_y < 0 || son_y >= sizey)
//                continue;
//
//            if(costmap_.data[son_x + son_y*sizex] > 0)
//                continue;
//
//            std::vector<int> newHindex = calSwing(newnode.hindex_, newnode.x_, newnode.y_, son_x, son_y);
//
//            // We omit looping paths
//            if(newHindex.size() >= 2 && newHindex[newHindex.size()-1] == newHindex[newHindex.size()-2])
//            {
//                continue;
//            }
//
//            // If the node is not cable-length-admissible, we also remove it
//            // Here we use the global buffer
//            if(!isEqualHC(newHindex, Storage_, I_, son_x, son_y))
//            {
//                continue;
//            }
//
//            double newg = newnode.g_ + sqrt( (newnode.x_ - son_x)*(newnode.x_ - son_x) + (newnode.y_ - son_y)*(newnode.y_ - son_y) );
//
//
//            // if(newg > max_cable_length_)
//            //     continue;
//
//            double newf = newg + sqrt( (goalLocation.first - son_x)*(goalLocation.first - son_x) + (goalLocation.second - son_y)*(goalLocation.second - son_y) );
//
//            int newsonindex = private_Storage.size();
//
//            if(private_I[son_x][son_y].empty() || !isEqualHC(newHindex, private_Storage, private_I, son_x, son_y))
//            {
//                // std::cout << "no before: check son and son_g: (" << son_x << ", " << son_y << ", " << newg << ")" << std::endl;
//
//
//                // If the node has not been visited
//                // Or, we find non-homotopic paths to the son location, we preserve them both. I.e., we create a new node
//                KimNode newson(son_x, son_y, newg, newf, newHindex, newsonindex, newnode.index_);
//                private_Queue.push_back(newson);
//                std::push_heap(private_Queue.begin(), private_Queue.end(),
//                    [](const KimNode& a, const KimNode& b){
//                    return a.f_ > b.f_;
//                    });
//
//                private_Storage.push_back(newson);
//                private_I[son_x][son_y].push_back(newsonindex);
//            }
//            else
//            {
//                bool B = isEqualHC(newHindex, private_Storage, private_I, son_x, son_y);
//                // std::cout << "check B: " << B << std::endl;
//                // We can find the equivalent Hindex in the I matrix, but we may carry out a shortcut
//                int oldindex = findEqualHCNode(newHindex, private_Storage, private_I, son_x, son_y);
//                // std::cout << "check oldindex: " << oldindex << std::endl;
//
//                if(private_Storage[oldindex].g_ <= newg)
//                {
//                    continue;
//                }
//
//                // std::cout << "overwrite: check son and son_g: (" << son_x << ", " << son_y << ", " << newg << ")" << std::endl;
//
//
//                // std::cout << "Here we will overwrite somenode (" << son_x << ", " << son_y << ") < (" << private_Storage[oldindex].x_ << ", " << private_Storage[oldindex].y_ << ")" << std::endl;
//
//                private_Storage[oldindex].g_ = newg;
//                private_Storage[oldindex].f_ = newf;
//                private_Storage[oldindex].hindex_ = newHindex; // In fact, it doesn't change
//                private_Storage[oldindex].fatherindex_ = newnode.index_;
//
//                // If the old node is open, then it is in the queue. We only change its cost and parent
//                if(private_Storage[oldindex].open_)
//                {
//                    // Find the position of the son in the queue, and update it
//                    auto iter = std::find_if(private_Queue.begin(), private_Queue.end(), 
//                        [&](const KimNode& a){return a.index_ == oldindex;});
//                    *iter = private_Storage[oldindex];
//                    std::make_heap(private_Queue.begin(), private_Queue.end(),
//                        [](const KimNode& a, const KimNode& b){
//                        return a.f_ > b.f_;
//                        });
//                }
//                else
//                {
//                    // If the old node is closed, we may re-open it and push it into the queue
//                    private_Storage[oldindex].open_ = true;
//                    private_Queue.push_back(private_Storage[oldindex]);
//                    std::push_heap(private_Queue.begin(), private_Queue.end(),
//                        [](const KimNode& a, const KimNode& b){
//                        return a.f_ > b.f_;
//                        });
//                }
//            }
//        }
//
//        // std::cout << std::endl;
//    }
//
//	return true;
//}


int Kim2013::findEqualHCNode(const std::vector<int>& newH, const std::vector<KimNode>& theStorage, Indices** theI, int son_x, int son_y)
{
    if(theI[son_x][son_y].empty())
    {
        return -1;
    }

    

    for(auto iter = theI[son_x][son_y].begin(); iter != theI[son_x][son_y].end(); ++iter)
    {
//        std::vector<int> oldH = theStorage[*iter].hindex_;

        if(theStorage[*iter].hindex_.empty() && newH.empty())
        {
            return *iter;
        }


        if(theStorage[*iter].hindex_.size() == newH.size())
        {
            bool thesame = true;
            for(unsigned int i = 0; i < theStorage[*iter].hindex_.size(); ++i)
            {
                if(theStorage[*iter].hindex_[i] != newH[i])
                {
                    thesame = false;
                    break;
                }
            }
            if(thesame)
            {
                return *iter;
            }
        }
    }

    std::cout << "We check the info of newH: " << std::endl;
    for(auto iter = newH.begin(); iter != newH.end(); ++iter)
    {
        std::cout << *iter << ", ";
    }
    std::cout << std::endl;

    std::cout << "We check all existing indices: " << std::endl;
    for(auto iter = theI[son_x][son_y].begin(); iter != theI[son_x][son_y].end(); ++iter)
    {
        std::vector<int> oldH = theStorage[*iter].hindex_;
        std::cout << "[" ;
        for(unsigned int i = 0; i < oldH.size(); ++i)
        {
            std::cout << oldH[i] << ", ";
        }
        std::cout << "]" << std::endl;

    }

    std::cout << "findEqualHCNode: error, we should not reach here." << std::endl;
    return -1;
}

bool Kim2013::isEqualHC(const std::vector<int>& newH, const std::vector<KimNode>& theStorage, Indices** theI, int son_x, int son_y)
{
    if(theI[son_x][son_y].empty())
    {
        return false;
    }

    for(auto iter = theI[son_x][son_y].begin(); iter != theI[son_x][son_y].end(); ++iter)
    {
//        std::vector<int> oldH = theStorage[*iter].hindex_;
        if(theStorage[*iter].hindex_.empty() && newH.empty())
            return true;
            
        if(theStorage[*iter].hindex_.size() == newH.size())
        {
            bool thesame = true;
            for(unsigned int i = 0; i < theStorage[*iter].hindex_.size(); ++i)
            {
                if(theStorage[*iter].hindex_[i] != newH[i])
                {
                    thesame = false;
                    break;
                }
            }
            if(thesame)
            {
                return true;
            }
        }
    }
    return false;
}

bool Kim2013::isSelfCrossing(const std::vector<std::pair<int, int> >& P)
{
    const int tmpx = 10000;
    std::unordered_set<int> S;
    for(auto iter = P.begin(); iter != P.end(); ++iter)
    {
        S.insert(iter->second*tmpx + iter->first);
    }
    if(S.size() < P.size())
        return true;

    return false;
}

void Kim2013::sedFill(std::vector<std::pair<int, int> >& ObsPosition)
{
    ObsPosition.clear();
    // int nx = costmap_->getSizeInCellsX();
    // int ny = costmap_->getSizeInCellsY();
    int nx = MAX_X_SIZE;
    int ny = MAX_Y_SIZE;

    // We first copy the map cost value to another 2D array
    int** mask;
    mask = new int*[nx];
    for(unsigned int i = 0;i < nx; ++i)
    {
        mask[i] = new int[ny];
    }
    // Copy value
    for(unsigned int i = 0; i < nx; ++i)
    {
        for(unsigned int j = 0; j < ny; ++j)
        {
            mask[i][j] = costmap_.data[i+j*MAX_X_SIZE]== 254? 254: 0;// In matlab, we use the original map, so here we do the same as in matlab
        }
    }
	for (unsigned int i = 0; i < MAX_X_SIZE; ++i)
	{
		mask[i][0] = 0;
		mask[i][MAX_Y_SIZE - 1] = 0;
	}
	for (unsigned int j = 0; j < MAX_Y_SIZE; ++j)
	{
		mask[0][j] = 0;
		mask[MAX_X_SIZE - 1][j] = 0;
	}

    // Clear the boundary of the costmap
    // YT: it seems that we needn't remove the outer contour of the map. Just see them as the same obstacle
    for(unsigned int i = 1; i < nx -1; ++i)
    {
        for(unsigned int j = 1; j < ny - 1; ++j)
        {
            if(mask[i][j] == 0)
            {
				// free space 
                continue;
            }

            std::pair<int, int> seed(i, j);
            // std::cout << "seed: (" << i << ", " << j << "): " << (int)mask[i][j] << std::endl;

            ObsPosition.push_back(seed);
            // We start a sedFill to eliminate all connected obstacles;
            std::vector<std::pair<int, int> > s;
            s.push_back(seed);
            while(!s.empty())
            {
                auto cur = s.back();
                if(INMAP(cur.first, cur.second) && mask[cur.first][cur.second] == 254)
                {
                    mask[cur.first][cur.second] = 0;
                    // std::cout << "check adjs: (" 
                    //     << (int)mask[cur.first-1][cur.second] << "), ("
                    //     << (int)mask[cur.first+1][cur.second] << "), ("
                    //     << (int)mask[cur.first][cur.second-1] << "), ("
                    //     << (int)mask[cur.first][cur.second+1] << ")" << std::endl;
                        
                    s.pop_back();
					s.push_back(std::pair<int, int>(cur.first - 1, cur.second-1));
					s.push_back(std::pair<int, int>(cur.first - 1, cur.second));
					s.push_back(std::pair<int, int>(cur.first-1, cur.second+1));
					s.push_back(std::pair<int, int>(cur.first + 1, cur.second-1));
					s.push_back(std::pair<int, int>(cur.first + 1, cur.second));
					s.push_back(std::pair<int, int>(cur.first+1, cur.second+1));
                    s.push_back(std::pair<int, int>(cur.first,   cur.second-1));
                    s.push_back(std::pair<int, int>(cur.first,   cur.second+1));
                }
                else
                {
                    s.pop_back();
                }
            }//while
        }
    }


    // delete mask
    for(unsigned int i = 0; i < nx; ++i)
        delete[] mask[i];

    delete[] mask;
}

void Kim2013::tracePath(const KimNode& goalnode, std::vector<std::pair<int, int> >& result_path, const std::vector<KimNode>& theStorage,
    const KimNode& startnode)
{
    result_path.clear();
    auto cur = goalnode;
    int COUNT = theStorage.size();
    int count = 0;
    while(count < COUNT)
    {
            
        result_path.push_back(std::pair<int, int>(cur.x_, cur.y_));
        // int father = cur.fatherindex_;
        // if(father == -1)
        if(cur.x_ == startnode.x_ && cur.y_ == startnode.y_ && cur.hindex_ == startnode.hindex_)
        {
            // std::cout << "check father: " << cur.fatherindex_ << std::endl;
            break;
        }

        // int father_x = cur.fatherindex_ % size_x;
        // int father_y = (cur.fatherindex_ - father_x) / size_x;

        // std::cout << "check cur: (x, y, father) = " << cur.x_ << ", " << cur.y_ 
        //     << ", " << cur.fatherindex_ << ", (" << father_x << ", " << father_y << ")" << std::endl;

        // std::cout << "[" << cur.x_ << ", " << cur.y_ << ", " << cur.fatherindex_ << "], ";
        // std::cout << "(we check the next one: theStorage[cur.fatherindex_] = [" << theStorage[cur.fatherindex_].x_ << ", " << theStorage[cur.fatherindex_].y_ << "]), ";
        cur = theStorage[cur.fatherindex_];
        count++;

    }
    // result_path.push_back(std::pair<int, int>(cur.x_, cur.y_));

    // We cout the result path
    //std::cout << "We cout the result path" << std::endl;
    //for(auto iter = result_path.begin(); iter != result_path.end(); ++iter)
    //    std::cout << "[" << iter->first << ", " << iter->second << "], ";
    //std::cout << std::endl;

    //std::reverse(result_path.begin(), result_path.end());
    //std::cout << "size of new local path: " << result_path.size() << std::endl;
}

void Kim2013::new_optimal_tp(const Path2D& ref_path, std::pair<double, double> goal_location, Path2D& result_path)
{
	std::cout << "size of ref_path: " << ref_path.data.size() << std::endl;
	// We firstly construct the initial configurations
	for (auto iter = Storage_.begin(); iter != Storage_.end(); ++iter)
	{
		iter->g_ = 10000;
		iter->open_ = false;
	}

	clock_t startTime = clock();


	double COUNT = 0;


	int x = floor(ref_path.data[0].first);
	int y = floor(ref_path.data[0].second);
	int new_x, new_y;
	std::vector<int> newHindex;
	for (auto iter = ref_path.data.begin(); iter != ref_path.data.end(); ++iter)
	{
		new_x = floor(iter->first);
		new_y = floor(iter->second);
		newHindex = calSwing(newHindex, x, y, new_x, new_y);
		x = new_x;
		y = new_y;
	}

	int start_x = floor(ref_path.data.back().first);
	int start_y = floor(ref_path.data.back().second);
	int startnodeindex = findEqualHCNode(newHindex, Storage_, I_, start_x, start_y);

	std::cout << "check startnode: [" << Storage_[startnodeindex].x_ << ", " << Storage_[startnodeindex].y_ << "]" << std::endl;

	Storage_[startnodeindex].g_ = 0;

	Queue_.clear();
	Queue_.push_back(Storage_[startnodeindex]);

	while (1)
	{
		if (Queue_.empty())
		{
			break;
		}

		// we choose the best element
		KimNode newnode = Queue_[0];
		std::pop_heap(Queue_.begin(), Queue_.end(),
			[](const KimNode& a, const KimNode& b) {
			return a.f_ > b.f_;
		});



		Queue_.pop_back();

		Storage_[newnode.index_].open_ = false;

		if (newnode.x_ == floor(goal_location.first) && newnode.y_ == floor(goal_location.second))
		{
			std::cout << "YT: we have found one path.\n" << std::endl;
			std::vector<std::pair<int, int> > result_temp;
			tracePath(newnode, result_temp, Storage_, Storage_[startnodeindex]);


			for (auto iter = result_temp.begin(); iter != result_temp.end(); ++iter)
			{
				result_path.data.push_back(std::pair<double, double>(iter->first + 0.5, iter->second + 0.5));
			}

			//std::cout << "We check optimal TP solution: length: " << pathLength(result_path.data) << std::endl;
			//result_path.assign(result_temp.begin(), result_temp.end());
			//goalNode = newnode;

			break;
		}



		std::vector<int> son = newnode.adj_kimnode_;

		//std::vector<std::pair<int, int> > son;
		//newnode.sons(son);

		//std::cout << "size of sons: " << newnode.adj_kimnode_.size() << std::endl;

		//logfile << "newnode: [" << newnode.x_ << ", " << newnode.y_ << "], hindex_ = ";
		//for (unsigned int i = 0; i < son.size(); ++i)
		//{
		//	logfile << "-->["<< Storage_[son[i]].x_ << ", " << Storage_[son[i]].y_ << "], ";
		//}
		//logfile << std::endl;

		for (unsigned int j = 0; j < son.size(); ++j)
		{

			//COUNT += 0.0000001;
			auto son_x = Storage_[son[j]].x_;
			auto son_y = Storage_[son[j]].y_;

			//std::cout << "Son: [" << son_x << ", " << son_y << "]" << std::endl;

			double newg = newnode.g_ + sqrt((newnode.x_ - son_x)*(newnode.x_ - son_x) + (newnode.y_ - son_y)*(newnode.y_ - son_y)) + COUNT;

			double newf = newg + sqrt((goal_location.first - son_x)*(goal_location.first - son_x) + (goal_location.second - son_y)*(goal_location.second - son_y));

			int newsonindex = son[j];

			int oldindex = newsonindex;
			if (Storage_[oldindex].g_ <= newg)
			{
				//std::cout << "The old node is shorter than the new node" << std::endl;
				continue;
			}

			Storage_[oldindex].g_ = newg;
			Storage_[oldindex].f_ = newf;
			Storage_[oldindex].fatherindex_ = newnode.index_;

			// If the old node is open, then it is in the queue. We only change its cost and parent
			if (Storage_[oldindex].open_)
			{
				//std::cout << "The old node is opened" << std::endl;
				// Find the position of the son in the queue, and update it
				auto iter = std::find_if(Queue_.begin(), Queue_.end(),
					[&](const KimNode& a) {return a.index_ == oldindex; });
				*iter = Storage_[oldindex];
				std::make_heap(Queue_.begin(), Queue_.end(),
					[](const KimNode& a, const KimNode& b) {
					return a.f_ > b.f_;
				});
			}
			else
			{
				// If the old node is closed, we may re-open it and push it into the queue
				Storage_[oldindex].open_ = true;
				Queue_.push_back(Storage_[oldindex]);
				std::push_heap(Queue_.begin(), Queue_.end(),
					[](const KimNode& a, const KimNode& b) {
					return a.f_ > b.f_;
				});
			}

		}

	}

	clock_t endTime = clock();

	std::cout << "(in functions) Time for Kim2013 optimal tp: " << (double)(endTime - startTime)*1000.0 / CLOCKS_PER_SEC << "ms" << std::endl;


//	std::cout << "YT: we have collected all valid waypoints" << std::endl;

}



void Kim2013::optimal_tp(const Path2D& ref_path, std::pair<double, double> goal_location, Path2D& result_path)
{
	// We firstly construct the initial configurations

	int x = floor(ref_path.data[0].first);
	int y = floor(ref_path.data[0].second);
	int new_x, new_y;
	std::vector<int> newHindex;
	for (auto iter = ref_path.data.begin(); iter != ref_path.data.end(); ++iter)
	{
		new_x = floor(iter->first);
		new_y = floor(iter->second);
		newHindex = calSwing(newHindex, x, y, new_x, new_y);
		x = new_x;
		y = new_y;
	}

	KimNode startnode;
	startnode.index_ = 0;
	startnode.x_ = ref_path.data.back().first;
	startnode.y_ = ref_path.data.back().second;
	startnode.g_ = 0;
	startnode.f_ = sqrt((startnode.x_ - goal_location.first)*(startnode.x_ - goal_location.first) + 
		(startnode.y_ - goal_location.second)*(startnode.y_ - goal_location.second)); // This is meaningless when precalculating the whole space
	startnode.hindex_ = newHindex;
	startnode.fatherindex_ = -1;

	std::vector<KimNode> private_Storage; // The buffer for only this C2LPlanning
	private_Storage.clear();
	private_Storage.reserve(MAX_X_SIZE*MAX_Y_SIZE);

	private_Storage.push_back(startnode);

	Indices** private_I;
	private_I = new Indices*[MAX_X_SIZE];
	for (unsigned int i = 0; i < MAX_X_SIZE; ++i)
	{
		private_I[i] = new Indices[MAX_Y_SIZE];
	}

	clock_t optimal_tp_startTime = clock();
	

	private_I[startnode.x_][startnode.y_].push_back(startnode.index_);

	std::vector<KimNode> private_Queue;
	private_Queue.clear();
	private_Queue.push_back(startnode);

	while (1)
	{
		if (private_Queue.empty())
		{
			std::cout << "The private_queue is empty" << std::endl;
			break;
		}

		// we choose the best element
		auto newnode = private_Queue[0];

		std::pop_heap(private_Queue.begin(), private_Queue.end(),
			[](const KimNode& a, const KimNode& b) {
			return a.f_ > b.f_;
		});

		private_Queue.pop_back();
		private_Storage[newnode.index_].open_ = false;


		if (newnode.x_ == goal_location.first && newnode.y_ == goal_location.second)
		{

			std::cout << "size of buffer: " << private_Storage.size() << std::endl;
			std::cout << "YT: we have found one path.\n" << std::endl;
			std::vector<std::pair<int, int> > result_temp;
			tracePath(newnode, result_temp, private_Storage, startnode);


			for (auto iter = result_temp.begin(); iter != result_temp.end(); ++iter)
			{
				result_path.data.push_back(std::pair<double, double>(iter->first + 0.5, iter->second + 0.5));
			}

			std::cout << "We check optimal TP solution: length: " << pathLength(result_path.data) << std::endl;
			//result_path.assign(result_temp.begin(), result_temp.end());
			//goalNode = newnode;

			break;
		}

		std::vector<std::pair<int, int> > son;
		newnode.sons(son);

		for (unsigned int j = 0; j < son.size(); ++j)
		{
			auto son_x = son[j].first;
			auto son_y = son[j].second;

			//if (son_x < 0 || son_x >= sizex || son_y < 0 || son_y >= sizey)
			//	continue;

			if (costmap_.data[son_x + son_y * MAX_X_SIZE] > 0)
				continue;

			std::vector<int> newHindex = calSwing(newnode.hindex_, newnode.x_, newnode.y_, son_x, son_y);

			//// We omit looping paths
			//if (newHindex.size() >= 2 && newHindex[newHindex.size() - 1] == newHindex[newHindex.size() - 2])
			//{
			//	continue;
			//}

			// If the node is not in the workspace, we also remove it
			// Here we use the global buffer
			if (!isEqualHC(newHindex, Storage_, I_, son_x, son_y))
			{
				continue;
			}

			double newg = newnode.g_ + sqrt((newnode.x_ - son_x)*(newnode.x_ - son_x) + (newnode.y_ - son_y)*(newnode.y_ - son_y));


			// if(newg > max_cable_length_)
			//     continue;

//			double newf = newg + sqrt((goal_location.first - son_x)*(goal_location.first - son_x) + (goal_location.second - son_y)*(goal_location.second - son_y));
			double newf = newg;

			int newsonindex = private_Storage.size();

			if (private_I[son_x][son_y].empty() || !isEqualHC(newHindex, private_Storage, private_I, son_x, son_y))
			{
				// std::cout << "no before: check son and son_g: (" << son_x << ", " << son_y << ", " << newg << ")" << std::endl;


				// If the node has not been visited
				// Or, we find non-homotopic paths to the son location, we preserve them both. I.e., we create a new node
				KimNode newson(son_x, son_y, newg, newf, newHindex, newsonindex, newnode.index_);
				private_Queue.push_back(newson);
				std::push_heap(private_Queue.begin(), private_Queue.end(),
					[](const KimNode& a, const KimNode& b) {
					return a.f_ > b.f_;
				});

				private_Storage.push_back(newson);
				private_I[son_x][son_y].push_back(newsonindex);
			}
			else
			{
				//bool B = isEqualHC(newHindex, private_Storage, private_I, son_x, son_y);
				// std::cout << "check B: " << B << std::endl;
				// We can find the equivalent Hindex in the I matrix, but we may carry out a shortcut
				int oldindex = findEqualHCNode(newHindex, private_Storage, private_I, son_x, son_y);
				// std::cout << "check oldindex: " << oldindex << std::endl;

				if (private_Storage[oldindex].g_ <= newg)
				{
					continue;
				}

				// std::cout << "overwrite: check son and son_g: (" << son_x << ", " << son_y << ", " << newg << ")" << std::endl;


				// std::cout << "Here we will overwrite somenode (" << son_x << ", " << son_y << ") < (" << private_Storage[oldindex].x_ << ", " << private_Storage[oldindex].y_ << ")" << std::endl;

				private_Storage[oldindex].g_ = newg;
				private_Storage[oldindex].f_ = newf;
				private_Storage[oldindex].hindex_ = newHindex; // In fact, it doesn't change
				private_Storage[oldindex].fatherindex_ = newnode.index_;

				// If the old node is open, then it is in the queue. We only change its cost and parent
				if (private_Storage[oldindex].open_)
				{
					// Find the position of the son in the queue, and update it
					auto iter = std::find_if(private_Queue.begin(), private_Queue.end(),
						[&](const KimNode& a) {return a.index_ == oldindex; });
					*iter = private_Storage[oldindex];
					std::make_heap(private_Queue.begin(), private_Queue.end(),
						[](const KimNode& a, const KimNode& b) {
						return a.f_ > b.f_;
					});
				}
				else
				{
					// If the old node is closed, we may re-open it and push it into the queue
					private_Storage[oldindex].open_ = true;
					private_Queue.push_back(private_Storage[oldindex]);
					std::push_heap(private_Queue.begin(), private_Queue.end(),
						[](const KimNode& a, const KimNode& b) {
						return a.f_ > b.f_;
					});
				}
			}
		}

		// std::cout << std::endl;
	}

	clock_t optimal_tp_endTime = clock();
	std::cout << "(In functions) Time for optimal TP: " << (double)(optimal_tp_endTime - optimal_tp_startTime)*1000.0 / CLOCKS_PER_SEC << "ms" << std::endl;


	for (unsigned int i = 0; i < MAX_X_SIZE; ++i)
	{
		delete[] private_I[i];
	}
	delete[] private_I;


}

void Kim2013::preCalculateAllConfigurations()
{
	// We firstly verify how many collision-free locations
	int collision_free_count = 0;
	for (unsigned int i = 0; i < MAX_X_SIZE; ++i)
	{
		for (unsigned int j = 0; j < MAX_Y_SIZE; ++j)
		{
			if (costmap_.data[i + j * MAX_X_SIZE] == 0)
			{
				collision_free_count++;
			}
		}
	}
	std::cout << "Number of collision-free point: " << collision_free_count << std::endl;


    // We firstly collect all obstacles
    std::cout << "We collect obstacles: " << std::endl;
    g_obs_.clear();
    sedFill(g_obs_);
    std::cout << "we check size of obstacles: " << g_obs_.size() << std::endl;

	std::cout << "We check g_obs: " << std::endl;
	for (unsigned int i = 0; i < g_obs_.size(); ++i)
	{
		std::cout << "[" << g_obs_[i].first << ", " << g_obs_[i].second << "]" << std::endl;
	}

    int sizex = MAX_X_SIZE;
    int sizey = MAX_Y_SIZE;

	clock_t startTime = clock();


	double COUNT = 0;

    KimNode startnode;
    startnode.index_ = 0;
    startnode.x_ = start_point_.first;
    startnode.y_ = start_point_.second;
    startnode.g_ = 0;
    startnode.f_ = -1; // This is meaningless when precalculating the whole space
    startnode.hindex_.clear();
    startnode.fatherindex_ = -1;

    Storage_.clear();
    Storage_.reserve(sizex*sizey);

    Storage_.push_back(startnode);

    std::vector<Indices> vectortemp;
    std::cout << "startnode: (" << startnode.x_ << ", " << startnode.y_ << ")" << std::endl;
    
    I_[startnode.x_][startnode.y_].push_back(startnode.index_);

    Queue_.clear();
    Queue_.push_back(startnode);

    while(1)
    {

        if(Queue_.empty())
        {
            break;
        }

        // we choose the best element
        KimNode newnode = Queue_[0]; 
        std::pop_heap(Queue_.begin(), Queue_.end(), 
            [](const KimNode& a, const KimNode& b){
                return a.g_ > b.g_;
            });

		//logfile << Storage_.size() << ", newnode: [" << newnode.x_ << ", " << newnode.y_ << "], hindex_ = ";
		//for (unsigned int i = 0; i < newnode.hindex_.size(); ++i)
		//{
		//	logfile << newnode.hindex_[i] << ", ";
		//}
		//logfile << std::endl;

        Queue_.pop_back();

		Storage_[newnode.index_].open_ = false;

		std::vector<std::pair<int, int> > son;
        newnode.sons(son);

        for(unsigned int j = 0; j < son.size(); ++j)
        {
			COUNT += 0.0000001;
            auto son_x = son[j].first;
            auto son_y = son[j].second;

            if(costmap_.data[son_x + son_y*sizex] > 0)
                continue;

            std::vector<int> newHindex = calSwing(newnode.hindex_, newnode.x_, newnode.y_, son_x, son_y);

            // We try our best to use the minimum computational cost to distinguish looping paths
            //if(newHindex.size() >= 2 && newHindex[newHindex.size()-1] == newHindex[newHindex.size()-2])
            //{
            //    continue;
            //}

            double newg = newnode.g_ + sqrt( (newnode.x_ - son_x)*(newnode.x_ - son_x) + (newnode.y_ - son_y)*(newnode.y_ - son_y) ) + COUNT;
            
            if(newg > FINITE_LENGTH)
                continue;

            double newf = 0;

            int newsonindex = Storage_.size();

            if(I_[son_x][son_y].empty() || !isEqualHC(newHindex, Storage_, I_, son_x, son_y))
            {
                // If the node has not been visited
                // Or, we find non-homotopic paths to the son location, we preserve them both. I.e., we create a new node
                KimNode newson(son_x, son_y, newg, newf, newHindex, newsonindex, newnode.index_);
                Queue_.push_back(newson);
                std::push_heap(Queue_.begin(), Queue_.end(),
                    [](const KimNode& a, const KimNode& b){
                    return a.g_ > b.g_;
                    });

                Storage_.push_back(newson);
                I_[son_x][son_y].push_back(newsonindex);

				Storage_[newnode.index_].adj_kimnode_.push_back(newsonindex);
				//Storage_[newsonindex].adj_kimnode_.push_back(newnode.index_);
            }
            else
            {
                // We can find the equivalent Hindex in the I matrix, but we may carry out a shortcut
                int oldindex = findEqualHCNode(newHindex, Storage_, I_, son_x, son_y);

				Storage_[newnode.index_].adj_kimnode_.push_back(oldindex);
				//Storage_[oldindex].adj_kimnode_.push_back(newnode.index_);

                if(Storage_[oldindex].g_ <= newg)
                {
                    continue;
                }

                Storage_[oldindex].g_ = newg;
                Storage_[oldindex].f_ = newf;
                Storage_[oldindex].hindex_ = newHindex; // In fact, it doesn't change
                Storage_[oldindex].fatherindex_ = newnode.index_;

                // If the old node is open, then it is in the queue. We only change its cost and parent
                if(Storage_[oldindex].open_)
                {
                    // Find the position of the son in the queue, and update it
                    auto iter = std::find_if(Queue_.begin(), Queue_.end(), 
                        [&](const KimNode& a){return a.index_ == oldindex;});
                    *iter = Storage_[oldindex];
                    std::make_heap(Queue_.begin(), Queue_.end(),
                        [](const KimNode& a, const KimNode& b){
                        return a.g_ > b.g_;
                        });
                }
                else
                {
                    // If the old node is closed, we may re-open it and push it into the queue
                    Storage_[oldindex].open_ = true;
                    Queue_.push_back(Storage_[oldindex]);
                    std::push_heap(Queue_.begin(), Queue_.end(),
                        [](const KimNode& a, const KimNode& b){
                        return a.g_ > b.g_;
                        });
                }
            }

        }

    }
    
	clock_t endTime = clock();

	std::cout << "(in functions) Time for Kim2013 homotopic planning precalculation: " << (double)(endTime - startTime)*1000.0 / CLOCKS_PER_SEC << "ms" << std::endl;


    std::cout << "YT: we have collected all valid waypoints" << std::endl;

    
}

} // namespace

