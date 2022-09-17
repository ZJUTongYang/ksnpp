#include "optimal_tmv_functions.h"
#include "Atom.h"
#include <ctime>

#include <corecrt_math_defines.h>

void calAngleDiff(const Path2D& P, std::vector<double>& A)
{
	A.resize(P.data.size(), 0);
	for (unsigned int point_index = 0; point_index < P.data.size(); ++point_index)
	{
		if (point_index <= 2 || point_index >= P.data.size() - 3)
			A[point_index] = M_PI;
		else
		{
			std::pair<double, double> pre_pos;
			pre_pos.first = (P.data[point_index - 1].first + P.data[point_index - 2].first + P.data[point_index - 3].first) / 3;
			pre_pos.second = (P.data[point_index - 1].second + P.data[point_index - 2].second + P.data[point_index - 3].second) / 3;

			double pre_angle = atan2(pre_pos.second - P.data[point_index].second, pre_pos.first - P.data[point_index].first);

			std::pair<double, double> post_pos;
			post_pos.first = (P.data[point_index + 1].first + P.data[point_index + 2].first + P.data[point_index + 3].first) / 3;
			post_pos.second = (P.data[point_index + 1].second + P.data[point_index + 2].second + P.data[point_index + 3].second) / 3;

			double post_angle = atan2(post_pos.second - P.data[point_index].second, post_pos.first - P.data[point_index].first);

			A[point_index] = fabs(normalize_angle(pre_angle - post_angle));

		}
	}
}

bool isCspaceWallFollowing(std::pair<int, int> p, std::pair<double, double>& obs)
{
	int x = p.first;
	int y = p.second;
	if (costmap_.data[(x - 1) + (y - 1)*MAX_X_SIZE] > 0)
	{
		obs.first = x - 0.5;
		obs.second = x - 0.5;
		return true;
	}
	if (costmap_.data[(x - 1) + y * MAX_X_SIZE] > 0)
	{
		obs.first = x - 0.5;
		obs.second = y + 0.5;
		return true;
	}
	if (costmap_.data[(x - 1) + (y + 1)*MAX_X_SIZE] > 0)
	{
		obs.first = x - 0.5;
		obs.second = y + 1.5;
		return true;
	}
	if (costmap_.data[x + (y - 1)*MAX_X_SIZE] > 0)
	{
		obs.first = x + 0.5;
		obs.second = y - 0.5;
		return true;
	}
	if (costmap_.data[x + (y + 1)*MAX_X_SIZE] > 0)
	{
		obs.first = x + 0.5;
		obs.second = y + 1.5;
		return true;
	}
	if (costmap_.data[x + 1 + (y - 1)*MAX_X_SIZE] > 0)
	{
		obs.first = x + 1.5;
		obs.second = y - 0.5;
		return true;
	}
	if (costmap_.data[x + 1 + y * MAX_X_SIZE] > 0)
	{
		obs.first = x + 1.5;
		obs.second = y + 0.5;
		return true;
	}
	if (costmap_.data[x + 1 + (y + 1)*MAX_X_SIZE] > 0)
	{
		obs.first = x + 1.5;
		obs.second = y + 1.5;
		return true;
	}
	obs.first = -1.0;
	obs.second = -1.0;
	return false;
}

void UPScspace(Path2D& result_path)
{
	//Note: The input path has been added 0.5. 
	//std::cout << "test 1" << std::endl;
	// We firstly remove repeated waypoitns, to speed up the process
	result_path.data.erase(std::unique(result_path.data.begin(), result_path.data.end()), result_path.data.end());
	int i = result_path.data.size() - 2;
	while (i >= 2)
	{
		if (result_path.data[i + 1].first == result_path.data[i - 1].first && result_path.data[i + 1].second == result_path.data[i - 1].second)
		{
			result_path.data.erase(result_path.data.begin() + i, result_path.data.begin() + i + 2);
		}
		i = i - 1;
		if (i > result_path.data.size() - 2)
		{
			i = result_path.data.size() - 2;
		}

	}
	//std::cout << "test 2" << std::endl;

	std::vector<double> angle_diff;
	calAngleDiff(result_path, angle_diff);
	//std::cout << "test 3" << std::endl;

	bool stable = false;
	int COUNT = 0;
	while (1)
	{
		COUNT++;
		if (COUNT > 50)
			break;
		if (stable)
			break;
		//std::cout << "test 4" << std::endl;

		stable = true;

		i = result_path.data.size() - 4;
		while (i >= 3)
		{
			int min_index = i;
			int min_index_temp = min_index;
			while (min_index_temp >= 3)
			{
				//std::cout << "test 5" << std::endl;

				std::pair<double, double> obs;
				std::pair<int, int> theWaypoint;
				theWaypoint.first = floor(result_path.data[min_index_temp].first);
				theWaypoint.second = floor(result_path.data[min_index_temp].second);
				bool b = isCspaceWallFollowing(theWaypoint, obs);
				obs.first += 0.5;
				obs.second += 0.5;

				std::pair<double, double> obs_diff;
				obs_diff.first = obs.first - result_path.data[min_index_temp].first;
				obs_diff.second = obs.second - result_path.data[min_index_temp].second;

				std::pair<double, double> pre_pos;
				pre_pos.first = (result_path.data[min_index_temp - 3].first + result_path.data[min_index_temp - 2].first + result_path.data[min_index_temp - 1].first) / 3;
				pre_pos.second = (result_path.data[min_index_temp - 3].second + result_path.data[min_index_temp - 2].second + result_path.data[min_index_temp - 1].second) / 3;
				std::pair<double, double> pre_diff;
				pre_diff.first = pre_pos.first - result_path.data[min_index_temp].first;
				pre_diff.second = pre_pos.second - result_path.data[min_index_temp].second;

				std::pair<double, double> post_pos;
				post_pos.first = (result_path.data[min_index_temp +1].first + result_path.data[min_index_temp + 2].first + result_path.data[min_index_temp +3].first) / 3;
				post_pos.second = (result_path.data[min_index_temp +1].second + result_path.data[min_index_temp + 2].second + result_path.data[min_index_temp +3].second) / 3;
				std::pair<double, double> post_diff;
				post_diff.first = post_pos.first - result_path.data[min_index_temp].first;
				post_diff.second = post_pos.second - result_path.data[min_index_temp].second;

				std::pair<double, double> bisector;
				bisector.first = pre_diff.first + post_diff.first;
				bisector.second = pre_diff.second + post_diff.second;
				//std::cout << "test 6" << std::endl;

				if (bisector.first != 0 && bisector.second != 0)
				{
					if (b && (bisector.first*obs_diff.first + bisector.second*obs_diff.second) < 0)
					{
						if (angle_diff[min_index_temp] < 2.4)
						{
							min_index = min_index_temp;
							break;
						}
					}
					else
					{
						if (angle_diff[min_index_temp] < 2.6)
						{
							min_index = min_index_temp;
							break;
						}
					}
				}
				min_index_temp--;
			}
			if (min_index_temp <= 2)
			{
				break;
			}
			//std::cout << "test 7" << std::endl;

			int s_index = min_index - 1;
			int e_index = min_index + 1;
			if (straightPath(result_path.data[s_index], result_path.data[e_index]))
			{
				bool forward = true;
				bool forward_end = true;
				bool backward_end = true;
				// We evaluate the interval
				while (1)
				{
					//std::cout << "test 8" << std::endl;

					if (forward_end && backward_end)
					{
						break;
					}
					if (forward)
					{
						if (e_index == result_path.data.size() - 1)
						{
							forward_end = true;
							forward = false;
							continue;
						}
						else
						{
							int temp_e_index = e_index + 1;
							if (straightPath(result_path.data[s_index], result_path.data[temp_e_index]))
							{
								e_index = temp_e_index;
								forward = false;
							}
							else
							{
								forward_end = true;
								forward = false;
							}
						}
					}
					else
					{
						if (s_index == 1)
						{
							backward_end = true;
							forward = true;
							continue;
						}
						else
						{
							int temp_s_index = s_index - 1;
							if (straightPath(result_path.data[temp_s_index], result_path.data[e_index]))
							{
								s_index = temp_s_index;
								forward = true;
							}
							else
							{
								backward_end = true;
								forward = true;
							}
						}
					}
				}

				//std::cout << "test 9" << std::endl;


				// We replace the old path
				std::vector<std::pair<int, int> > intP;
				classicBresenham(result_path.data[s_index].first, result_path.data[s_index].second, result_path.data[e_index].first, result_path.data[e_index].second, intP);
				std::vector<std::pair<double, double> > new_path;
				for (auto iter = intP.begin(); iter != intP.end(); ++iter)
				{
					new_path.push_back(std::pair<double, double>(iter->first + 0.5, iter->second + 0.5));
				}

				//std::cout << "test 10" << std::endl;


				// If the new path is different to the old path, stable = false;
				std::vector<std::pair<double, double> > old_path(result_path.data.begin() + s_index, result_path.data.begin() + e_index + 1);
				if (old_path.size() != new_path.size())
					stable = false;
				else
				{
					for (unsigned int j = 0; j < new_path.size(); ++j)
					{
						if (new_path[j].first != old_path[j].first || new_path[j].second != new_path[j].second)
						{
							stable = false;
							break;
						}
					}
				}

				//std::cout << "test 11" << std::endl;


				// We update the path
				Path2D result_path_temp;
				//std::cout << "s_e_index = " << s_index << ", " << e_index << std::endl;
				result_path_temp.data.assign(result_path.data.begin(), result_path.data.begin() + s_index + 1);
				//std::cout << "result_path.size() = " << result_path.data.size() << std::endl;
				//std::cout << "new_path.size() = " << new_path.size() << std::endl;
				if (new_path.size() == 1)
					result_path_temp.data.insert(result_path_temp.data.end(), new_path[0]);
				else
					result_path_temp.data.insert(result_path_temp.data.end(), new_path.begin() + 1, new_path.end() - 1);

				result_path_temp.data.insert(result_path_temp.data.end(), result_path.data.begin() + e_index, result_path.data.end());

				//std::cout << "Test 11.1" << std::endl;
				std::vector<double> angle_diff_temp;
				angle_diff_temp.resize(result_path_temp.data.size(), 0);
				calAngleDiff(result_path_temp, angle_diff_temp);


				//std::cout << "Test 11.2" << std::endl;

				angle_diff.swap(angle_diff_temp);
				result_path.data.swap(result_path_temp.data);

				//std::cout << "test 12" << std::endl;

			}
			i = s_index;
		}
	}
}
void UPSmspace(Path2D& P)
{

}


void optimal_tmv()
{
	std::cout << "start optimal_tmv" << std::endl;
	int N = multi_goal_multi_solutions.size();

	std::vector<std::vector<Path2D> > Gamma;
	std::vector<std::vector<double> > F;

	// initialise Gamma
	for (unsigned int i = 0; i < N; ++i)
	{
		std::vector<Path2D> gamma_temp;
		Gamma.push_back(gamma_temp);
		std::vector<double> F_temp;
		F.push_back(F_temp);
	}

	std::cout << "start create Gamma and F" << std::endl;

	// Gamma{1} is the optimal TR from goal 0 (the home configruation) to goal 1
	Gamma[0].clear();
	for (auto iter = multi_goal_multi_solutions[0].begin(); iter != multi_goal_multi_solutions[0].end(); ++iter)
	{
		Gamma[0].push_back(*iter);
		F[0].push_back(pathLength(iter->data));
	}

	std::cout << "finish create Gamma[0] and F[0]" << std::endl;

	double UPStime = 0;
	double UPSnum = 0;
	for (unsigned goal_index = 1; goal_index < N; ++goal_index)
	{
		for (unsigned int i = 0; i < multi_goal_multi_solutions[goal_index - 1].size(); ++i)
		{
			for (unsigned int j = 0; j < multi_goal_multi_solutions[goal_index].size(); ++j)
			{
				Path2D pathtemp;
				//std::cout << "i, j = " << i << ", " << j << std::endl;
				pathtemp.data.assign(multi_goal_multi_solutions[goal_index - 1][i].data.begin(), multi_goal_multi_solutions[goal_index - 1][i].data.end());
				std::reverse(pathtemp.data.begin(), pathtemp.data.end());
				pathtemp.data.insert(pathtemp.data.end(), multi_goal_multi_solutions[goal_index][j].data.begin(), multi_goal_multi_solutions[goal_index][j].data.end());
				clock_t startTime = clock();
				UPScspace(pathtemp);
				clock_t endTime = clock();
				UPStime += (double)(endTime - startTime) *1000.0 / CLOCKS_PER_SEC;
				UPSnum++;
				Gamma[goal_index].push_back(pathtemp);
				F[goal_index].push_back(pathLength(pathtemp.data));
			}
		}
	}
	std::cout << "Time for UPS (all): " << UPStime << "ms" << ", Time for each UPS: " << UPStime / UPSnum << "ms" << std::endl;


	// 2022.06.22 We need not finish this, because we only need to find the computational time for the path shortening. 

	std::vector<std::vector<Path2D> > Gamma_star;
	std::vector<std::vector<double> > F_star;
	// We initialise the list
	for (unsigned int i = 0; i < N; ++i)
	{
		std::vector<Path2D> gamma_temp;
		Gamma_star.push_back(gamma_temp);
		std::vector<double> F_temp;
		F_star.push_back(F_temp);
	}

	for (unsigned int goal_index = 1; goal_index < N; ++goal_index)
	{
		for (unsigned int j = 0; j < multi_goal_multi_solutions[goal_index].size(); ++j)
		{
			//double min_cost = 
		}
	}


	

}
void optimal_ttsp()
{

}
void optimal_tmv_with_precalculated_tr()
{

}

