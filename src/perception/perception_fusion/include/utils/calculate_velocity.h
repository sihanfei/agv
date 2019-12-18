#ifndef _CALCULATE_VELOCITY_H_
#define _CALCULATE_VELOCITY_H_

#include <iostream>
#include <vector>
#include <numeric>
#include <boost/array.hpp>

using namespace std;

pair<float, float> calculateMeanVxVy(vector<float>& sx_list, vector<float>& sy_list, vector<float>& detal_t_list);
float calculateVariance(vector<float>& resultSet);

#endif // _CALCULATE_VELOCITY_H_