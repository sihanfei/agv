#ifndef _CALCULATESIMILARITY_H_
#define _CALCULATESIMILARITY_H_

#include <math.h>
#include <vector>
#include <Eigen/Dense>

using namespace std;

namespace sensor_lidar{
    float calculateIOU(Eigen::VectorXf new_object_state, Eigen::VectorXf global_object_state);
    float calculateVelocitySimilarity(vector<float>& new_vx_list, vector<float>& new_vy_list, vector<float>& old_vx_list, vector<float>& old_vy_list);
}

#endif // _CALCULATESIMILARITY_H_