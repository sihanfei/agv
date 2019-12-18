#ifndef _CALCULATESIMILARITY_H_
#define _CALCULATESIMILARITY_H_

#include <math.h>
#include <vector>
#include <Eigen/Dense>

using namespace std;

float calculateIOU(Eigen::VectorXf new_object_state, Eigen::VectorXf global_object_state);

float calculateVelocitySimilarity(float new_vx, float new_vy, float old_vx, float old_vy);

float calculateVariance(vector<float>& resultSet);

#endif // _CALCULATESIMILARITY_H_