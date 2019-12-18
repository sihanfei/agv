#ifndef _NORMALKALMANFILTER_H_
#define _NORMALKALMANFILTER_H_

#include <iostream>
#include "base_filter.h"
using namespace std;

// #define DEBUG_NORMAL_KALMAN_FILTER

class NormalKalmanFilter: public BaseFilter
{
public:
    explicit NormalKalmanFilter(boost::array<float, NUM_STATE> state, boost::array<float, NUM_STATE*NUM_STATE> measurement_cov);
    ~NormalKalmanFilter();

    void predict();
    void updateFilterGain();
    void setDetalTForTransitionMatrix(float detal_t);
    void update(Eigen::VectorXf new_state, Eigen::MatrixXf measurement_cov, float detal_t);
};

#endif // _NORMALKALMANFILTER_H_