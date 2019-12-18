#ifndef _BASEFILTER_H_
#define _BASEFILTER_H_

#include <iostream>
#include <Eigen/Dense>
#include <boost/array.hpp>

using namespace std;

#define NUM_STATE 6

// #define DEBUG_BASEFILTER

namespace sensor_lidar
{
    class BaseFilter
    {
    public:
        explicit BaseFilter(boost::array<float, NUM_STATE>& state, boost::array<float, NUM_STATE*NUM_STATE>& measurement_cov);
        virtual ~BaseFilter();

        void setState(Eigen::VectorXf& x);
        void setMeasurementCov(Eigen::MatrixXf& measurement_cov);
        Eigen::VectorXf getState() const;
        Eigen::MatrixXf getMeasurementCov() const;
        int getUpdateCount() const;

        virtual void predict() = 0;
        virtual void updateFilterGain() = 0;
        virtual void setDetalTForTransitionMatrix(float detal_t) = 0;
        virtual void update(Eigen::VectorXf new_state, Eigen::MatrixXf measurement_cov, float detal_t) = 0;

    protected:
        // state vector
        Eigen::VectorXf x_;
        // state transition matrix
        Eigen::MatrixXf F_;
        // state covariance matrix
        Eigen::MatrixXf P_;
        // process noise convariance matrix
        Eigen::MatrixXf Q_;
        // measurement transition matrix
        Eigen::MatrixXf H_;
        // measurement covariance matrix
        Eigen::MatrixXf R_;
        // filter gain matrix
        Eigen::MatrixXf K_;

        int update_count_ = 0;
    };
}


#endif // _BASEFILTER_H_