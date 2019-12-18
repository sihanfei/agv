#include "filter/base_filter.h"

sensor_camera::BaseFilter::BaseFilter(boost::array<float, NUM_STATE> state, boost::array<float, NUM_STATE*NUM_STATE> measurement_cov):
x_(Eigen::VectorXf(NUM_STATE, 1)),
R_(Eigen::MatrixXf(NUM_STATE, NUM_STATE)),
F_(Eigen::MatrixXf(NUM_STATE, NUM_STATE)),
P_(Eigen::MatrixXf(NUM_STATE, NUM_STATE)),
Q_(Eigen::MatrixXf(NUM_STATE, NUM_STATE)),
H_(Eigen::MatrixXf(NUM_STATE, NUM_STATE))
{
#ifdef DEBUG_BASEFILTER
    cout << "BaseFilter ctor start" << endl;
#endif 

    x_ << state[0], state[1], state[2], state[3];

    R_ << measurement_cov[0],  measurement_cov[1],  measurement_cov[2],  measurement_cov[3],  
          measurement_cov[4],  measurement_cov[5],  measurement_cov[6],  measurement_cov[7], 
          measurement_cov[8],  measurement_cov[9],  measurement_cov[10], measurement_cov[11], 
          measurement_cov[12], measurement_cov[13], measurement_cov[14], measurement_cov[15];
    
    F_ << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, 
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;

    P_ << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, 
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;

    Q_ << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, 
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;

    H_ << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, 
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;

#ifdef DEBUG_BASEFILTER
    cout << "BaseFilter ctor end" << endl;
#endif 
}

sensor_camera::BaseFilter::~BaseFilter()
{
}

Eigen::VectorXf sensor_camera::BaseFilter::getState() const
{
	return x_;
}

Eigen::MatrixXf sensor_camera::BaseFilter::getMeasurementCov() const
{
	return R_;
}

void sensor_camera::BaseFilter::setState(Eigen::VectorXf& x)
{
    x_ = x;
}

void sensor_camera::BaseFilter::setMeasurementCov(Eigen::MatrixXf& measurement_cov)
{
    R_ = measurement_cov;
}

int sensor_camera::BaseFilter::getUpdateCount() const
{
    return update_count_;
}