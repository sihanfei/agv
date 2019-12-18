#include "filter/base_filter.h"

BaseFilter::BaseFilter(boost::array<float, NUM_STATE> state, boost::array<float, NUM_STATE*NUM_STATE> measurement_cov):
x_(NUM_STATE, 1),
R_(NUM_STATE, NUM_STATE),
F_(NUM_STATE, NUM_STATE),
P_(NUM_STATE, NUM_STATE),
Q_(NUM_STATE, NUM_STATE),
H_(NUM_STATE, NUM_STATE)
{
#ifdef DEBUG_BASEFILTER
    cout << "BaseFilter ctor start" << endl;
#endif 

    x_ << state[0], state[1], state[2], state[3], state[4], state[5];

    R_ << measurement_cov[0],   measurement_cov[1],  measurement_cov[2],  measurement_cov[3],  measurement_cov[4],  measurement_cov[5],
          measurement_cov[6],   measurement_cov[7],  measurement_cov[8],  measurement_cov[9],  measurement_cov[10], measurement_cov[11],
          measurement_cov[12],  measurement_cov[13], measurement_cov[14], measurement_cov[15], measurement_cov[16], measurement_cov[17],
          measurement_cov[16],  measurement_cov[17], measurement_cov[18], measurement_cov[19], measurement_cov[20], measurement_cov[21],
          measurement_cov[22],  measurement_cov[23], measurement_cov[24], measurement_cov[25], measurement_cov[26], measurement_cov[27],
          measurement_cov[28],  measurement_cov[29], measurement_cov[30], measurement_cov[31], measurement_cov[32], measurement_cov[33];
    
    F_ << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    P_ << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    Q_ << 250000.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 250000.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 250000.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 250000.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 500000.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 500000.0;

    H_ << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

#ifdef DEBUG_BASEFILTER
    cout << "BaseFilter ctor end" << endl;
#endif 
}

BaseFilter::~BaseFilter()
{
    // x_.resize(0);
    // R_.resize(0,0);
    // F_.resize(0,0);
    // P_.resize(0,0);
    // Q_.resize(0,0);
    // H_.resize(0,0);
    // K_.resize(0,0);
}

Eigen::VectorXf BaseFilter::getState() const
{
	return x_;
}

Eigen::MatrixXf BaseFilter::getMeasurementCov() const
{
	return R_;
}

void BaseFilter::setState(Eigen::VectorXf& x)
{
    x_ = x;
}

void BaseFilter::setMeasurementCov(Eigen::MatrixXf& measurement_cov)
{
    R_ = measurement_cov;
}

int BaseFilter::getUpdateCount() const
{
    return update_count_;
}