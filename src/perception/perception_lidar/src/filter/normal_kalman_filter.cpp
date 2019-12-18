#include "filter/normal_kalman_filter.h"

sensor_lidar::NormalKalmanFilter::NormalKalmanFilter(
	boost::array<float, NUM_STATE>& state, 
	boost::array<float, NUM_STATE*NUM_STATE>& measurement_cov):
sensor_lidar::BaseFilter(state, measurement_cov)
{
#ifdef DEBUG_NORMAL_KALMAN_FILTER
	cout << "NormalKalmanFilter ctor start" << endl;
#endif

#ifdef DEBUG_NORMAL_KALMAN_FILTER
	cout << "NormalKalmanFilter ctor start" << endl;
#endif
}

sensor_lidar::NormalKalmanFilter::~NormalKalmanFilter()
{
}


void sensor_lidar::NormalKalmanFilter::predict()
{
#ifdef DEBUG_NORMAL_KALMAN_FILTER
	cout << "NormalKalmanFilter predict start" << endl;
#endif

	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;

#ifdef DEBUG_NORMAL_KALMAN_FILTER
	cout << "NormalKalmanFilter predict endl;" << endl;
#endif
}

void sensor_lidar::NormalKalmanFilter::updateFilterGain()
{
#ifdef DEBUG_NORMAL_KALMAN_FILTER
	cout << "NormalKalmanFilter gain start" << endl;
#endif

	Eigen::MatrixXf PH_t = P_ * H_.transpose();
	Eigen::MatrixXf HPH_t = H_ * PH_t;
	K_ = PH_t * (HPH_t + R_).inverse();

#ifdef DEBUG_NORMAL_KALMAN_FILTER
	cout << "NormalKalmanFilter gain end" << endl;
#endif  
}

void sensor_lidar::NormalKalmanFilter::update(Eigen::VectorXf new_state, Eigen::MatrixXf measurement_cov, float detal_t)
{
#ifdef DEBUG_NORMAL_KALMAN_FILTER
	cout << "NormalKalmanFilter update start" << endl;
#endif

    setDetalTForTransitionMatrix(detal_t);
	setMeasurementCov(measurement_cov);

	predict();
	updateFilterGain();
	
	if(update_count_ > 3)
	{
		x_ = x_ + K_ * (new_state - H_ * x_);
	}
	else
	{
		x_ = new_state;
	}

	long x_size = x_.size();
	Eigen::MatrixXf I_ = Eigen::MatrixXf::Identity(x_size, x_size);
	P_ = (I_ - K_ * H_) * P_;

#ifdef DEBUG_NORMAL_KALMAN_FILTER
	cout << "NormalKalmanFilter update end" << endl;
#endif

}

void sensor_lidar::NormalKalmanFilter::setDetalTForTransitionMatrix(float detal_t)
{
#ifdef DEBUG_NORMAL_KALMAN_FILTER
	cout << "NormalKalmanFilter setDetalTForTransitionMatrix start" << endl;
#endif

    F_(0, 4) = detal_t;
	F_(1, 5) = detal_t;
	F_(2, 4) = detal_t;
	F_(3, 5) = detal_t;

#ifdef DEBUG_NORMAL_KALMAN_FILTER
	cout << "NormalKalmanFilter setDetalTForTransitionMatrix end" << endl;
#endif
}