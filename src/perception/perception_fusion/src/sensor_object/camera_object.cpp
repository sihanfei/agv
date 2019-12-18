#include "sensor_object/camera_object.h"

CameraObject::CameraObject(
      int id,
      ros::Time timestamp,
      uint8_t object_class,
      float exist_confidence,
      float class_confidence,
      float att[3],
      Eigen::Vector3d& veh_llh,
      BaseFilter* filter,
      float point4[8]):
BaseObject(id, timestamp, object_class, exist_confidence, class_confidence, att, veh_llh, filter, point4)
{
}

CameraObject::~CameraObject()
{
}

void CameraObject::update(BaseObject& obj)
{
#ifdef DEBUG_FUSIONOBJECT
    cout << "CameraObject.update start" << endl;
#endif

    float detal_t = timestamp_.toSec() - obj.timestamp_.toSec();

    obj.timestamp_ = timestamp_;
    // obj.setExistConfidence(getExistConfidence());
    // obj.setClassConfidence(getClassConfidence());
    // obj.setObjectClass(getObjectClass());
    
    // Eigen::VectorXf new_state = getState();

    // obj.updateFilter(new_state, getMeasurementCov(), detal_t);

    // obj.veh_llh_ = veh_llh_;

#ifdef DEBUG_FUSIONOBJECT
    cout << "CameraObject.update endl" << endl;
#endif
}

void CameraObject::updateFilter(Eigen::VectorXf& new_state, Eigen::MatrixXf measurement_cov, float detal_t)
{
    filter_->update(new_state, measurement_cov, detal_t);
}

Eigen::VectorXf CameraObject::getState() const
{
    return filter_->getState();
}

Eigen::MatrixXf CameraObject::getMeasurementCov() const
{
    return filter_->getMeasurementCov();
}

// 外推目标
Eigen::VectorXf CameraObject::prediction(ros::Time pub_timestamp)
{
#ifdef DEBUG_FUSIONOBJECT
    cout << "CameraObject.prediction start" << endl;
#endif
    Eigen::VectorXf state = getState();

    if(filter_->getUpdateCount() > 3)
    {
        // 外推时间
        double time_diff = pub_timestamp.toSec() - timestamp_.toSec();

        // 外推state
        int vx = state(4);
        int vy = state(5);
        state(0) += time_diff * vx;
        state(1) += time_diff * vy;
        state(2) += time_diff * vx;
        state(3) += time_diff * vy;
    }

#ifdef DEBUG_FUSIONOBJECT
    cout << "CameraObject.prediction endl" << endl;
#endif

    return state;
}