#include "sensor_object/camera_object.h"

sensor_camera::CameraObject::CameraObject(
      ros::Time timestamp,
      uint8_t object_class,
      float exist_confidence,
      float class_confidence,
      sensor_camera::BaseFilter* world_filter,
      sensor_camera::BaseFilter* pix_filter,
      float state_3d[6]):
sensor_camera::BaseObject(timestamp, object_class, exist_confidence, class_confidence, world_filter, pix_filter, state_3d)
{
#ifdef DEBUG_CameraObject
    cout << "CameraObject ctor start" << endl;
#endif

#ifdef DEBUG_CameraObject
    cout << "CameraObject ctor end" << endl;
#endif
}

sensor_camera::CameraObject::~CameraObject()
{
}

void sensor_camera::CameraObject::update(sensor_camera::BaseObject& obj)
{
#ifdef DEBUG_CameraObject
    cout << "CameraObject.update start" << endl;
#endif

    obj.getState3d();
    
    float detal_t = timestamp_.toSec() - obj.timestamp_.toSec();

    obj.timestamp_ = timestamp_;
    obj.setExistConfidence(getExistConfidence());
    obj.setClassConfidence(getClassConfidence());
    obj.setObjectClass(getObjectClass());

    // ====================== update pix state ========================
    Eigen::VectorXf new_pix_state = getPixState();
    Eigen::MatrixXf new_pix_measurementCov = getPixMeasurementCov();
    obj.updatePixFilter(new_pix_state, new_pix_measurementCov, 0.0);
#ifdef DEBUG_CameraObject
    cout << "CameraObject.update pix end" << endl;
#endif

    // ====================== update world state ========================
    Eigen::VectorXf new_world_state = getWorldState();
    Eigen::VectorXf old_world_state = obj.getWorldState();
    obj.setSxList(new_world_state(0) - old_world_state(0));
    obj.setSyList(new_world_state(1) - old_world_state(1));
    obj.setDetalTList(detal_t);

    pair<float, float> vx_vy = sensor_camera::calculateMeanVxVy(obj.getSxList(), obj.getSyList(), obj.getDetalTList());

    obj.setVxList(vx_vy.first);
    obj.setVyList(vx_vy.second);
    new_world_state(2) = vx_vy.first;
    new_world_state(3) = vx_vy.second;
    
    obj.updateWorldFilter(new_world_state, getWorldMeasurementCov(), detal_t);
#ifdef DEBUG_CameraObject
    cout << "CameraObject.update world end" << endl;
#endif

    Eigen::VectorXf new_state_3d = getState3d();
    float state_3d[6] = {new_state_3d[0], new_state_3d[1], new_state_3d[2], new_state_3d[3], new_state_3d[4], new_state_3d[5]};
    obj.setState3d(state_3d);

#ifdef DEBUG_CameraObject
    cout << "CameraObject.update endl" << endl;
#endif
}

void sensor_camera::CameraObject::updatePixFilter(Eigen::VectorXf& new_state, Eigen::MatrixXf measurement_cov, float detal_t)
{
    pix_filter_->update(new_state, measurement_cov, detal_t);
}

Eigen::VectorXf sensor_camera::CameraObject::getPixState() const
{
    return pix_filter_->getState();
}

Eigen::MatrixXf sensor_camera::CameraObject::getPixMeasurementCov() const
{
    return pix_filter_->getMeasurementCov();
}

void sensor_camera::CameraObject::updateWorldFilter(Eigen::VectorXf& new_state, Eigen::MatrixXf measurement_cov, float detal_t)
{
    world_filter_->update(new_state, measurement_cov, detal_t);
}

Eigen::VectorXf sensor_camera::CameraObject::getWorldState() const
{
    return world_filter_->getState();
}

Eigen::MatrixXf sensor_camera::CameraObject::getWorldMeasurementCov() const
{
    return world_filter_->getMeasurementCov();
}

// 外推目标
Eigen::VectorXf sensor_camera::CameraObject::prediction(ros::Time pub_timestamp)
{
#ifdef DEBUG_CameraObject
    cout << "CameraObject.prediction start" << endl;
#endif
    Eigen::VectorXf state = getWorldState();

    if(world_filter_->getUpdateCount() > 3)
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

#ifdef DEBUG_CameraObject
    cout << "CameraObject.prediction endl" << endl;
#endif

    return state;
}