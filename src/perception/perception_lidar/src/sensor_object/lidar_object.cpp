#include "sensor_object/lidar_object.h"

sensor_lidar::LidarObject::LidarObject(
      const ros::Time& timestamp,
      uint8_t object_class,
      float exist_confidence,
      float class_confidence,
      sensor_lidar::BaseFilter* filter,
      float x1,
      float y1,
      float x2,
      float y2,
      float x3,
      float y3,
      float x4,
      float y4):
sensor_lidar::BaseObject(timestamp, object_class, exist_confidence, class_confidence, filter, x1, y1, x2, y2, x3, y3, x4, y4)
{
#ifdef DEBUG_LIDAROBJECT
    cout << "LidarObject ctor start" << endl;
#endif

#ifdef DEBUG_LIDAROBJECT
    cout << "LidarObject ctor end" << endl;
#endif
}

sensor_lidar::LidarObject::~LidarObject()
{
}

void sensor_lidar::LidarObject::update(sensor_lidar::BaseObject& obj)
{
#ifdef DEBUG_LIDAROBJECT
    cout << "LidarObject.update start" << endl;
#endif
    
    // float detal_t = obj.timestamp_.toSec() - timestamp_.toSec();
    float detal_t = timestamp_.toSec() - obj.timestamp_.toSec();

    obj.timestamp_ = timestamp_;
    obj.setExistConfidence(getExistConfidence());
    obj.setClassConfidence(getClassConfidence());
    obj.setObjectClass(getObjectClass());

    Eigen::VectorXf new_state = getState();
    Eigen::VectorXf old_state = obj.getState();
    obj.setSxMinList(new_state(0) - old_state(0));
    obj.setSyMinList(new_state(1) - old_state(1));
    obj.setSxMaxList(new_state(2) - old_state(2));
    obj.setSyMaxList(new_state(3) - old_state(3));
    obj.setDetalTList(detal_t);

    pair<float, float> vxmin_vymin = sensor_lidar::calculateMeanVxVy(obj.getSxMinList(), obj.getSyMinList(), obj.getDetalTList());
    pair<float, float> vxmax_vymax = sensor_lidar::calculateMeanVxVy(obj.getSxMaxList(), obj.getSyMaxList(), obj.getDetalTList());

    obj.setVxMinList(vxmin_vymin.first);
    obj.setVyMinList(vxmin_vymin.second);
    obj.setVxMaxList(vxmax_vymax.first);
    obj.setVyMaxList(vxmax_vymax.second);

    float variance_vxmin = sensor_lidar::calculateVariance(obj.getVxMinList());
    float variance_vxmax = sensor_lidar::calculateVariance(obj.getVxMaxList());
    float variance_vymin = sensor_lidar::calculateVariance(obj.getVyMinList());
    float variance_vymax = sensor_lidar::calculateVariance(obj.getVyMaxList());

    new_state(4) = variance_vxmin > variance_vxmax ? vxmax_vymax.first : vxmin_vymin.first;
    new_state(5) = variance_vymin > variance_vymax ? vxmax_vymax.second : vxmin_vymin.second;
    
    obj.updateFilter(new_state, getMeasurementCov(), detal_t);

    obj.x1_ = x1_;
    obj.y1_ = y1_;
    obj.x2_ = x2_;
    obj.y2_ = y2_;
    obj.x3_ = x3_;
    obj.y3_ = y3_;
    obj.x4_ = x4_;
    obj.y4_ = y4_;

#ifdef DEBUG_LIDAROBJECT
    cout << "LidarObject.update endl" << endl;
#endif
}

void sensor_lidar::LidarObject::updateFilter(Eigen::VectorXf& new_state, Eigen::MatrixXf measurement_cov, float detal_t)
{
    filter_->update(new_state, measurement_cov, detal_t);
}

Eigen::VectorXf sensor_lidar::LidarObject::getState() const
{
    return filter_->getState();
}

Eigen::MatrixXf sensor_lidar::LidarObject::getMeasurementCov() const
{
    return filter_->getMeasurementCov();
}

// 外推目标
Eigen::VectorXf sensor_lidar::LidarObject::prediction(ros::Time pub_timestamp)
{
#ifdef DEBUG_LIDAROBJECT
    cout << "LidarObject.prediction start" << endl;
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

#ifdef DEBUG_LIDAROBJECT
    cout << "LidarObject.prediction endl" << endl;
#endif

    return state;
}
