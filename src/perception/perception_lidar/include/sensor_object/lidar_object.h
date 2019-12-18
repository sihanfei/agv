#ifndef _LIDAROBJECT_H_
#define _LIDAROBJECT_H_

#include "base_object.h"
#include "utils/calculate_velocity.h"
#include "utils/calculate_similarity.h"

// #define DEBUG_LIDAROBJECT
namespace sensor_lidar
{
  class LidarObject: public sensor_lidar::BaseObject
  {
  public:
      explicit LidarObject(
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
        float y4);

      ~LidarObject();
      
      // 更新障碍物信息
      void update(sensor_lidar::BaseObject& obj);
      // 更新滤波器
      void updateFilter(Eigen::VectorXf& new_state, Eigen::MatrixXf measurement_cov, float detal_t);
      // 获取状态量
      Eigen::VectorXf getState() const;
      // 获取协防差矩阵
      Eigen::MatrixXf getMeasurementCov() const;
      // 外推目标
      Eigen::VectorXf prediction(ros::Time timestamp);

  };
}

#endif // _LIDAROBJECT_H_
