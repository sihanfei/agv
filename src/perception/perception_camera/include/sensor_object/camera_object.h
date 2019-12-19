#ifndef _CameraObject_H_
#define _CameraObject_H_

#include "base_object.h"
#include "utils/tools.h"
#include "utils/calculate_velocity.h"
#include "utils/calculate_similarity.h"

// #define DEBUG_CameraObject
namespace sensor_camera
{
  class CameraObject: public sensor_camera::BaseObject
  {
  public:
      explicit CameraObject(
        ros::Time timestamp,
        uint8_t object_class,
        float exist_confidence,
        float class_confidence,
        sensor_camera::BaseFilter* world_filter,
        sensor_camera::BaseFilter* pix_filter,
        float state_3d[6]
      );

      ~CameraObject();
      
      // 更新障碍物信息
      void update(sensor_camera::BaseObject& obj);
      // 更新滤波器
      void updatePixFilter(Eigen::VectorXf& new_state, Eigen::MatrixXf measurement_cov, float detal_t);
      void updateWorldFilter(Eigen::VectorXf& new_state, Eigen::MatrixXf measurement_cov, float detal_t);

      // 获取状态量
      Eigen::VectorXf getPixState() const;
      Eigen::VectorXf getWorldState() const;
      // 获取协防差矩阵
      Eigen::MatrixXf getPixMeasurementCov() const;
      Eigen::MatrixXf getWorldMeasurementCov() const;
      // 外推目标
      Eigen::VectorXf prediction(ros::Time timestamp);
  };
}

#endif // _CameraObject_H_