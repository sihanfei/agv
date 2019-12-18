#ifndef _CAMERAOBJECT_H_
#define _CAMERAOBJECT_H_

#include "base_object.h"

class CameraObject: public BaseObject
{
public:
    explicit CameraObject(
      int id,
      ros::Time timestamp,
      uint8_t object_class,
      float exist_confidence,
      float class_confidence,
      float att[3],
      Eigen::Vector3d& veh_llh,
      BaseFilter* filter,
      float point4[8]
    );

    ~CameraObject();
    
    // 更新障碍物信息
    void update(BaseObject& obj);
    // 更新滤波器
    void updateFilter(Eigen::VectorXf& new_state, Eigen::MatrixXf measurement_cov, float detal_t);
    // 获取状态量
    Eigen::VectorXf getState() const;
    // 获取协防差矩阵
    Eigen::MatrixXf getMeasurementCov() const;
    // 外推目标
    Eigen::VectorXf prediction(ros::Time pub_timestamp);

};

#endif // _CAMERAOBJECT_H_