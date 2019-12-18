#include "sensor_object/lidar_object.h"

LidarObject::LidarObject(
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
#ifdef DEBUG_LIDAROBJECT
    cout << "LidarObject ctor start" << endl;
#endif

#ifdef DEBUG_LIDAROBJECT
    cout << "LidarObject ctor end" << endl;
#endif
}

LidarObject::~LidarObject()
{

}

void LidarObject::update(BaseObject& obj)
{
#ifdef DEBUG_LIDAROBJECT
    cout << "LidarObject.update start" << endl;
#endif

    float detal_t = timestamp_.toSec() - obj.timestamp_.toSec();

    obj.timestamp_ = timestamp_;
    obj.setExistConfidence(getExistConfidence());
    obj.setClassConfidence(getClassConfidence());
    obj.setObjectClass(getObjectClass());
    
    Eigen::VectorXf new_state = getState();

    float gx = veh_llh_(0);
    float gy = veh_llh_(1);
    float yaw = att_[0];
    yaw = -yaw * M_PI / 180;
    float x1 = point4_[0];
    float y1 = point4_[1];
    float x2 = point4_[2];
    float y2 = point4_[3];
    float x3 = point4_[4];
    float y3 = point4_[5];
    float x4 = point4_[6];
    float y4 = point4_[7];

    float global_x1 = x1*cos(yaw)-y1*sin(yaw);
    float global_y1 = y1*cos(yaw)+x1*sin(yaw);
    float global_x2 = x2*cos(yaw)-y2*sin(yaw);
    float global_y2 = y2*cos(yaw)+x2*sin(yaw);
    float global_x3 = x3*cos(yaw)-y3*sin(yaw);
    float global_y3 = y3*cos(yaw)+x3*sin(yaw);
    float global_x4 = x4*cos(yaw)-y4*sin(yaw);
    float global_y4 = y4*cos(yaw)+x4*sin(yaw);
    global_x1 = global_x1 + gx;
    global_y1 = global_y1 + gy;
    global_x2 = global_x2 + gx;
    global_y2 = global_y2 + gy;
    global_x3 = global_x3 + gx;
    global_y3 = global_y3 + gy;
    global_x4 = global_x4 + gx;
    global_y4 = global_y4 + gy;
    float x_vector[4], y_vector[4];
    x_vector[0] = global_x1;
    y_vector[0] = global_y1;
    x_vector[1] = global_x2;
    y_vector[1] = global_y2;
    x_vector[2] = global_x3;
    y_vector[2] = global_y3;
    x_vector[3] = global_x4;
    y_vector[3] = global_y4;
    float xmin = *min_element(x_vector, x_vector+4);
    float xmax = *max_element(x_vector, x_vector+4);
    float ymin = *min_element(y_vector, y_vector+4);
    float ymax = *max_element(y_vector, y_vector+4);

    std::cout << "xmin = " << xmin << std::endl;
    std::cout << "xmax = " << xmax << std::endl;
    std::cout << "ymin = " << ymin << std::endl;
    std::cout << "ymax = " << ymax << std::endl;
    std::cout << "xmin_ = " << obj.xmin_ << std::endl;
    std::cout << "xmax_ = " << obj.xmax_ << std::endl;
    std::cout << "ymin_ = " << obj.ymin_ << std::endl;
    std::cout << "ymax_ = " << obj.ymax_ << std::endl;
    std::cout << "first_frame = " << obj.first_frame << std::endl;

    if (obj.first_frame)
    {
        obj.setSxMinList(0);
        obj.setSyMinList(0);
        obj.setSxMaxList(0);
        obj.setSyMaxList(0);
        obj.setDetalTList(detal_t);
        obj.xmin_ = xmin;
        obj.xmax_ = xmax;
        obj.ymin_ = ymin;
        obj.ymax_ = ymax;
        obj.first_frame = false;
    }
    else
    {
        obj.setSxMinList(xmin - obj.xmin_);
        obj.setSyMinList(ymin - obj.ymin_);
        obj.setSxMaxList(xmax - obj.xmax_);
        obj.setSyMaxList(ymax - obj.ymax_);
        std::cout << "delta_xmin = " << (xmin - obj.xmin_) << std::endl;
        std::cout << "delta_xmax_ = " << (ymin - obj.ymin_) << std::endl;
        std::cout << "delta_ymin_ = " << (xmax - obj.xmax_) << std::endl;
        std::cout << "delta_ymax_ = " << (ymax - obj.ymax_) << std::endl;
        obj.setDetalTList(detal_t);
        obj.xmin_ = xmin;
        obj.xmax_ = xmax;
        obj.ymin_ = ymin;
        obj.ymax_ = ymax;
    }

    pair<float, float> vxmin_vymin = calculateMeanVxVy(obj.getSxMinList(), obj.getSyMinList(), obj.getDetalTList());
    pair<float, float> vxmax_vymax = calculateMeanVxVy(obj.getSxMaxList(), obj.getSyMaxList(), obj.getDetalTList());

    obj.setVxMinList(vxmin_vymin.first);
    obj.setVyMinList(vxmin_vymin.second);
    obj.setVxMaxList(vxmax_vymax.first);
    obj.setVyMaxList(vxmax_vymax.second);

    float variance_vxmin = calculateVariance(obj.getVxMinList());
    float variance_vxmax = calculateVariance(obj.getVxMaxList());
    float variance_vymin = calculateVariance(obj.getVyMinList());
    float variance_vymax = calculateVariance(obj.getVyMaxList());

    new_state(4) = variance_vxmin > variance_vxmax ? vxmax_vymax.first : vxmin_vymin.first;
    new_state(5) = variance_vymin > variance_vymax ? vxmax_vymax.second : vxmin_vymin.second;
 
    std::cout << "vx = " << new_state(4) << std::endl;
    std::cout << "vy = " << new_state(5) << std::endl;

    obj.updateFilter(new_state, getMeasurementCov(), detal_t);

    obj.veh_llh_ = veh_llh_;

    obj.point4_[0] =  point4_[0];
    obj.point4_[1] =  point4_[1];
    obj.point4_[2] =  point4_[2];
    obj.point4_[3] =  point4_[3];
    obj.point4_[4] =  point4_[4];
    obj.point4_[5] =  point4_[5];
    obj.point4_[6] =  point4_[6];
    obj.point4_[7] =  point4_[7];



#ifdef DEBUG_LIDAROBJECT
    cout << "LidarObject.update endl" << endl;
#endif
}

void LidarObject::updateFilter(Eigen::VectorXf& new_state, Eigen::MatrixXf measurement_cov, float detal_t)
{
    filter_->update(new_state, measurement_cov, detal_t);
}

Eigen::VectorXf LidarObject::getState() const
{
    return filter_->getState();
}

Eigen::MatrixXf LidarObject::getMeasurementCov() const
{
    return filter_->getMeasurementCov();
}

// 外推目标
Eigen::VectorXf LidarObject::prediction(ros::Time pub_timestamp)
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