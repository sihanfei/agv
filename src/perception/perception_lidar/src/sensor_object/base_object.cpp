#include "sensor_object/base_object.h"

sensor_lidar::BaseObject::BaseObject(
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
timestamp_(timestamp),
object_class_(object_class),
exist_confidence_(exist_confidence),
class_confidence_(class_confidence),
filter_(filter), 
x1_(x1), 
y1_(y1), 
x2_(x2), 
y2_(y2), 
x3_(x3), 
y3_(y3), 
x4_(x4), 
y4_(y4)
{
// #ifdef DEBUG_BASEOBJECT
//     cout << "BaseObject ctor start" << endl;
// #endif

    sxmin_list_.reserve(20);
    symin_list_.reserve(20);
    vxmin_list_.reserve(20);
    vymin_list_.reserve(20);

    sxmax_list_.reserve(20);
    symax_list_.reserve(20);
    vxmax_list_.reserve(20);
    vymax_list_.reserve(20);

    detal_t_list_.reserve(20);

// #ifdef DEBUG_BASEOBJECT
//     cout << "BaseObject ctor end" << endl;
// #endif
}

sensor_lidar::BaseObject::~BaseObject()
{
    delete filter_;
}

float sensor_lidar::BaseObject::getExistConfidence() const
{
    return exist_confidence_;
}

float sensor_lidar::BaseObject::getClassConfidence() const
{
    return class_confidence_;
}

uint8_t sensor_lidar::BaseObject::getObjectClass() const
{
    return object_class_;
}

std::vector<float>& sensor_lidar::BaseObject::getSxMinList()
{
    return sxmin_list_;
}

std::vector<float>& sensor_lidar::BaseObject::getSyMinList()
{
    return symin_list_;
}

std::vector<float>& sensor_lidar::BaseObject::getSxMaxList()
{
    return sxmax_list_;
}

std::vector<float>& sensor_lidar::BaseObject::getSyMaxList()
{
    return symax_list_;
}

std::vector<float>& sensor_lidar::BaseObject::getVxMinList()
{
    return vxmin_list_;
}

std::vector<float>& sensor_lidar::BaseObject::getVyMinList()
{
    return vymin_list_;
}

std::vector<float>& sensor_lidar::BaseObject::getVxMaxList()
{
    return vxmax_list_;
}

std::vector<float>& sensor_lidar::BaseObject::getVyMaxList()
{
    return vymax_list_;
}

std::vector<float>& sensor_lidar::BaseObject::getDetalTList()
{
    return detal_t_list_;
}

void sensor_lidar::BaseObject::setSxMinList(float detal_sxmin)
{
    sxmin_list_.push_back(detal_sxmin);
    if(sxmin_list_.size() > erase_num) 
        sxmin_list_.erase(sxmin_list_.begin());
}

void sensor_lidar::BaseObject::setSyMinList(float detal_symin)
{
    symin_list_.push_back(detal_symin);
    if(symin_list_.size() > erase_num) 
        symin_list_.erase(symin_list_.begin());
}

void sensor_lidar::BaseObject::setSxMaxList(float detal_sxmax)
{
    sxmax_list_.push_back(detal_sxmax);
    if(sxmax_list_.size() > erase_num) 
        sxmax_list_.erase(sxmax_list_.begin());
}

void sensor_lidar::BaseObject::setSyMaxList(float detal_symax)
{
    symax_list_.push_back(detal_symax);
    if(symax_list_.size() > erase_num) 
        symax_list_.erase(symax_list_.begin());
}

void sensor_lidar::BaseObject::setVxMinList(float detal_vxmin)
{
    vxmin_list_.push_back(detal_vxmin);
    if(vxmin_list_.size() > erase_num) 
        vxmin_list_.erase(vxmin_list_.begin());
}

void sensor_lidar::BaseObject::setVyMinList(float detal_vymin)
{
    vymin_list_.push_back(detal_vymin);
    if(vymin_list_.size() > erase_num) 
        vymin_list_.erase(vymin_list_.begin());
}

void sensor_lidar::BaseObject::setVxMaxList(float detal_vxmax)
{
    vxmax_list_.push_back(detal_vxmax);
    if(vxmax_list_.size() > erase_num) 
        vxmax_list_.erase(vxmax_list_.begin());
}

void sensor_lidar::BaseObject::setVyMaxList(float detal_vymax)
{
    vymax_list_.push_back(detal_vymax);
    if(vymax_list_.size() > erase_num) 
        vymax_list_.erase(vymax_list_.begin());
}

void sensor_lidar::BaseObject::setDetalTList(float detal_t)
{
    detal_t_list_.push_back(detal_t);
    if(detal_t_list_.size() > erase_num) 
        detal_t_list_.erase(detal_t_list_.begin());
}

void sensor_lidar::BaseObject::setExistConfidence(float exist_confidence)
{
    exist_confidence_ = exist_confidence;
}

void sensor_lidar::BaseObject::setClassConfidence(float class_confidence)
{
    class_confidence_ = class_confidence;
}

void sensor_lidar::BaseObject::setObjectClass(uint8_t object_class)
{
    object_class_ = object_class;
}

float sensor_lidar::BaseObject::calculateSimilarity(sensor_lidar::BaseObject& obj)
{
    // =========================== iou ===========================
    float iou = sensor_lidar::calculateIOU(filter_->getState(), obj.filter_->getState());
    // cout << "iou: " << iou << endl;

    // =========================== velocity similarity ===========================
    // bool has_velocity_similarity = false;
    // float velocity_similarity = 0;
    // if(obj.getVxMinList().size() > 0)
    // {
    //     float old_variance_vxmin = sensor_lidar::calculateVariance(obj.getVxMinList());
    //     float old_variance_vymin = sensor_lidar::calculateVariance(obj.getVyMinList());
    //     float old_variance_vxmax = sensor_lidar::calculateVariance(obj.getVxMaxList());
    //     float old_variance_vymax = sensor_lidar::calculateVariance(obj.getVyMaxList());
    //     vector<float> old_vx_list(old_variance_vxmin > old_variance_vxmax ? obj.getVxMaxList() : obj.getVxMinList());
    //     vector<float> old_vy_list(old_variance_vxmin > old_variance_vxmax ? obj.getVyMaxList() : obj.getVyMinList());

    //     Eigen::VectorXf new_state = filter_->getState();
    //     Eigen::VectorXf old_state = obj.filter_->getState();
    //     vector<float> new_sx_min_list(obj.getSxMinList());
    //     vector<float> new_sy_min_list(obj.getSyMinList());
    //     vector<float> new_sx_max_list(obj.getSxMaxList());
    //     vector<float> new_sy_max_list(obj.getSyMaxList());
    //     vector<float> new_detal_t_list(obj.getDetalTList());
    //     new_sx_min_list.push_back(new_state(0) - old_state(0));
    //     new_sy_min_list.push_back(new_state(1) - old_state(1));
    //     new_sx_max_list.push_back(new_state(2) - old_state(2));
    //     new_sy_max_list.push_back(new_state(3) - old_state(3));
    //     new_detal_t_list.push_back(timestamp_.toSec() - obj.timestamp_.toSec());

    //     pair<float, float> vxmin_vymin = sensor_lidar::calculateMeanVxVy(new_sx_min_list, new_sy_min_list, new_detal_t_list);
    //     pair<float, float> vxmax_vymax = sensor_lidar::calculateMeanVxVy(new_sx_max_list, new_sy_max_list, new_detal_t_list);
    //     vector<float> new_vx_min_list(obj.getVxMinList());
    //     vector<float> new_vy_min_list(obj.getVyMinList());
    //     vector<float> new_vx_max_list(obj.getVxMinList());
    //     vector<float> new_vy_max_list(obj.getVyMaxList());
    //     new_vx_min_list.push_back(vxmin_vymin.first);
    //     new_vy_min_list.push_back(vxmin_vymin.second);
    //     new_vx_max_list.push_back(vxmax_vymax.first);
    //     new_vy_max_list.push_back(vxmax_vymax.second);
    //     float new_variance_vxmin = sensor_lidar::calculateVariance(new_vx_min_list);
    //     float new_variance_vymin = sensor_lidar::calculateVariance(new_vy_min_list);
    //     float new_variance_vxmax = sensor_lidar::calculateVariance(new_vx_max_list);
    //     float new_variance_vymax = sensor_lidar::calculateVariance(new_vy_max_list);
    //     vector<float> new_vx_list(new_variance_vxmin > new_variance_vxmax ? new_vx_min_list : new_vx_max_list);
    //     vector<float> new_vy_list(new_variance_vymin > new_variance_vymax ? new_vy_min_list : new_vy_max_list);

    //     velocity_similarity = sensor_lidar::calculateVelocitySimilarity(new_vx_list, new_vy_list, old_vx_list, old_vy_list);
    //     // cout << "velocity_similarity: " << velocity_similarity << endl;
    //     has_velocity_similarity = true;
    // }
    
    float result = iou;
    // if(has_velocity_similarity)
    // {
    //     result = (iou + 0.2 * velocity_similarity) / 2;
    // }

    return result;
}
