#include "sensor_object/base_object.h"

BaseObject::BaseObject(
    int id,
    ros::Time timestamp,
    uint8_t object_class,
    float exist_confidence,
    float class_confidence,
    float att[3],
    Eigen::Vector3d& veh_llh,
    BaseFilter* filter,
    float point4[8]):
timestamp_(timestamp),
object_class_(object_class),
exist_confidence_(exist_confidence),
class_confidence_(class_confidence),
veh_llh_(veh_llh[0], veh_llh[1], veh_llh[2]),
filter_(filter)
{
#ifdef DEBUG_BASEOBJECT
    cout << "BaseObject ctor start" << endl;
#endif

    att_[0] = att[0];
    att_[1] = att[1];
    att_[2] = att[2];

    point4_[0] = point4[0];
    point4_[1] = point4[1];
    point4_[2] = point4[2];
    point4_[3] = point4[3];
    point4_[4] = point4[4];
    point4_[5] = point4[5];
    point4_[6] = point4[6];
    point4_[7] = point4[7];

    sxmin_list_.reserve(20);
    symin_list_.reserve(20);
    vxmin_list_.reserve(20);
    vymin_list_.reserve(20);

    sxmax_list_.reserve(20);
    symax_list_.reserve(20);
    vxmax_list_.reserve(20);
    vymax_list_.reserve(20);

    detal_t_list_.reserve(20);

#ifdef DEBUG_BASEOBJECT
    cout << "BaseObject ctor end" << endl;
#endif
}

BaseObject::~BaseObject()
{
    delete filter_;
}

float BaseObject::getExistConfidence() const
{
    return exist_confidence_;
}

float BaseObject::getClassConfidence() const
{
    return class_confidence_;
}

uint8_t BaseObject::getObjectClass() const
{
    return object_class_;
}

std::vector<float>& BaseObject::getSxMinList()
{
    return sxmin_list_;
}

std::vector<float>& BaseObject::getSyMinList()
{
    return symin_list_;
}

std::vector<float>& BaseObject::getSxMaxList()
{
    return sxmax_list_;
}

std::vector<float>& BaseObject::getSyMaxList()
{
    return symax_list_;
}

std::vector<float>& BaseObject::getVxMinList()
{
    return vxmin_list_;
}

std::vector<float>& BaseObject::getVyMinList()
{
    return vymin_list_;
}

std::vector<float>& BaseObject::getVxMaxList()
{
    return vxmax_list_;
}

std::vector<float>& BaseObject::getVyMaxList()
{
    return vymax_list_;
}

std::vector<float>& BaseObject::getDetalTList()
{
    return detal_t_list_;
}

void BaseObject::setSxMinList(float detal_sxmin)
{
    sxmin_list_.push_back(detal_sxmin);
    if(sxmin_list_.size() > erase_num) 
        sxmin_list_.erase(sxmin_list_.begin());
}

void BaseObject::setSyMinList(float detal_symin)
{
    symin_list_.push_back(detal_symin);
    if(symin_list_.size() > erase_num) 
        symin_list_.erase(symin_list_.begin());
}

void BaseObject::setSxMaxList(float detal_sxmax)
{
    sxmax_list_.push_back(detal_sxmax);
    if(sxmax_list_.size() > erase_num) 
        sxmax_list_.erase(sxmax_list_.begin());
}

void BaseObject::setSyMaxList(float detal_symax)
{
    symax_list_.push_back(detal_symax);
    if(symax_list_.size() > erase_num) 
        symax_list_.erase(symax_list_.begin());
}

void BaseObject::setVxMinList(float detal_vxmin)
{
    vxmin_list_.push_back(detal_vxmin);
    if(vxmin_list_.size() > erase_num) 
        vxmin_list_.erase(vxmin_list_.begin());
}

void BaseObject::setVyMinList(float detal_vymin)
{
    vymin_list_.push_back(detal_vymin);
    if(vymin_list_.size() > erase_num) 
        vymin_list_.erase(vymin_list_.begin());
}

void BaseObject::setVxMaxList(float detal_vxmax)
{
    vxmax_list_.push_back(detal_vxmax);
    if(vxmax_list_.size() > erase_num) 
        vxmax_list_.erase(vxmax_list_.begin());
}

void BaseObject::setVyMaxList(float detal_vymax)
{
    vymax_list_.push_back(detal_vymax);
    if(vymax_list_.size() > erase_num) 
        vymax_list_.erase(vymax_list_.begin());
}

void BaseObject::setDetalTList(float detal_t)
{
    detal_t_list_.push_back(detal_t);
    if(detal_t_list_.size() > erase_num) 
        detal_t_list_.erase(detal_t_list_.begin());
}

void BaseObject::setExistConfidence(float exist_confidence)
{
    exist_confidence_ = exist_confidence;
}

void BaseObject::setClassConfidence(float class_confidence)
{
    class_confidence_ = class_confidence;
}

void BaseObject::setObjectClass(uint8_t object_class)
{
    object_class_ = object_class;
}

float BaseObject::calculateSimilarity(const BaseObject& obj) const
{
    float iou = calculateIOU(filter_->getState(), obj.filter_->getState());

    Eigen::VectorXf new_state = getState();
    Eigen::VectorXf old_state = obj.getState();
    float velocity_similarity = calculateVelocitySimilarity(new_state[4], new_state[5], old_state[4], old_state[5]);

    float result = (iou + 0.2 * velocity_similarity) / 2;

    return result;
}

