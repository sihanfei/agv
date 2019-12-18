#include "sensor_object/base_object.h"

sensor_camera::BaseObject::BaseObject(
    ros::Time timestamp,
    uint8_t object_class,
    float exist_confidence,
    float class_confidence,
    sensor_camera::BaseFilter* world_filter,
    sensor_camera::BaseFilter* pix_filter,
    float state_3d[6]):
timestamp_(timestamp),
object_class_(object_class),
exist_confidence_(exist_confidence),
class_confidence_(class_confidence),
world_filter_(world_filter),
pix_filter_(pix_filter),
state_3d_(Eigen::VectorXf(6, 1))
{
#ifdef DEBUG_BASEOBJECT
    cout << "BaseObject ctor start" << endl;
#endif

    state_3d_ << state_3d[0] , state_3d[1], state_3d[2], state_3d[3], state_3d[4], state_3d[5];

    sx_list_.reserve(20);
    sy_list_.reserve(20);
    vx_list_.reserve(20);
    vy_list_.reserve(20);

    detal_t_list_.reserve(20);

#ifdef DEBUG_BASEOBJECT
    cout << "BaseObject ctor end" << endl;
#endif
}

sensor_camera::BaseObject::~BaseObject()
{
    delete world_filter_;
    delete pix_filter_;
}

float sensor_camera::BaseObject::getExistConfidence() const
{
    return exist_confidence_;
}

float sensor_camera::BaseObject::getClassConfidence() const
{
    return class_confidence_;
}

uint8_t sensor_camera::BaseObject::getObjectClass() const
{
    return object_class_;
}

std::vector<float>& sensor_camera::BaseObject::getSxList()
{
    return sx_list_;
}

std::vector<float>& sensor_camera::BaseObject::getSyList()
{
    return sy_list_;
}

std::vector<float>& sensor_camera::BaseObject::getVxList()
{
    return vx_list_;
}

std::vector<float>& sensor_camera::BaseObject::getVyList()
{
    return vy_list_;
}

std::vector<float>& sensor_camera::BaseObject::getDetalTList()
{
    return detal_t_list_;
}

Eigen::VectorXf sensor_camera::BaseObject::getState3d()
{
    return state_3d_;
}

void sensor_camera::BaseObject::setSxList(float detal_sx)
{
    sx_list_.push_back(detal_sx);
    if(sx_list_.size() > erase_num) 
        sx_list_.erase(sx_list_.begin());
}

void sensor_camera::BaseObject::setSyList(float detal_sy)
{
    sy_list_.push_back(detal_sy);
    if(sy_list_.size() > erase_num) 
        sy_list_.erase(sy_list_.begin());
}

void sensor_camera::BaseObject::setVxList(float detal_vx)
{
    vx_list_.push_back(detal_vx);
    if(vx_list_.size() > erase_num) 
        vx_list_.erase(vx_list_.begin());
}

void sensor_camera::BaseObject::setVyList(float detal_vy)
{
    vy_list_.push_back(detal_vy);
    if(vy_list_.size() > erase_num) 
        vy_list_.erase(vy_list_.begin());
}

void sensor_camera::BaseObject::setDetalTList(float detal_t)
{
    detal_t_list_.push_back(detal_t);
    if(detal_t_list_.size() > erase_num) 
        detal_t_list_.erase(detal_t_list_.begin());
}

void sensor_camera::BaseObject::setExistConfidence(float exist_confidence)
{
    exist_confidence_ = exist_confidence;
}

void sensor_camera::BaseObject::setClassConfidence(float class_confidence)
{
    class_confidence_ = class_confidence;
}

void sensor_camera::BaseObject::setObjectClass(uint8_t object_class)
{
    object_class_ = object_class;
}

void sensor_camera::BaseObject::setState3d(float state_3d[6])
{
    state_3d_[0] = state_3d[0];
    state_3d_[1] = state_3d[1];
    state_3d_[2] = state_3d[2];
    state_3d_[3] = state_3d[3];
    state_3d_[4] = state_3d[4];
    state_3d_[5] = state_3d[5];
}

void sensor_camera::BaseObject::setWorldState(Eigen::VectorXf& world_state)
{
    world_filter_->setState(world_state);
}

float sensor_camera::BaseObject::calculateSimilarity(sensor_camera::BaseObject& obj)
{
    // =========================== iou ===========================
    float pix_iou = sensor_camera::calculateIOU(pix_filter_->getState(), obj.pix_filter_->getState());
    float world_iou = sensor_camera::calculateIOU(state_3d_, obj.getState3d());
#ifdef DEBUG_BASEOBJECT
    cout << "pix_iou: " << pix_iou << endl;
    cout << "state_3d: " << state_3d_[0] << ", " << state_3d_[1] << ", " << state_3d_[2] << ", " << state_3d_[3] << endl;
    cout << "world_iou: " << world_iou << endl;
#endif

    float iou = pix_iou * world_iou;

    // =========================== velocity similarity ===========================
    bool has_velocity_similarity = false;
    float velocity_similarity = 0;
    if(obj.getVxList().size() > 0)
    {
        vector<float> old_vx_list(obj.getVxList());
        vector<float> old_vy_list(obj.getVyList());

        Eigen::VectorXf new_world_state = world_filter_->getState();
        Eigen::VectorXf old_world_state = obj.world_filter_->getState();
        vector<float> new_sx_list(obj.getSxList());
        vector<float> new_sy_list(obj.getSyList());
        vector<float> new_detal_t_list(obj.getDetalTList());
        new_sx_list.push_back(new_world_state(0) - old_world_state(0));
        new_sy_list.push_back(new_world_state(1) - old_world_state(1));
        new_detal_t_list.push_back(timestamp_.toSec() - obj.timestamp_.toSec());

        pair<float, float> vx_vy = sensor_camera::calculateMeanVxVy(new_sx_list, new_sy_list, new_detal_t_list);
        vector<float> new_vx_list(obj.getVxList());
        vector<float> new_vy_list(obj.getVyList());
        new_vx_list.push_back(vx_vy.first);
        new_vy_list.push_back(vx_vy.second);
        
        velocity_similarity = sensor_camera::calculateVelocitySimilarity(new_vx_list, new_vy_list, old_vx_list, old_vy_list);
#ifdef DEBUG_BASEOBJECT
        cout << "velocity_similarity: " << velocity_similarity << endl;
#endif
        has_velocity_similarity = true;
    }
    
    float result = iou;
    if(has_velocity_similarity)
    {
        result = (iou + velocity_similarity) / 2;
    }
#ifdef DEBUG_BASEOBJECT
    cout << "BaseObject calculateSimilarity end" << endl;
#endif

    return result;
}

