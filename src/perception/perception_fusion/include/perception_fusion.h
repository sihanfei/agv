#ifndef _PERCEPTION_FUSION_H_
#define _PERCEPTION_FUSION_H_

#include <map>
#include <vector>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <common_msgs/ObstacleInfo.h>
#include <location_msgs/FusionDataInfo.h>
#include <perception_sensor_msgs/ObjectList.h>
#include <perception_msgs/FusionDataInfo.h>

#include "utils/tools.h"
#include "sensor_object/base_object.h"
#include "associate/base_association.h"

using namespace std;

// #define DEBUG_PERCEPTION_FUSION

class PerceptionFusion
{
public:
    explicit PerceptionFusion(BaseAssociation* base_association, float velocity_threshold, bool is_draw = false, bool global_rviz = true,
    float range_xmax = 5.0, float range_xmin = -5.0, float range_ymax = 15.0, float range_ymin = -15.0,
    float agv_xmax = 1.8, float agv_xmin = -1.8, float agv_ymax = 8.0, float agv_ymin = -8.0);
    ~PerceptionFusion();    

private:
    void callbackLidarFusion(const perception_sensor_msgs::ObjectList::ConstPtr& msg);
    void callbackFusion(const perception_sensor_msgs::ObjectList::ConstPtr& msg, const location_msgs::FusionDataInfo::ConstPtr& location_msg);

    void updateNewObject(vector<BaseObject*>& obj_list);
    void updateAssociatedObject(vector<BaseObject*>& new_obj, Eigen::MatrixXd& matrix);
    void updateUnassociatedObject(vector<BaseObject*>& new_obj, Eigen::MatrixXd& matrix);
    // void publishFusionObject(ros::Time pub_time, bool is_draw, float att[3], int sensor_type);
    // void publishFusionObjectWithCamera(ros::Time pub_time, bool is_draw, float att[3], int sensor_type, vector<BaseObject*> base_object_list);
    // void publishFusionObjectWithCamera(ros::Time pub_time, bool is_draw, float att[3]);
    void publishFusionObjectWithCamera(const ros::Time pub_time, const bool is_draw, const float att[3], const map<uint32_t, BaseObject*> &publish_map);
    // void updateEryuanObject(const perception_sensor_msgs::ObjectList msg_object_list, float att[], Vector3d &veh_llh);

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_obstacle_info_;
    ros::Publisher pub_rviz_bounding_box_;
    ros::Publisher pub_rviz_bounding_box_info_;

    typedef message_filters::sync_policies::ApproximateTime<perception_sensor_msgs::ObjectList, location_msgs::FusionDataInfo> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;

    boost::shared_ptr<Sync> sync_camera_location_;
    boost::shared_ptr<Sync> sync_lidar_location_;
    message_filters::Subscriber<perception_sensor_msgs::ObjectList> sub_camera_;
    message_filters::Subscriber<perception_sensor_msgs::ObjectList> sub_lidar_;
    message_filters::Subscriber<location_msgs::FusionDataInfo> sub_location_;

    ros::Subscriber sub_lidar_no_sync_;

    BaseAssociation* base_association_;
    map<uint32_t, BaseObject*> global_object_;
    static const int eryuan_number = 8;
    map<uint32_t, BaseObject*> eryuan_object_list[eryuan_number];

    uint32_t global_id_ = 0;
    bool is_draw_ = false;

    float veh_vxvy_[2] = {0, 0};
    float velocity_threshold_ = 0.0;
    bool global_rviz_ = true;
    float range_xmax_ = 5.0, range_xmin_ = -5.0, range_ymax_ = 15.0, range_ymin_ = -15.0;
    float agv_xmax_ = 1.8, agv_xmin_ = -1.8, agv_ymax_ = 8.0, agv_ymin_ = -8.0;
    ros::Time lidar_time;
    ros::Time eryuan_time;
    int eryuan_frequency_number = 0;
};

#endif // _PERCEPTION_FUSION_H_