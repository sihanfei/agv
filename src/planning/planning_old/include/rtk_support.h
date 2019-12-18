#ifndef RTK_SUPPORT_H
#define RTK_SUPPORT_H

#include "ros/ros.h"

#include <perception_msgs/FusionDataInfo.h>
#include <location_msgs/FusionDataInfo.h>
#include "status_msgs/NodeStatus.h"
#include <plan_msgs/DecisionInfo.h>
#include <map_msgs/REFPointArray.h>
#include <control_msgs/AGVStatus.h>

#include "common/reference_line.h"
#include "common/cartesian_point.h"
#include "common/frenet_point.h"
#include "common/perception_info.h"
#include "math/cartesian_frenet_converter.h"
#include "common/vcu_status.h"

#include "common/utils.h"

namespace planning
{

class RTKSupport
{

public:
    RTKSupport(ros::NodeHandle& n);
    RTKSupport() = default;
    ~RTKSupport() = default;

private:

    ros::NodeHandle n_;

    ros::Subscriber perception_sub_;
    ros::Subscriber location_sub_;

    ros::Subscriber ref_line_sub_;

    ros::Subscriber vcu_sub_;

    ros::Publisher node_status_pub_;

    ros::Publisher decision_pub_;

    void startRTKSupport();

    void publishEstopPlanningMsg();
    void publishErrorStatusMsg(std::string number);

    void perceptionCallback(const perception_msgs::FusionDataInfo::ConstPtr& perception_msg);

    void locationCallback(const location_msgs::FusionDataInfo::ConstPtr& location_msg);

    void refLineCallback(const map_msgs::REFPointArray::ConstPtr& ref_line_msg);
    void vcuCallback(const control_msgs::AGVStatus::ConstPtr& vcu_msg);

    CartesianPoint carlocation_;//笛卡尔定位点
    CartesianPoint cargoal_;//笛卡尔目标点
    CartesianPoint last_carlocation_;//上一帧笛卡尔定位点

    FrenetPoint fre_location_;//Frenet定位点
    FrenetPoint fregoal_;//Frenet目标点

    ReferenceLine ref_line_;//参考线

    PerceptionInfo envi_info_;//感知

    VCUStatus vcu_info_;

    bool is_first_location_;
    bool is_ref_line_read_finished_;

    bool is_location_info_finished_;
    bool is_perception_info_finished_;
    bool is_control_info_finished_;

    bool is_vcu_info_finished_;

    uint32_t location_seq_;
    uint32_t perception_seq_;
    uint32_t control_seq_;

    uint32_t cmd_seq_;

    uint32_t vcu_seq_;

    uint32_t location_mismatch_count_;
    uint32_t perception_mismatch_count_;
    uint32_t control_mismatch_count_;

    uint32_t vcu_mismatch_count_;

};



} // end namespace


#endif // RTK_SUPPORT_H
