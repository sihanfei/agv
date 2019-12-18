#ifndef DECISION_H
#define DECISION_H

#include <ros/ros.h>

#include <hmi_msgs/VMSControlAD.h>
#include <hmi_msgs/ADStatus.h>

#include <perception_msgs/FusionDataInfo.h>
#include <location_msgs/FusionDataInfo.h>

#include <map_msgs/REFPointArray.h>

#include <plan_msgs/DecisionInfo.h>

#include <control_msgs/AGVRunningStatus.h>

#include <control_msgs/AGVStatus.h>

#include "common/control_info.h"
#include "common/perception_info.h"
#include "common/reference_line.h"

#include "status_msgs/NodeStatus.h"
#include "math/cartesian_frenet_converter.h"
#include "common/get_boundary_by_cartesian_location.h"
#include "common/trajectory.h"
#include "common/vcu_status.h"

namespace planning
{

enum decision//决策枚举
{Ignore,Stop,LeftNudge,RightNudge,Follow,Yield,OverTake,Emergency};

class Decision
{
public:
    Decision();
    ~Decision()=default;

    Decision(ros::NodeHandle& n);

    void reset();

    bool setGoal(CartesianPoint& goal);
    FrenetPoint getFreGoal() const;
    CartesianPoint getCarGoal() const;

    bool isObstaclesSafe(FrenetPoint& frelocation);
    uint8_t checkVCU();
    VCUStatus getVCUInfo();

    double getObstacleStopDistance(FrenetPoint& frelocation);

    void perceptionCallback(const perception_msgs::FusionDataInfo::ConstPtr& perception_msg);
    void locationCallback(const location_msgs::FusionDataInfo::ConstPtr& location_msg);
    void controlCallback(const control_msgs::AGVRunningStatus::ConstPtr& control_msg);
    void refLineCallback(const map_msgs::REFPointArray::ConstPtr& ref_line_msg);

    void vcuCallback(const control_msgs::AGVStatus::ConstPtr& vcu_msg);

    bool getPlanningStartPoint(FrenetPoint& fre_planning_start_point, Trajectory& last_trajectory) const;
    bool getPlanningStopPoint(FrenetPoint& fre_planning_stop_point) const;

    decision makeDecision(FrenetPoint& fre_location);

    uint8_t getCarLocation(CartesianPoint& car_point) const;
    uint8_t getFreLocation(FrenetPoint& fre_location) const;

    uint8_t getReferenceLine(ReferenceLine& ref_line) const;
    uint8_t getReferenceLineOnce(ReferenceLine& ref_line) const;

    uint8_t getPerception(PerceptionInfo& per_info) const;
    uint8_t getControl(ControlInfo& control_info) const;

private:

    ros::NodeHandle n_;

    CartesianPoint carlocation_;//笛卡尔定位点
    CartesianPoint cargoal_;//笛卡尔目标点
    CartesianPoint last_carlocation_;//上一帧笛卡尔定位点

    FrenetPoint fre_location_;//Frenet定位点
    FrenetPoint fregoal_;//Frenet目标点

    ReferenceLine ref_line_;//参考线

    PerceptionInfo envi_info_;//感知
    ControlInfo control_info_;//控制

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

    //uint32_t cmd_seq_;

    double stop_dis_;//停车距离，返回规划停车点时使用

};
}


#endif // DECISION_H
