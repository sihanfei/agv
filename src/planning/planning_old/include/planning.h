
#ifndef PLANNING_H
#define PLANNING_H

#include "ros/ros.h"

//#include "std_msgs/String.h"
//#include "geometry_msgs/Twist.h"
//#include "std_msgs/UInt8.h"
//#include "std_msgs/UInt16.h"
//#include "std_msgs/UInt32.h"
//#include "std_msgs/Int16.h"

// for test
//#include "geometry_msgs/Polygon.h"
//#include "nav_msgs/Path.h"
//#include "geometry_msgs/PolygonStamped.h"

//#include <hmi_msgs/ADStatus.h>
//#include <hmi_msgs/VMSControlAD.h>

//#include <location_msgs/FusionDataInfo.h>
//#include <perception_msgs/FusionDataInfo.h>

//#include <map_msgs/REFPointArray.h>

//#include <plan_msgs/DecisionInfo.h>

#include <visualization_msgs/MarkerArray.h>

//#include "Perception_Environment.h"
#include "lattice/lattice_planner.h"
#include "decision.h"
//#include "math/cartesian_frenet_converter.h"

#include "planning/params_1_Config.h"
#include "planning/params_2_Config.h"
#include <dynamic_reconfigure/server.h>

namespace planning
{

class Planning
{

public:
    Planning(ros::NodeHandle& n, ros::NodeHandle& private_nh1, ros::NodeHandle& private_nh2);
    ~Planning();

    void Paramscallback_1(planning::params_1_Config &config, uint32_t level);
    void Paramscallback_2(planning::params_2_Config &config, uint32_t level);

    boost::shared_ptr< dynamic_reconfigure::Server< planning::params_1_Config > > srv1_;
    boost::shared_ptr< dynamic_reconfigure::Server< planning::params_2_Config > > srv2_;

private:

    ros::NodeHandle n_;

    ros::Subscriber vms_info_sub_;
    ros::Subscriber perception_sub_;
    ros::Subscriber location_sub_;
    ros::Subscriber control_sub_;

    ros::Subscriber ref_line_sub_;

    ros::Subscriber vcu_sub_;

    ros::Publisher decision_pub_;
    ros::Publisher adstatus_pub_;

    ros::Publisher node_status_pub_;

    ros::Publisher rviz_path_decision_pub_;

    LatticePlanner *lp_ptr_;
    Decision *decision_ptr_;

    Trajectory last_best_trajectory_; //上一次规划发布轨迹

    void executeCb(const hmi_msgs::VMSControlAD::ConstPtr& cmd_msg);

    void startPlanning();

    void statusPublisher();

    void publishEstopPlanningMsg();
    void publishPlanningMsg(const Trajectory &best_trajectory);

    void publishErrorStatusMsg(std::string number);

    //  int8_t running_status_;
    //  uint8_t task_status_;
    //  uint8_t task_ID_;

    CartesianPoint cargoal_;
    CartesianPoint last_cargoal_;
    uint8_t cmd_type_;
    bool start_planning_flag_;
    bool running_task_end_flag_;

    std::string node_file_name_;
    uint64_t node_pid_;

    std::string planning_task_ID_;
    std::string planning_task_status_;

    uint64_t counter_;

    status_msgs::NodeStatus node_status_;
};

//for score debug
//old score
double WEIGHT_TARGET_SPEED;//速度权重
double WEIGHT_DIST_TRAVELLED;//距离权重
//const double WEIGHT_TIME_LEN = 5.0;
//longitudinal jerk
double LONGITUDINAL_JERK_UPPER_BOUND;

//longitudinal collision
double LON_COLLISION_COST_STD;
double LON_COLLISION_YIELD_BUFFER;
double LON_COLLISION_OVERTAKE_BUFFER;
//centripetal acceleration

//the offset of the center line of the lane
double LAT_OFFSET_BOUND;
double WEIGHT_OPPOSITE_SIDE_OFFSET;
double WEIGHT_SAME_SIDE_OFFSET;

//lateral acceleration

//weight of every cost
double WEIGHT_LON_OBJECTIVE;
double WEIGHT_LON_COMFORT;
double WEIGHT_LON_COLLISION;
double WEIGHT_LAT_OFFSET;
double WEIGHT_LAT_COMFORT;
double WEIGHT_CENTRIPETAL_ACCELERATION;

//new score
double WEIGHT_LAT_JERK; // 横向规划时，Jerk的权重系数
double WEIGHT_LAT_S_LEN; // 横向规划时，S的权重系数
double WEIGHT_LAT_L_OFFSET; // 横向规划时，D的权重系数

double WEIGHT_LON_JERK; // 纵向规划时，Jerk的权重系数
double WEIGHT_LON_T_SPEN; // 纵向规划时，T的权重系数
double WEIGHT_LON_V_DIF; // 纵向规划时，D或V的权重系数

//add by rain.wei 2019-6-25
double WEIGHT_LON_STOP_DIS_TO_GOAL;
double WEIGHT_LON_STOP_END_V;
//----

double WEIGHT_TOTAL_LAT; // 横向轨迹的权重系数
double WEIGHT_TOTAL_LON; // 纵向轨迹的权重系数

//std::vector<Trajectory> all_traj_for_show;

} // end namespace

#endif // PLANNING_H
