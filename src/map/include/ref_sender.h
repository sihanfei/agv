#ifndef MAP_INCLUDE_REF_SENDER_H_
#define MAP_INCLUDE_REF_SENDER_H_

#include "coordinate.h"
#include "ros/ros.h"
#include <algorithm>
#include <arpa/inet.h>
#include <eigen3/Eigen/Dense>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include "hmi_msgs/ADStatus.h"
#include "hmi_msgs/HMIControlAD.h"
#include "hmi_msgs/VMSControlAD.h"
#include "location_msgs/FusionDataInfo.h"
#include "map_common_utils.h"
#include "map_msgs/REFPointArray.h"
#include "nav_msgs/Path.h"
#include "perception_sensor_msgs/UltrasonicInfo.h"
#include "plan_msgs/DecisionInfo.h"
#include "std_msgs/Int64MultiArray.h"

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "get_ref_line.h"

#define ENABLE_SZ_MAP 0

#define ENABLE_ZJ_NEW_MAP 1
#define ZJ_NEW_MAP_X 0
#define ZJ_NEW_MAP_Y 0

using namespace Eigen;

namespace superg_agv
{
namespace map
{
#define MAP_REF_POINT_CONNECT_PATH "/work/superg_agv/src/data/map_data"

#if ENABLE_SZ_MAP

#define REF_LINE_NAME "/shenzhuan_ref_line_map.json"
#define MAP_CONNECT_NAME "/shenzhuan_connect_map.json"

#define Y_lan0 22.67080000
#define X_lon0 114.33280000
#define H_h0 59.537175
#define SCALE 100000.0

#else

#define REF_LINE_NAME "/ref_line_map.json"
#define MAP_CONNECT_NAME "/connect_map.json"

#endif

#define FRE 1
#define BUF_LEN 10

#define OFFSET_X 18.75
#define OFFSET_Y -0.75
#define GRID_SIZE 0.116

#define MAX_DISTANCE_SQUARE 12 * 12
#define INTERVAL_X 0.5 * GRID_SIZE

class RefSender
{
public:
  RefSender(ros::NodeHandle &nh, const double &point_x_, const double &point_y_, const int &para_tip_);
  virtual ~RefSender();
  void recvClickCallback(const geometry_msgs::PointStampedConstPtr &msg);
  void recvVCUCallback(const location_msgs::FusionDataInfoConstPtr &msg);
  void recvRouteLaneIDArrayCallback(const std_msgs::Int64MultiArrayConstPtr &msg);
  void refSender();
  void routeRefPlanningPub(const std::vector< RefLaneData > &rrps);
  void vcuLocationPub();
  void readRefFromCSV();
  int findLaneIDWithXY(TaskPoint &find_point_in_, int find_mode);
  int mathDistance(const double &x_, const double &y_, const std::vector< std::vector< double > > &ref_line_point_,
                   MinRefPoint &min_rfd);
  double pointDistanceSquare(const double &x1, const double &y1, const double &x2, const double &y2);
  static bool sortFun(const MinRefPoint &d1, const MinRefPoint &d2);
  float getCross(const Point &p1, const Point &p2, const Point &p);
  bool isPointInMatrix(const MinRefPoint &rfd, const double &x_, const double &y_);
  int findLaneRefFromCSV(const int &lid, std::vector< RefLaneData > &route_rfd, double &sum_s_);
  void vectorToRefLaneData(const std::vector< double > &ref_lp_data_, RefLaneData &rfd_, double &ss_);

  void rvizAllPointPub();
  void rvizRouteRefPointPub(const std::vector< RefLaneData > &rrps);
  void findLaneIDFromRoute(const RouteData &rd_, const TaskPoint &tp_, const int spt_, int &last_, int &now_,
                           int &next_);

  int getPointFromRoute(const RouteData &rd_, const TaskPoint &vlp_, const std::vector< RefLaneData > &trrd_,
                        std::vector< RefLaneData > &trrd_t_, int &last_, int &now_, int &next_, const int &last_ret_);
  void taskPointPub(const TaskPoint &t_point_);
  double angleChange(const double theta_in_);
  void recvUltraInfoCallback(const perception_sensor_msgs::UltrasonicInfoConstPtr &msg);
  void recvHMIControlADCallback(const hmi_msgs::HMIControlADConstPtr &msg);
  void recvADStatusCallback(const hmi_msgs::ADStatusConstPtr &msg);
  void refSenderInit();
  void taskPointFind();
  void vcuPointFind();
  void routePointFind();
  void hmiReset();

  void rvizAllMapAppendixAttributePub();
  void rvizAllPointPubBoundrayPoints();
  void readRefFromGetRefLine();
  void rvizAllPointPubNewMap();
  void setRefLineInit();
  int findLaneRefFromMap(const int &l_id_, std::vector< RefLaneData > &route_rfd, double &sum_s_);
  void translateRefPointsToRefLaneData(RefPoints &rp_, RefLaneData &rfd_, double &ss_);
  void ultraPointPub();
  void rvizBJPointPubNewMap();
  void rvizVCUPointPubNewMap();
  void rvizVcuPointPubTest();
  void recvFusionLocationCallback(const location_msgs::FusionDataInfoConstPtr &msg);
  void rvizColorTest();
  void routeDecisionInfoPub(const std::vector< RefLaneData > &rrps);

  void addExtraVirtualRefLaneData(RefLaneData &rfd_in_, RefLaneData &rfd_, double &begin_ss, double &add_ss_,
                                  int index_);
  int addBeforeBeginRef(const int &l_id_, std::vector< RefLaneData > &route_rfd, double &sum_s_);
  int addAfterEndRef(const int &l_id_, std::vector< RefLaneData > &route_rfd, double &sum_s_);

private:
  ros::NodeHandle nh_;
  ros::Subscriber task_click_sub_;         //点击反馈点
  ros::Subscriber vcu_locatio_sub_;        //车辆实时坐标
  ros::Subscriber route_laneID_array_sub_; //全局规划路段id
  ros::Subscriber hmi_control_sub_;
  ros::Subscriber ultra_info_sub_;
  ros::Subscriber ad_status_sub_;
  ros::Subscriber location_fusion_sub_;

  ros::Publisher rviz_color_test_pub_;
  ros::Publisher rviz_point_pub_;
  ros::Publisher rviz_vcu_point_pub_;
  ros::Publisher rviz_bj_point_pub_;
  ros::Publisher rviz_all_point_pub_;
  ros::Publisher rviz_route_ref_pub_;
  ros::Publisher vms_info_pub_;
  ros::Publisher route_ref_planning_pub_;         //局部路径参考点
  ros::Publisher vcu_location_pub_;               //发布带道路id的vcu坐标
  ros::Publisher route_decision_info_pub_;        //参考线转局部规划路径格式直接发给控制
  ros::Publisher rviz_appendix_attributemap_pub_; //参考线转局部规划路径格式直接发给控制

  std::map< int, std::vector< std::vector< double > > > ref_line_data;

  int route_recv_tip;
  int vcu_location_recv_tip;
  int task_click_recv_tip;
  int get_csv_tip;
  int find_vcu_laneID_tip;
  int find_laneID_ref_tip;
  int send_planing_;
  int send_planing_first_;
  int last_id_;
  int next_id_;
  int now_id_;
  int route_last_ret_;
  uint8_t hmi_cmd_type_;
  int hmi_cmd_type_recv_tip;
  uint8_t hmi_cmd_num_;
  int ultra_status_;
  int ultra_recv_tip;
  int running_status_;
  int ad_status_recv_tip;

  TaskPoint task_click_;
  TaskPoint vcu_location_;
  TaskPoint vcu_location_temp_;
  std::vector< TaskPoint > vcu_location_vec;

  RouteData route_data_;
  std::vector< RefLaneData > task_route_ref_data;
  std::vector< UltrasonicStatus > ultra_point_vec;

  GetRefLine *get_ref_line;
  std::map< int, RefLine > ref_line_map;
  std::map< int, std::vector< RefPoints > > ref_points_map;
  std::map< int, AppendixAttribute > appendix_attribute_map;
  std::vector< RefPoints > ref_points_vec;

  std::map< int, BoundrayLine > boundray_line_map;
  std::map< int, BoundrayCircle > boundray_circle_map;
  std::vector< BoundrayPoint > boundray_points_vec;

  std::map< int, RefLine > bj_line_map;
  std::map< int, std::vector< RefPoints > > bj_points_map;
  std::vector< RefPoints > bj_points_vec;
};

} // namespace map
} // namespace superg_agv
#endif