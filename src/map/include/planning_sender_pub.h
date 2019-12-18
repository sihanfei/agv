#ifndef MAP_INCLUDE_PLANNING_SENDER_PUB_H_
#define MAP_INCLUDE_PLANNING_SENDER_PUB_H_

#include "hmi_msgs/HMIControlAD.h"
#include "hmi_msgs/VMSControlAD.h"
#include "map_common_utils.h"
#include "novalet_gps/NovatelPosition.h"
#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/String.h"
#include <arpa/inet.h>
#include <fcntl.h>
#include <geometry_msgs/PointStamped.h>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <sys/socket.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace superg_agv
{
namespace map
{
class TaskSender
{
public:
  TaskSender(ros::NodeHandle &nh, int &s_laneID, int &e_laneID);
  virtual ~TaskSender();
  void recvClickCallback(const geometry_msgs::PointStampedConstPtr &msg);
  void recvVCULocationCallback(const geometry_msgs::PointStampedConstPtr &msg);
  int routeLaneIDArrayPub();
  void taskSender();
  int socketLink();
  int socketSendAndRecv(std::string &data_buf_);
  void socketClose();
  int getRouteDataFromString(std::string &data_buf_);

private:
  ros::NodeHandle nh_;
  ros::Subscriber task_click_sub_;
  ros::Subscriber vcu_locatio_sub_;
  ros::Publisher route_laneID_array_pub_;

  int task_click_recv_tip;
  int vcu_location_recv_tip;
  int route_laneID_pub_tip;
  int first_position_tip;

  int sockfd;

  int vcu_ID_;
  int task_laneID_;
  int vcu_laneID_;

  superg_agv::map::TaskPoint task_click;
  superg_agv::map::TaskPoint vcu_location;
  superg_agv::map::TaskPoint task_click_last_;
  superg_agv::map::TaskPoint vcu_location_last_;
  superg_agv::map::RouteData route_data;
};

} // namespace map
} // namespace superg_agv
#endif