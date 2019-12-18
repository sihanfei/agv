

#include "ros/ros.h"
#include "rviz_monitor/HmiControlADConfig.h"
#include <dynamic_reconfigure/server.h>

#include "hmi_msgs/HMIControlAD.h"
#include "std_msgs/String.h"

using namespace std;
#define NODE_NAME "hmi_control_node"

class HmiControlClass
{

public:
  HmiControlClass(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~HmiControlClass()
  {
  }
  ros::Publisher hmi_control_info_pub_;

  ros::Publisher pub;

  ros::NodeHandle private_nh_;

  std_msgs::String str;

  /// Callback for dynamic reconfigure
  void dynamicReconfigcallback(rviz_monitor::HmiControlADConfig &config, uint32_t level);

  boost::shared_ptr< dynamic_reconfigure::Server< rviz_monitor::HmiControlADConfig > > srv_;
};

HmiControlClass::HmiControlClass(ros::NodeHandle node, ros::NodeHandle private_nh) : private_nh_(private_nh)
{
  ROS_INFO_STREAM("node: [" << node.getNamespace() << "] [private_nh=" << private_nh_.getNamespace() << "]");
  // Initialize dynamic reconfigure
  dynamic_reconfigure::Server< rviz_monitor::HmiControlADConfig >::CallbackType f;
  srv_ = boost::make_shared< dynamic_reconfigure::Server< rviz_monitor::HmiControlADConfig > >(private_nh_);

  f = boost::bind(&HmiControlClass::dynamicReconfigcallback, this, _1, _2);
  srv_->setCallback(f); // Set callback function und call initially

  hmi_control_info_pub_ = private_nh.advertise< hmi_msgs::HMIControlAD >("/monitor/hmi_control_ad", 10);
}

void HmiControlClass::dynamicReconfigcallback(rviz_monitor::HmiControlADConfig &config, uint32_t level)
{
  ROS_INFO("control command: %d", config.hmi_control);

  if (hmi_control_info_pub_.getNumSubscribers() == 0) // no one listening?
    return;
  hmi_msgs::HMIControlADPtr hmi_control_msg(new hmi_msgs::HMIControlAD);
  hmi_control_msg->header.frame_id = "/odom";
  hmi_control_msg->header.stamp    = ros::Time::now();
  hmi_control_msg->CMD_Type        = config.hmi_control;

  hmi_control_info_pub_.publish(hmi_control_msg);
}

int main(int argc, char *argv[])
{
  ROS_INFO("ROS node is star, name is [%s], file name is %s", NODE_NAME, argv[0]);
  ros::init(argc, argv, NODE_NAME);

  ros::NodeHandle n;
  ros::NodeHandle private_nh("/monitor/hmi_control");
  ros::Rate loop_rate(100);

  HmiControlClass hmi_control_info_(n, private_nh);

  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}