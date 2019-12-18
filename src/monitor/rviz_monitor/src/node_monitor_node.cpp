#include <ros/ros.h>

// #include "glog_helper.h"

#include <iostream>

#include "common_functions.h"
// #include <sys/types.h>

#include <thread>

#include <list>
#include <map>

#include <yaml-cpp/yaml.h>

#include <sys/syscall.h>
#include <unistd.h> //getpid()

#include <vector>

#include "common_msgs/KeyValue.h"
#include "status_msgs/NodeStatus.h"
#include "status_msgs/SafetyStatus.h"

#include "monitor_msgs/PidStatusArray.h"

using namespace std;

#define NODE_NAME "node_monitor_node"

#define CONTI_YAML_FILE_PATH "/work/superg_agv/src/monitor/rviz_monitor/cfg"
#define CONTI_YAML_FILE_NAME "/node_list.yaml"

struct Node_List
{
  std::string node_name;
  bool node_enable;
  int pid;
  ros::Time time;
  int error_heartbeat;
  int heartbeat_pre;
  unsigned long procputime;
};

namespace YAML
{
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template < typename T > void operator>>(const YAML::Node &node, T &i)
{
  i = node.as< T >();
}
} /* YAML */

// now the extraction operators for these types //重载 >> 预算符。。。。
void operator>>(const YAML::Node &node, Node_List &nodelist)
{
  node["node_name"] >> nodelist.node_name;
  node["node_enable"] >> nodelist.node_enable;
}

// g
std::map< string, Node_List > g_node_info_map_;
// ros::NodeHandle g_nh_;
ros::Publisher node_cpu_mem_pub_;
unsigned long totalcputime1, totalcputime2, totalcputime;

void nodeStatusCallback(const status_msgs::NodeStatus::ConstPtr &msg)
{

  // ros::Time t1 = ros::Time::now();
  // ROS_INFO("PID[%ld] state_num[%d]", msg->node_pid, msg->header.seq);

  map< string, Node_List >::iterator it;
  it = g_node_info_map_.find(msg->node_name);
  if (it != g_node_info_map_.end())
  {
    it->second.pid           = msg->node_pid;
    it->second.time          = msg->header.stamp;
    it->second.heartbeat_pre = 0;
  }

  // ros::Time t2    = ros::Time::now();
  // ros::Duration d = t2 - t1;
  // ROS_INFO("[%s]nodeStatusCallback NSec = [%ld]", msg->node_name.c_str(), d.toNSec()); // d.toSec());
}

void timerCallback(const ros::TimerEvent &event)
{
  // ros::Time t1 = ros::Time::now();

  map< string, Node_List >::iterator iter;
  float cpu, mem;
  monitor_msgs::PidStatusArray msg;
  monitor_msgs::PidStatus ps;

  totalcputime2 = get_cpu_total_occupy();
  totalcputime  = totalcputime2 - totalcputime1;
  totalcputime1 = totalcputime2;

  for (iter = g_node_info_map_.begin(); iter != g_node_info_map_.end(); iter++)
  {
    ps.node_name = iter->second.node_name;
    ps.node_pid  = iter->second.pid;
    // ROS_WARN("second.pid[%d]", iter->second.pid);
    if (iter->second.pid == 0)
    {
      ps.cpu_occupy = 0;
      ps.mem_occupy = 0;
    }
    else
    {
      ps.cpu_occupy = get_proc_cpu(iter->second.pid, totalcputime, iter->second.procputime);
      ps.mem_occupy = get_proc_mem(iter->second.pid);
    }
    msg.pid_status.push_back(ps);
  }
  msg.header.stamp = ros::Time::now();
  node_cpu_mem_pub_.publish(msg);

  // ros::Time t2    = ros::Time::now();
  // ros::Duration d = t2 - t1;
  // ROS_INFO("timerCallback NSec = [%ld]", d.toNSec());
}

int main(int argc, char *argv[])
{
  ROS_INFO("ROS node is start, name is [%s], file name is %s", NODE_NAME, argv[0]);
  ros::init(argc, argv, NODE_NAME); // node name
  ros::NodeHandle nh;

  ////////////////////glog
  // GLogHelper gh(argv[0]);
  // gh.setLogDirectory(NODE_NAME);

  //////////////////yaml file
  char *home_path           = getenv("HOME");
  char yaml_file_name[1024] = {0};
  sprintf(yaml_file_name, "%s%s%s", home_path, CONTI_YAML_FILE_PATH, CONTI_YAML_FILE_NAME);
  // SUPERG_INFO << "yaml_file_name:" << yaml_file_name;

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  std::string file_name = yaml_file_name;
  YAML::Node doc;
  doc = YAML::LoadFile(file_name);

  ostringstream log_dir_stream;
  log_dir_stream.fill('0');
  log_dir_stream << home_path << doc["log_dir"].as< string >();

  for (unsigned i = 0; i < doc["node_list"].size(); i++)
  {

    Node_List node_list_;
    doc["node_list"][i] >> node_list_;

    ROS_INFO("[%s]", node_list_.node_name.c_str());

    totalcputime2 = get_cpu_total_occupy();
    totalcputime  = totalcputime2 - totalcputime1;
    totalcputime1 = totalcputime2;

    if (node_list_.node_enable)
    {
      node_list_.pid             = 0;
      node_list_.error_heartbeat = 0;
      node_list_.heartbeat_pre   = 0;
      node_list_.time            = ros::Time::now();
      node_list_.procputime      = get_cpu_proc_occupy(node_list_.pid);

      g_node_info_map_.insert(pair< string, Node_List >(node_list_.node_name, node_list_));
    }
  }

  node_cpu_mem_pub_   = nh.advertise< monitor_msgs::PidStatusArray >("/monitor/pid_status_msg", 1);
  ros::Subscriber sub = nh.subscribe("/node/node_status", 100, nodeStatusCallback);

  ros::Timer timer = nh.createTimer(ros::Duration(1.0), timerCallback);

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    // ROS_WARN("spinOnce end");

    map< string, Node_List >::iterator iter;
    for (iter = g_node_info_map_.begin(); iter != g_node_info_map_.end(); iter++)
    {
    }

    loop_rate.sleep();
  }

  //   ros::spin();
  ros::waitForShutdown();
  return 0;
}