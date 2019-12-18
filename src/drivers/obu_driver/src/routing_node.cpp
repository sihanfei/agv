#include <ros/ros.h>

#include "glog_helper.h"
#include "tcp_process.h"
#include <iostream>

#include "common_functions.h"
// #include <sys/types.h>

#include <boost/predef/other/endian.h>
#include <boost/thread.hpp>

#include <thread>

#include <list>
#include <map>

#include <yaml-cpp/yaml.h>

#include <sys/syscall.h>
#include <unistd.h> //getpid()

#include <pthread.h>

#include "common_msgs/KeyValue.h"
#include "status_msgs/NodeStatus.h"
#include "status_msgs/SafetyStatus.h"

#include "location_sensor_msgs/FixedLidarInfo.h"

using namespace std;

#define NODE_NAME "routing_node"

#define routing_YAML_FILE_PATH "/work/superg_agv/src/drivers/obu_driver/cfg"
#define routing_YAML_FILE_NAME "/routing.yaml"

struct Obu_Device
{
  std::string device_name;
  bool device_enable;
  std::string device_ip;
  int intput_port;
  int isConn;
  boost::shared_ptr< TcpProcess > tcp_process;
};

Obu_Device obu_device_;
uint16_t g_agv_id;
double g_line_heading; //                               #车道线方向角

//定义线程锁
pthread_mutex_t list_mutex;

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
void operator>>(const YAML::Node &node, Obu_Device &obuDevice)
{
  node["device_name"] >> obuDevice.device_name;
  node["device_enable"] >> obuDevice.device_enable;
  node["device_ip"] >> obuDevice.device_ip;
  node["intput_port"] >> obuDevice.intput_port;
}

class RoutingTcpProcess : public TcpProcessCallBack
{
public:
  RoutingTcpProcess();
  ~RoutingTcpProcess();
  void OnTcpProcessCallBack(unsigned char *data, int len, struct sockaddr_in addr_);

  void OnTcpStatusCallBack(int istatus, struct sockaddr_in addr_);

  void status_Sender();

  void start();

  list< location_sensor_msgs::FixedLidarInfo > list_fixedLidar_info_;

  // ros::Publisher Fixed_Lidar_msg_pub_; //    = nh.advertise< location_sersor_msgs::FixedLidarInfo >(topic1_.data(),
  // 2);
};

RoutingTcpProcess::RoutingTcpProcess()
{
  ROS_INFO("RoutingTcpProcess process[%d] thread[%u]", getpid(), ( unsigned int )pthread_self());
}
RoutingTcpProcess::~RoutingTcpProcess()
{
  // udp_.reset();
}

void RoutingTcpProcess::OnTcpStatusCallBack(int istatus, struct sockaddr_in addr_)
{
  //申请线程锁
  pthread_mutex_lock(&list_mutex);
  // ROS_INFO("ip[%s] port[%d] istatus = %d", inet_ntoa(addr_.sin_addr), htons(addr_.sin_port), istatus);
  // ostringstream map_key;

  // map_key.str("");
  // map_key << inet_ntoa(addr_.sin_addr) << ":" << htons(addr_.sin_port);
  // map< string, Obu_Device >::iterator status_it_;
  // status_it_ = g_map_obu_status.find(map_key.str());
  // if (status_it_ != g_map_obu_status.end())
  //   status_it_->second.isConn = istatus;
  obu_device_.isConn = istatus;
  //释放线程锁
  pthread_mutex_unlock(&list_mutex);
}

void RoutingTcpProcess::OnTcpProcessCallBack(unsigned char *data, int len, struct sockaddr_in addr_)
{
  ostringstream ost;
  data[len] = 0x00;

  ost << data;
  ROS_INFO("v_string[%s]", ost.str().c_str());
  //分割字符串
  std::vector< std::string > v_string_ = string_split(ost.str(), ",");
}
void RoutingTcpProcess::status_Sender()
{

  ROS_INFO("process[%d] thread[%lu] status_Sender Start!", getpid(), pthread_self());

  ros::NodeHandle nh;

  ros::Rate loop_rate(100);
  while (ros::ok())
  {

    //申请线程锁
    pthread_mutex_lock(&list_mutex);
    // ROS_INFO("publish...");

    //释放线程锁
    pthread_mutex_unlock(&list_mutex);

    loop_rate.sleep();

  } // while (ros::ok())
}

void RoutingTcpProcess::start()
{
  // udp_->Initial(boost::shared_ptr< RadarDataProcess >(this), device_ip_, intput_port_, 13);

  boost::thread thrd(boost::bind(&RoutingTcpProcess::status_Sender, this));
  thrd.detach();
}

int main(int argc, char *argv[])
{

  ROS_INFO("ROS node is start, name is [%s], file name is %s", NODE_NAME, argv[0]);
  ros::init(argc, argv, NODE_NAME); // node name
  ros::NodeHandle nh;

  ////////////////////glog
  //   GLogHelper gh(argv[0]);
  //   gh.setLogDirectory(argv[0]);

  //初始化线程锁
  pthread_mutex_init(&list_mutex, NULL);

  //////////////////yaml file
  char *home_path           = getenv("HOME");
  char yaml_file_name[1024] = {0};
  sprintf(yaml_file_name, "%s%s%s", home_path, routing_YAML_FILE_PATH, routing_YAML_FILE_NAME);
  SUPERG_INFO << "yaml_file_name:" << yaml_file_name;

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  std::string file_name = yaml_file_name;
  YAML::Node yaml_doc;
  yaml_doc = YAML::LoadFile(file_name);

  ostringstream log_dir_stream;
  log_dir_stream.fill('0');
  log_dir_stream << home_path << yaml_doc["log_dir"].as< string >();

  // map
  ostringstream map_key;
  for (unsigned i = 0; i < yaml_doc["device_list"].size(); i++)
  {
    //    Obu_Device obu_device_;
    yaml_doc["device_list"][i] >> obu_device_;
    obu_device_.isConn = 0;
    // if (obu_device_.device_enable)
    // {
    //   map_key.str("");
    //   map_key << obu_device_.device_ip << ":" << obu_device_.intput_port;
    //   g_map_obu_status[map_key.str()] = obu_device_;
    // }
  }
  //  data process class
  boost::shared_ptr< RoutingTcpProcess > routing_tcp_(new RoutingTcpProcess());
  routing_tcp_->start();
  ros::Rate loop_rate(0.1);
  int sec_count         = 0;
  int node_status_count = 0;
  uint16_t count        = 0;
  while (ros::ok())
  {
    ros::Time t1 = ros::Time::now();
    sec_count++;
    count++;
    node_status_count++;
    if (sec_count > 1)
    {
      sec_count = 0;
      if (obu_device_.isConn == 0)
      {
        obu_device_.tcp_process.reset(new TcpProcess());
        obu_device_.tcp_process->log_dir_     = log_dir_stream.str();
        obu_device_.tcp_process->sensor_name_ = yaml_doc["sensor_name"].as< string >();
        obu_device_.tcp_process->device_name_ = obu_device_.device_name;
        obu_device_.isConn =
            obu_device_.tcp_process->Initial(routing_tcp_, obu_device_.device_ip, obu_device_.intput_port, 1024);
      }
    }
    if (obu_device_.isConn != 0)
    {
      std::string sendbuf;
      std::stringstream ss;
      double cuv       = 3.0;
      double high      = 15.5;
      int vcu_location = 17;
      int task_click   = 25;
      ss << 40 << "," << high << "," << cuv << "," << vcu_location << "," << task_click;
      sendbuf = ss.str();

      ROS_INFO("sendTcpInfo"); // d.toSec());
      obu_device_.tcp_process->sendTcpInfo(( unsigned char * )sendbuf.c_str(), sendbuf.length());
    }

    ros::Time t2    = ros::Time::now();
    ros::Duration d = t2 - t1;
    // ROS_INFO("[%d]node  [%ld]NSec", sec_count, d.toNSec()); // d.toSec());
    ros::spinOnce();
    loop_rate.sleep();
  }

  //销毁线程锁
  pthread_mutex_destroy(&list_mutex);
  return 0;
}