#include <ros/ros.h>

// #include "glog_helper.h"
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

#define NODE_NAME "fixed_lidar_info_node"

#define DSRC_YAML_FILE_PATH "/work/superg_agv/src/drivers/obu_driver/cfg"
#define DSRC_YAML_FILE_NAME "/config.yaml"

struct Obu_Device
{
  std::string device_name;
  bool device_enable;
  std::string device_ip;
  int intput_port;
  int isConn;
  boost::shared_ptr< TcpProcess > tcp_process;
};

// map< string, Obu_Device > g_map_obu_status;

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

class ObuTcpProcess : public TcpProcessCallBack
{
public:
  ObuTcpProcess();
  ~ObuTcpProcess();
  void OnTcpProcessCallBack(unsigned char *data, int len, struct sockaddr_in addr_);

  void OnTcpStatusCallBack(int istatus, struct sockaddr_in addr_);

  void status_Sender();

  void start();

  list< location_sensor_msgs::FixedLidarInfo > list_fixedLidar_info_;

  std::string last_msec_;

  float temp_y_;

  bool is_;

  // ros::Publisher Fixed_Lidar_msg_pub_; //    = nh.advertise< location_sersor_msgs::FixedLidarInfo >(topic1_.data(),
  // 2);
};

ObuTcpProcess::ObuTcpProcess()
{
  last_msec_ = "";
  temp_y_    = -1.50;
  is_        = 1;
  // ROS_INFO("ObuTcpProcess process[%d] thread[%u]", getpid(), ( unsigned int )pthread_self());
}
ObuTcpProcess::~ObuTcpProcess()
{
  // udp_.reset();
}

void ObuTcpProcess::OnTcpStatusCallBack(int istatus, struct sockaddr_in addr_)
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

void ObuTcpProcess::OnTcpProcessCallBack(unsigned char *data, int len, struct sockaddr_in addr_)
{
  ostringstream ost;
  data[len] = 0x00;

  ost << data;
  //分割字符串
  std::vector< std::string > v_string_ = string_split(ost.str(), ",");
  // ROS_INFO("v_string_.size()[%zu][%s]", v_string_.size(), ost.str().c_str());
  if (v_string_.size() > 18)
  {
    if (atoi(v_string_[1].c_str()) == 1 && last_msec_ != v_string_[10])
    {
      last_msec_ = v_string_[10];

      // if (temp_y_ > 1.50)
      // {
      //   is_ = 0;
      // }
      // if (temp_y_ < -1.50)
      // {
      //   is_ = 1;
      // }
      // if (is_ == 1)
      //   temp_y_ += 0.01;
      // else
      //   temp_y_ -= 0.01;

      location_sensor_msgs::FixedLidarInfo flidar_info;
      flidar_info.header.frame_id = "/odom";
      flidar_info.header.stamp    = ros::Time::now();
      flidar_info.flidar_id       = atoi(v_string_[2].c_str());
      flidar_info.AGV_id          = g_agv_id;
      flidar_info.year            = atoi(v_string_[4].c_str());
      flidar_info.month           = atoi(v_string_[5].c_str());
      flidar_info.day             = atoi(v_string_[6].c_str());
      flidar_info.hour            = atoi(v_string_[7].c_str());
      flidar_info.min             = atoi(v_string_[8].c_str());
      flidar_info.sec             = atoi(v_string_[9].c_str());
      flidar_info.msec            = atof(v_string_[10].c_str()) * 100;

      flidar_info.tager_position.x = atof(v_string_[11].c_str());
      flidar_info.tager_position.y = atof(v_string_[12].c_str());
      flidar_info.tager_position.z = atof(v_string_[13].c_str());

      flidar_info.agv_distance.x = atof(v_string_[14].c_str());
      flidar_info.agv_distance.y = atof(v_string_[15].c_str());
      // flidar_info.agv_distance.y = temp_y_;

      flidar_info.heading = atof(v_string_[16].c_str());
      flidar_info.Vx      = atof(v_string_[17].c_str());
      flidar_info.Vy      = atof(v_string_[18].c_str());

      flidar_info.line_heading = g_line_heading;

      // ROS_INFO("list_fixedLidar_info_.push_back");
      //申请线程锁
      pthread_mutex_lock(&list_mutex);
      list_fixedLidar_info_.push_back(flidar_info);
      //释放线程锁
      pthread_mutex_unlock(&list_mutex);
    }
  }
}
void ObuTcpProcess::status_Sender()
{
  // ROS_INFO("status_Sender:int %d main process, the tid=%lu,pid=%ld", getpid(), pthread_self(), syscall(SYS_gettid));
  ROS_INFO("process[%d] thread[%lu] status_Sender Start!", getpid(), pthread_self());

  ros::NodeHandle nh;
  ros::Publisher Fixed_Lidar_msg_pub_ =
      nh.advertise< location_sensor_msgs::FixedLidarInfo >("/drivers/localization/fixed_lidar_msg", 1);
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    if (!list_fixedLidar_info_.empty())
    {

      //申请线程锁
      pthread_mutex_lock(&list_mutex);
      // ROS_INFO("publish...");
      Fixed_Lidar_msg_pub_.publish(list_fixedLidar_info_.front());
      list_fixedLidar_info_.pop_front();
      //释放线程锁
      pthread_mutex_unlock(&list_mutex);
    }
    loop_rate.sleep();

  } // while (ros::ok())
}

void ObuTcpProcess::start()
{
  // udp_->Initial(boost::shared_ptr< RadarDataProcess >(this), device_ip_, intput_port_, 13);

  boost::thread thrd(boost::bind(&ObuTcpProcess::status_Sender, this));
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
  sprintf(yaml_file_name, "%s%s%s", home_path, DSRC_YAML_FILE_PATH, DSRC_YAML_FILE_NAME);
  // SUPERG_INFO << "yaml_file_name:" << yaml_file_name;

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  std::string file_name = yaml_file_name;
  YAML::Node yaml_doc;
  yaml_doc = YAML::LoadFile(file_name);
  // SUPERG_INFO << "config_num: " << doc.size();
  // SUPERG_INFO << "doc[sensor_name]: " << doc["sensor_name"];

  // SUPERG_INFO << "doc[device_list]: " << doc["device_list"].size();

  ostringstream log_dir_stream;
  log_dir_stream.fill('0');
  log_dir_stream << home_path << yaml_doc["log_dir"].as< string >();

  // agv_id
  g_agv_id = yaml_doc["AGV_ID"].as< uint16_t >();

  g_line_heading = yaml_doc["line_heading"].as< double >();

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
  boost::shared_ptr< ObuTcpProcess > obu_status_(new ObuTcpProcess());
  obu_status_->start();

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  ros::Publisher pub_ = nh.advertise< status_msgs::NodeStatus >("/node/node_status", 10);

  ros::Rate loop_rate(25);
  int sec_count         = 0;
  int node_status_count = 0;
  uint16_t count        = 0;
  while (ros::ok())
  {
    ros::Time t1 = ros::Time::now();
    sec_count++;
    count++;
    node_status_count++;

    if (sec_count > 25)
    {
      sec_count = 0;
      if (obu_device_.isConn == 0)
      {
        obu_device_.tcp_process.reset(new TcpProcess());
        obu_device_.tcp_process->log_dir_     = log_dir_stream.str();
        obu_device_.tcp_process->sensor_name_ = yaml_doc["sensor_name"].as< string >();
        obu_device_.tcp_process->device_name_ = obu_device_.device_name;
        obu_device_.isConn =
            obu_device_.tcp_process->Initial(obu_status_, obu_device_.device_ip, obu_device_.intput_port, 1024);
      }
    }
    if (node_status_count > 3)
    {
      node_status_count = 0;
      status_msgs::NodeStatus ns;
      ns.node_name = ros::this_node::getName();
      ns.node_pid  = getpid();
      ns.state_num = 1;
      status_msgs::SafetyStatus ss;

      common_msgs::KeyValue kv;
      ss.message_code = "I0702001";
      ss.counter      = count;
      ss.hardware_id  = 0;

      ns.header.frame_id = "/odom";
      ns.header.stamp    = ros::Time::now();

      pub_.publish(ns);
    }
    if (obu_device_.isConn != 0)
    {
      char sendBuf[100];
      sprintf(sendBuf, "%ld,%d,", count, g_agv_id);
      // ROS_INFO("id[%d]  agv_id[%d] str[%s]", count, g_agv_id, sendBuf); // d.toSec());
      obu_device_.tcp_process->sendTcpInfo(( unsigned char * )sendBuf, strlen(sendBuf));
    }

    ros::Time t2    = ros::Time::now();
    ros::Duration d = t2 - t1;
    // ROS_INFO("[%d]node  [%ld]NSec", sec_count, d.toNSec()); // d.toSec());
    ros::spinOnce();
    loop_rate.sleep();
  }

  // clear
  // g_map_obu_status.clear();
  //销毁线程锁
  pthread_mutex_destroy(&list_mutex);
  return 0;
}