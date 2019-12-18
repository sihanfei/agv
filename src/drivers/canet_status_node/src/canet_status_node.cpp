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

using namespace std;

#define NODE_NAME "canet_status_node"

#define CANET_YAML_FILE_PATH "/work/superg_agv/src/drivers/canet_status_node/cfg"
#define CANET_YAML_FILE_NAME "/config.yaml"

struct Canet_Device
{
  std::string device_name;
  bool device_enable;
  std::string device_ip;
  int intput_port;
  int isConn;
  boost::shared_ptr< TcpProcess > tcp_process;
};

map< string, Canet_Device > g_map_canet_status;

//定义线程锁
pthread_mutex_t m_mutex;

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
void operator>>(const YAML::Node &node, Canet_Device &canetDevice)
{
  node["device_name"] >> canetDevice.device_name;
  node["device_enable"] >> canetDevice.device_enable;
  node["device_ip"] >> canetDevice.device_ip;
  node["intput_port"] >> canetDevice.intput_port;
}

class CanetStatusProcess : public TcpProcessCallBack
{
public:
  CanetStatusProcess();
  ~CanetStatusProcess();
  void OnTcpProcessCallBack(unsigned char *data, int len, struct sockaddr_in addr_);

  void OnTcpStatusCallBack(int istatus, struct sockaddr_in addr_);

  void status_Sender();

  void start();

  // boost::shared_ptr< UdpProcess > udp_;
  // std::string device_ip_;
  // int intput_port_;
};

CanetStatusProcess::CanetStatusProcess()
{
  ROS_INFO("CanetStatusProcess process[%d] thread[%u]", getpid(), ( unsigned int )pthread_self());

  // udp_.reset(new UdpProcess());
  // udp_->log_dir_     = log_dir;
  // udp_->sensor_name_ = sensor_name;
  // udp_->device_name_ = ContiDevice.device_name;
  // device_ip_         = ContiDevice.device_ip;
  // intput_port_       = ContiDevice.intput_port;
}
CanetStatusProcess::~CanetStatusProcess()
{
  // udp_.reset();
}

void CanetStatusProcess::OnTcpStatusCallBack(int istatus, struct sockaddr_in addr_)
{
  //申请线程锁
  pthread_mutex_lock(&m_mutex);
  ROS_INFO("ip[%s] port[%d] istatus = %d", inet_ntoa(addr_.sin_addr), htons(addr_.sin_port), istatus);
  ostringstream map_key;

  map_key.str("");
  map_key << inet_ntoa(addr_.sin_addr) << ":" << htons(addr_.sin_port);
  map< string, Canet_Device >::iterator status_it_;
  status_it_ = g_map_canet_status.find(map_key.str());
  if (status_it_ != g_map_canet_status.end())
    status_it_->second.isConn = istatus;
  //释放线程锁
  pthread_mutex_unlock(&m_mutex);
}

void CanetStatusProcess::OnTcpProcessCallBack(unsigned char *data, int len, struct sockaddr_in addr_)
{
  ostringstream ost;
  for (int ii = 0; ii < len; ii++)
  {
    ost << "0x" << hex << uppercase << setw(2) << setfill('0') << ( int )data[ii] << " ";
  }
  ROS_INFO("ip[%s] port[%d] rec = %s", inet_ntoa(addr_.sin_addr), htons(addr_.sin_port), ost.str().c_str());
}
void CanetStatusProcess::status_Sender()
{
  ROS_INFO("status_Sender:int %d main process, the tid=%lu,pid=%ld", getpid(), pthread_self(), syscall(SYS_gettid));
  ROS_INFO("process[%d] thread[%lu] status_Sender Start!", getpid(), pthread_self());

  // ros::NodeHandle nh;
  // ros::Publisher radar_info_pub_        = nh.advertise< perception_sensor_msgs::ObjectList >(topic1_.data(), 2);
  // ros::Publisher radar_point_cloud_pub_ = nh.advertise< sensor_msgs::PointCloud2 >(topic2_.data(), 2);

  // ros::Rate loop_rate(100);
  // while (ros::ok())
  // {
  //   loop_rate.sleep();
  //   if (!list_arsABC_objdata_.empty())
  //   {
  //     // ROS_INFO("thread[%u] list size = %d", ( unsigned int )pthread_self(), ( int )list_arsABC_objdata_.size());
  //     arsABC_ObjData arsABC_obj_;
  //     arsABC_obj_ = list_arsABC_objdata_.front();

  //     perception_sensor_msgs::ObjectList object_list;
  //     common_msgs::DetectionInfo detection_info;
  //     // perception_sensor_msgs::Point point;

  //     pcl::PointCloud< pcl::PointXYZ > radar_point;
  //     pcl::PointXYZ pcl_point;

  //     object_list.obstacle_num    = arsABC_obj_.Object_NofObjects_;
  //     object_list.header.frame_id = "odom";
  //     object_list.header.stamp    = ros::Time::now();
  //     for (int k = 0; k < object_list.obstacle_num; k++)
  //     {
  //       detection_info.id = arsABC_obj_.map_ars60B_data_[k].Object_ID_;

  //       detection_info.obj_class       = 255;
  //       detection_info.state[0]        = arsABC_obj_.map_ars60B_data_[k].Object_DistLong_ * 1000;
  //       detection_info.state[1]        = arsABC_obj_.map_ars60B_data_[k].Object_DistLat_ * 1000;
  //       detection_info.state[2]        = arsABC_obj_.map_ars60B_data_[k].Object_VrelLong_ * 1000;
  //       detection_info.state[3]        = arsABC_obj_.map_ars60B_data_[k].Object_VrelLat_ * 1000;
  //       detection_info.measurement_cov = {0.009, 0, 0, 0, 0, 0.009, 0, 0, 0, 0, 0.09, 0, 0, 0, 0, 0.09};
  //       detection_info.confidence      = arsABC_obj_.map_ars60C_data_[k].Obj_ProbOfExist_;
  //       detection_info.peek[0].x       = 2147483647;
  //       detection_info.peek[0].y       = 2147483647;
  //       detection_info.peek[1].x       = 2147483647;
  //       detection_info.peek[1].y       = 2147483647;
  //       detection_info.peek[2].x       = 2147483647;
  //       detection_info.peek[2].y       = 2147483647;
  //       detection_info.peek[3].x       = 2147483647;
  //       detection_info.peek[3].y       = 2147483647;

  //       object_list.object_list.push_back(detection_info);

  //       // pub for projection
  //       pcl_point.x = arsABC_obj_.map_ars60B_data_[k].Object_DistLong_;
  //       pcl_point.y = arsABC_obj_.map_ars60B_data_[k].Object_DistLat_;
  //       pcl_point.z = 0;
  //       radar_point.push_back(pcl_point);
  //     }
  //     sensor_msgs::PointCloud2 msg_radar;
  //     pcl::toROSMsg(radar_point, msg_radar);
  //     msg_radar.header.frame_id = "odom";
  //     msg_radar.header.stamp    = ros::Time::now();
  //     radar_point_cloud_pub_.publish(msg_radar);
  //     radar_point.clear();

  //     if (radar_info_pub_.getNumSubscribers() != 0) // no one listening?// avoid much work
  //     {
  //       radar_info_pub_.publish(object_list);
  //     }

  //     object_list.object_list.clear();
  //     list_arsABC_objdata_.pop_front();

  //   } // list_arsABC_objdata_.empty()
  // }   // while (ros::ok())
}

void CanetStatusProcess::start()
{
  // udp_->Initial(boost::shared_ptr< RadarDataProcess >(this), device_ip_, intput_port_, 13);

  boost::thread thrd(boost::bind(&CanetStatusProcess::status_Sender, this));
  thrd.detach();
}

// adstatus

int main(int argc, char *argv[])
{
  ROS_INFO("ROS node is start, name is [%s], file name is %s", NODE_NAME, argv[0]);
  ros::init(argc, argv, NODE_NAME); // node name
  ros::NodeHandle nh;

  ////////////////////glog
  GLogHelper gh(argv[0]);
  gh.setLogDirectory(argv[0]);

  //初始化线程锁
  pthread_mutex_init(&m_mutex, NULL);

  //////////////////yaml file
  char *home_path           = getenv("HOME");
  char yaml_file_name[1024] = {0};
  sprintf(yaml_file_name, "%s%s%s", home_path, CANET_YAML_FILE_PATH, CANET_YAML_FILE_NAME);
  SUPERG_INFO << "yaml_file_name:" << yaml_file_name;

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  std::string file_name = yaml_file_name;
  YAML::Node doc;
  doc = YAML::LoadFile(file_name);
  // SUPERG_INFO << "config_num: " << doc.size();
  // SUPERG_INFO << "doc[sensor_name]: " << doc["sensor_name"];

  // SUPERG_INFO << "doc[device_list]: " << doc["device_list"].size();

  ostringstream log_dir_stream;
  log_dir_stream.fill('0');
  log_dir_stream << home_path << doc["log_dir"].as< string >();

  // map
  ostringstream map_key;
  for (unsigned i = 0; i < doc["device_list"].size(); i++)
  {
    Canet_Device canet_device_;
    doc["device_list"][i] >> canet_device_;
    canet_device_.isConn = 0;
    if (canet_device_.device_enable)
    {
      map_key.str("");
      map_key << canet_device_.device_ip << ":" << canet_device_.intput_port;
      g_map_canet_status[map_key.str()] = canet_device_;
    }
  }

  //  data process class
  boost::shared_ptr< CanetStatusProcess > canet_status_(new CanetStatusProcess());
  canet_status_->start();

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  ros::Publisher pub_ = nh.advertise< status_msgs::NodeStatus >("/node/node_status", 10);

  ros::Rate loop_rate(10);
  int sec_count = 10;
  int count     = 0;
  while (ros::ok())
  {
    ros::Time t1 = ros::Time::now();
    sec_count++;
    count++;

    status_msgs::NodeStatus ns;
    ns.node_name = ros::this_node::getName();
    ns.node_pid  = getpid();
    ns.state_num = 1;
    status_msgs::SafetyStatus ss;

    count++;
    common_msgs::KeyValue kv;
    ss.message_code = "I0701001";
    ss.counter      = count;
    ss.hardware_id  = 0;

    if (sec_count > 10)
    {
      sec_count = 0;
      map< string, Canet_Device >::iterator canet_status_Iter;
      for (canet_status_Iter = g_map_canet_status.begin(); canet_status_Iter != g_map_canet_status.end();
           canet_status_Iter++)
      {
        // ROS_INFO("isConn[%d]", canet_status_Iter->second.isConn);
        if (canet_status_Iter->second.isConn == 0)
        {
          canet_status_Iter->second.tcp_process.reset(new TcpProcess());
          canet_status_Iter->second.tcp_process->log_dir_     = log_dir_stream.str();
          canet_status_Iter->second.tcp_process->sensor_name_ = doc["sensor_name"].as< string >();
          canet_status_Iter->second.tcp_process->device_name_ = canet_status_Iter->second.device_name;
          canet_status_Iter->second.isConn                    = canet_status_Iter->second.tcp_process->Initial(
              canet_status_, canet_status_Iter->second.device_ip, canet_status_Iter->second.intput_port, 8);
        }
      }
    }
    pub_.publish(ns);
    ros::Time t2    = ros::Time::now();
    ros::Duration d = t2 - t1;
    // ROS_INFO("[%d]node  [%ld]NSec", sec_count, d.toNSec()); // d.toSec());
    ros::spinOnce();
    loop_rate.sleep();
  }

  // clear
  g_map_canet_status.clear();
  //销毁线程锁
  pthread_mutex_destroy(&m_mutex);
  return 0;
}