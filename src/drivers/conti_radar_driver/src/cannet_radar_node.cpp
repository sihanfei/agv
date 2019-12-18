#include <ros/ros.h>

// #include "glog_helper.h"
#include "udp_process.h"
#include <iostream>

#include "common_functions.h"
// #include <sys/types.h>

#include <boost/predef/other/endian.h>
// #include <endian.h>
#include <pcl_ros/point_cloud.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <common_msgs/DetectionInfo.h>
#include <perception_sensor_msgs/ObjectList.h>

#include <thread>

#include <list>
#include <map>

#include <yaml-cpp/yaml.h>

#include <sys/syscall.h>
#include <unistd.h> //getpid()

#include "common_msgs/KeyValue.h"
#include "status_msgs/NodeStatus.h"
#include "status_msgs/SafetyStatus.h"

using namespace std;

#define NODE_NAME "cannet_radar_node"

#define CONTI_YAML_FILE_PATH "/work/superg_agv/src/drivers/conti_radar_driver/cfg"
#define CONTI_YAML_FILE_NAME "/config.yaml"

struct Conti_Device
{
  std::string device_name;
  bool device_enable;
  std::string device_ip;
  int intput_port;
  std::string topic_name_1;
  std::string topic_name_2;
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
void operator>>(const YAML::Node &node, Conti_Device &contiDevice)
{
  node["device_name"] >> contiDevice.device_name;
  node["device_enable"] >> contiDevice.device_enable;
  node["device_ip"] >> contiDevice.device_ip;
  node["intput_port"] >> contiDevice.intput_port;
  node["topic_name_1"] >> contiDevice.topic_name_1;
  node["topic_name_2"] >> contiDevice.topic_name_2;
}

typedef struct
{
  uint8_t ide;         /*!< 0: standard frame, 1: extended frame */
  uint8_t dlc;         /*!< data length */
  uint8_t rtr;         /*!< remote transmission request */
  uint8_t prio;        /*!< priority */
  uint32_t id;         /*!< identify */
  uint8_t can_data[8]; /*!< datas */
} * p_myCanData, myCanData;

typedef struct
{
  uint8_t Object_ID_;      // Object ID
  double Object_DistLong_; // Longitudinal (x) coordinate
  double Object_DistLat_;  // Lateral (y) coordinate
  double Object_VrelLong_; // Relative velocity in longitudinal direction (x)
  double Object_VrelLat_;  // Relative velocity in lateral direction (y)
  uint8_t Object_DynProp_; // Dynamic property of the object indicating if the object is moving or stationary

} * p_ars60B_Data, ars60B_Data;

typedef struct
{
  uint32_t icount_;         // 60C msgs counter
  uint8_t Obj_ProbOfExist_; // Probability of existence
} * p_ars60C_Data, ars60C_Data;

typedef struct
{
  int iflag_;
  uint8_t Object_NofObjects_;   // Number of objects (max. 100 Objects)
  uint16_t Object_MeasCounter_; // Measurement cycle counter (counting up since startup of sensor and restarting at 0
  // when > 65535)
  int obj60b_count;
  int obj60c_count;
  map< int, ars60B_Data > map_ars60B_data_;
  map< int, ars60C_Data > map_ars60C_data_;

} * p_arsABC_ObjData, arsABC_ObjData;

class RadarDataProcess : public UdpProcessCallBack
{
public:
  RadarDataProcess(std::string topic1, std::string topic2);

  RadarDataProcess(std::string log_dir, std::string sensor_name, Conti_Device &ContiDevice);
  ~RadarDataProcess();
  void OnUdpProcessCallBack(unsigned char *data, int len, struct sockaddr_in addr_);

  void radar_Sender();

  void start();

  list< arsABC_ObjData > list_arsABC_objdata_;

  arsABC_ObjData arsABC_objdata_;

  list< myCanData > list_myCanData_;

  std::string topic1_, topic2_;

  // boost::shared_ptr< UdpProcess > udp_;
  // std::string device_ip_;
  // int intput_port_;
};
RadarDataProcess::RadarDataProcess(std::string topic1, std::string topic2) : topic1_(topic1), topic2_(topic2)
{
  ROS_INFO("process[%d] thread[%u] %s", getpid(), ( unsigned int )pthread_self(), topic1_.data());

  arsABC_objdata_.iflag_ = 0;
}
RadarDataProcess::RadarDataProcess(std::string log_dir, std::string sensor_name, Conti_Device &ContiDevice)
{
  // ROS_INFO("RadarDataProcess process[%d] thread[%lu] %s", getpid(), ( unsigned int )pthread_self(), topic1_.data());
  arsABC_objdata_.iflag_ = 0;
  topic1_                = ContiDevice.topic_name_1;
  topic2_                = ContiDevice.topic_name_2;

  // udp_.reset(new UdpProcess());
  // udp_->log_dir_     = log_dir;
  // udp_->sensor_name_ = sensor_name;
  // udp_->device_name_ = ContiDevice.device_name;
  // device_ip_         = ContiDevice.device_ip;
  // intput_port_       = ContiDevice.intput_port;
}

RadarDataProcess::~RadarDataProcess()
{
  // udp_.reset();
}

void RadarDataProcess::OnUdpProcessCallBack(unsigned char *data, int len, struct sockaddr_in addr_)
{

  //解析CAN协议 提取CAN ID
  myCanData canbuf_;
  canbuf_.id = bytesToInt32(&data[1], 4);
  memcpy(canbuf_.can_data, &data[5], 8);

  list_myCanData_.push_back(canbuf_);

  //判断是否等于 60A 添加到发送列表并清空单包变量
  if (canbuf_.id == 0x60A)
  {
    if (arsABC_objdata_.iflag_ == 1)
      list_arsABC_objdata_.push_back(arsABC_objdata_);
    arsABC_objdata_.iflag_              = 1;
    arsABC_objdata_.obj60b_count        = 0;
    arsABC_objdata_.obj60c_count        = 0;
    arsABC_objdata_.Object_NofObjects_  = canbuf_.can_data[0];
    arsABC_objdata_.Object_MeasCounter_ = bytesToInt16(&canbuf_.can_data[1]);
    arsABC_objdata_.map_ars60B_data_.clear();
    arsABC_objdata_.map_ars60C_data_.clear();
  }
  // printf("%d-%d \n", data[5], data[5]);
  // printf("%d-%d \n", arsABC_objdata_.Object_NofObjects_, arsABC_objdata_.Object_MeasCounter_);

  // 60B 加入 OBJ 数据
  if (canbuf_.id == 0x60B)
  {
    ars60B_Data ars60B_Data_;
    // ID
    ars60B_Data_.Object_ID_ = (canbuf_.can_data[0] >> 0);
    // distance x
    ars60B_Data_.Object_DistLong_ = ((canbuf_.can_data[1] << 5) | (canbuf_.can_data[2] >> 3)) * 0.2 - 500;
    // distance y
    ars60B_Data_.Object_DistLat_ = (((canbuf_.can_data[2] & 0x07) << 8) | (canbuf_.can_data[3] >> 0)) * 0.2 - 204.6;
    // velocity x
    ars60B_Data_.Object_VrelLong_ = ((canbuf_.can_data[4] << 2) | (canbuf_.can_data[5] >> 6)) * 0.25 - 128;
    // velocity y
    ars60B_Data_.Object_VrelLat_ = (((canbuf_.can_data[5] & 0x3F) << 3) | (canbuf_.can_data[6] >> 5)) * 0.25 - 64;
    arsABC_objdata_.map_ars60B_data_.insert({arsABC_objdata_.obj60b_count, ars60B_Data_});
    arsABC_objdata_.obj60b_count++;
  }
  // 60C 加入
  if (canbuf_.id == 0x60C)
  {
    ars60C_Data ars60C_Data_;
    // ID
    ars60C_Data_.icount_ = arsABC_objdata_.obj60c_count;
    // existance probability
    ars60C_Data_.Obj_ProbOfExist_ = canbuf_.can_data[6] >> 5;

    arsABC_objdata_.map_ars60C_data_.insert({arsABC_objdata_.obj60c_count, ars60C_Data_});
    arsABC_objdata_.obj60c_count++;
  }
}

//发布话题
void RadarDataProcess::radar_Sender()
{
  ROS_INFO("radar_Sender:int %d main process, the tid=%lu,pid=%ld", getpid(), pthread_self(), syscall(SYS_gettid));
  ROS_INFO("process[%d] thread[%lu] radar_Sender Start!", getpid(), pthread_self());

  ros::NodeHandle nh;
  ros::Publisher radar_info_pub_        = nh.advertise< perception_sensor_msgs::ObjectList >(topic1_.data(), 2);
  ros::Publisher radar_point_cloud_pub_ = nh.advertise< sensor_msgs::PointCloud2 >(topic2_.data(), 2);

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    loop_rate.sleep();
    if (!list_arsABC_objdata_.empty())
    {
      // ROS_INFO("thread[%u] list size = %d", ( unsigned int )pthread_self(), ( int )list_arsABC_objdata_.size());
      arsABC_ObjData arsABC_obj_;
      arsABC_obj_ = list_arsABC_objdata_.front();

      perception_sensor_msgs::ObjectList object_list;
      common_msgs::DetectionInfo detection_info;
      // perception_sensor_msgs::Point point;

      pcl::PointCloud< pcl::PointXYZ > radar_point;
      pcl::PointXYZ pcl_point;
      // pcl::PointXYZI pcl_pointxyzi_;

      object_list.obstacle_num    = arsABC_obj_.Object_NofObjects_;
      object_list.header.frame_id = "odom";
      object_list.header.stamp    = ros::Time::now();
      for (int k = 0; k < object_list.obstacle_num; k++)
      {
        detection_info.id = arsABC_obj_.map_ars60B_data_[k].Object_ID_;

        detection_info.obj_class       = 255;
        detection_info.state[0]        = arsABC_obj_.map_ars60B_data_[k].Object_DistLong_ * 1000;
        detection_info.state[1]        = arsABC_obj_.map_ars60B_data_[k].Object_DistLat_ * 1000;
        detection_info.state[2]        = arsABC_obj_.map_ars60B_data_[k].Object_VrelLong_ * 1000;
        detection_info.state[3]        = arsABC_obj_.map_ars60B_data_[k].Object_VrelLat_ * 1000;
        detection_info.measurement_cov = {0.009, 0, 0, 0, 0, 0.009, 0, 0, 0, 0, 0.09, 0, 0, 0, 0, 0.09};
        detection_info.confidence      = arsABC_obj_.map_ars60C_data_[k].Obj_ProbOfExist_;
        detection_info.peek[0].x       = 2147483647;
        detection_info.peek[0].y       = 2147483647;
        detection_info.peek[1].x       = 2147483647;
        detection_info.peek[1].y       = 2147483647;
        detection_info.peek[2].x       = 2147483647;
        detection_info.peek[2].y       = 2147483647;
        detection_info.peek[3].x       = 2147483647;
        detection_info.peek[3].y       = 2147483647;

        object_list.object_list.push_back(detection_info);

        // pub for projection
        pcl_point.x = arsABC_obj_.map_ars60B_data_[k].Object_DistLong_;
        pcl_point.y = arsABC_obj_.map_ars60B_data_[k].Object_DistLat_;
        pcl_point.z = 0;
        // pcl_point.intensity = 1.1;

        radar_point.push_back(pcl_point);
      }

      sensor_msgs::PointCloud2 msg_radar;
      pcl::toROSMsg(radar_point, msg_radar);
      msg_radar.header.frame_id = "odom";
      msg_radar.header.stamp    = ros::Time::now();
      radar_point_cloud_pub_.publish(msg_radar);
      radar_point.clear();

      if (radar_info_pub_.getNumSubscribers() != 0) // no one listening?// avoid much work
      {
        radar_info_pub_.publish(object_list);
      }

      object_list.object_list.clear();
      list_arsABC_objdata_.pop_front();

    } // list_arsABC_objdata_.empty()
  }   // while (ros::ok())
}

void RadarDataProcess::start()
{
  // udp_->Initial(boost::shared_ptr< RadarDataProcess >(this), device_ip_, intput_port_, 13);

  boost::thread thrd(boost::bind(&RadarDataProcess::radar_Sender, this));
  thrd.detach();
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
  // SUPERG_INFO << "config_num: " << doc.size();
  // SUPERG_INFO << "doc[sensor_name]: " << doc["sensor_name"];

  // SUPERG_INFO << "doc[device_list]: " << doc["device_list"].size();

  ostringstream log_dir_stream;
  log_dir_stream.fill('0');
  log_dir_stream << home_path << doc["log_dir"].as< string >();

  // udp class
  // UdpProcess udp_[doc["device_list"].size()];
  boost::shared_ptr< UdpProcess > udp_[doc["device_list"].size()];

  // radar data process class
  boost::shared_ptr< RadarDataProcess > radar_[doc["device_list"].size()];
  for (unsigned i = 0; i < doc["device_list"].size(); i++)
  {
    Conti_Device Conti_Device_;
    doc["device_list"][i] >> Conti_Device_;

    if (Conti_Device_.device_enable)
    {
      // radar_[i].reset(new RadarDataProcess(log_dir_stream.str(), doc["sensor_name"].as< string >(), Conti_Device_));
      // radar_[i]->start();

      udp_[i].reset(new UdpProcess());
      udp_[i]->log_dir_     = log_dir_stream.str();
      udp_[i]->sensor_name_ = doc["sensor_name"].as< string >();
      udp_[i]->device_name_ = Conti_Device_.device_name;
      // RadarDataProcess radar_(Conti_Device_.topic_name_1, Conti_Device_.topic_name_2);
      radar_[i].reset(new RadarDataProcess(Conti_Device_.topic_name_1, Conti_Device_.topic_name_2));

      if (udp_[i]->Initial(radar_[i], Conti_Device_.device_ip, Conti_Device_.intput_port, 13) == 1)
      {
        radar_[i]->start();
      }
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  int pid = getpid();

  ros::Publisher pub_ = nh.advertise< status_msgs::NodeStatus >("/node/node_status", 10);

  ros::Rate loop_rate(100);

  int icount = 0;

  while (ros::ok())
  {
    icount++;
    if (icount > 10)
    {
      icount = 1;

      status_msgs::NodeStatus ns;
      ns.node_name = ros::this_node::getName();
      ns.node_pid  = pid;
      ns.state_num = 1;

      ns.header.frame_id = "/odom";
      ns.header.stamp    = ros::Time::now();

      pub_.publish(ns);
    }
    ros::spinOnce();

    loop_rate.sleep();
  }
  // ros::spin();
  return 0;
}