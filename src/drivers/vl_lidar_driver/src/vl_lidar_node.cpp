#include <ros/ros.h>

// #include "glog_helper.h"
#include "udp_process.h"
#include <iostream>
#include <sstream> //

#include "common_functions.h"
// #include <sys/types.h>

// #include <boost/predef/other/endian.h>
// // #include <endian.h>
// #include <pcl_ros/point_cloud.h>
// // #include <pcl/point_cloud.h>
// // #include <pcl/point_types.h>
// // #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/conversions.h>
// #include <sensor_msgs/PointCloud2.h>

// #include <common_msgs/DetectionInfo.h>
// #include <perception_sensor_msgs/ObjectList.h>

#include <boost/thread.hpp>

#include <thread>

#include <list>
#include <map>

#include <yaml-cpp/yaml.h>

#include <sys/syscall.h>
#include <unistd.h> //getpid()

#include "common_msgs/KeyValue.h"
#include "status_msgs/NodeStatus.h"
#include "status_msgs/SafetyStatus.h"
//////////////////spdlog//////////////////
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include "spdlog/logger.h"

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

#include "spdlog/async.h"
#include "spdlog/fmt/bin_to_hex.h"
//////////velodyne pointcloud////////////
#include "pointcloudXYZIR.h"
#include "rawdata.h"
#include <boost/foreach.hpp>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include "location_msgs/FusionDataInfo.h"
#include <perception_sensor_msgs/LidarPointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_msgs/VelodyneScan.h>

#include "ImageSegment_linh.h"

using namespace std;
using namespace boost;

using namespace Eigen;

#define NODE_NAME "vl_lidar_node"
//定义线程锁
pthread_mutex_t list_mutex_raw;
pthread_mutex_t list_mutex_cut;

pthread_mutex_t location_mutex;

template < class Type > Type strToNum(const string &str)
{
  istringstream iss(str);
  Type num;
  iss >> num;
  return num;
}
typedef struct
{
  // velodyne_msgs::VelodyneScan agv_scan_;
  velodyne_msgs::VelodyneScan obj_scan_;
  location_msgs::FusionDataInfo start_location_;
  location_msgs::FusionDataInfo end_location_;
} lidar_info_struct;
class LidarDataProcess : public UdpProcessCallBack
{
public:
  LidarDataProcess(ros::NodeHandle node);
  ~LidarDataProcess();
  void OnUdpProcessCallBack(unsigned char *data, int len, struct sockaddr_in addr_);

  void lidar_data_cut();
  void lidar_raw_pl2();

  void start();

  void recvFusionLocationCallback(const location_msgs::FusionDataInfo::ConstPtr &location_msg);

private:
  ros::NodeHandle nh_;
  ros::Subscriber location_sub_; // = n.subscribe("/localization/fusion_msg", 1, recvFusionLocationCallback);

  boost::shared_ptr< velodyne_driver::RawData > raw_data_;

  list< velodyne_msgs::VelodyneScan > list_raw_VelodyneScan_;

  std::list< lidar_info_struct > list_cut_VelodyneScan_;

  velodyne_msgs::VelodyneScan temp_raw_;
  lidar_info_struct temp_cut_;

  int packets_count;
  // int ttt;
  // int t1;
  int pub_raw_data_;

  ImageSegment imageSegment;
  Eigen::Matrix4f Matrix4f_1_;
  Eigen::Matrix4f Matrix4f_2_;

  location_msgs::FusionDataInfo temp_location_;
  //

  struct
  {
    std::string frame_id; ///< tf frame ID
    // std::string model;    ///< device model name
    int npackets; ///< number of packets to collect
                  // double rpm;           ///< device rotation rate (RPMs)
    // int cut_angle;        ///< cutting angle in 1/100°

    int packet_size;     //数据包大小
    int begin_cut_angle; //车体切割开始角度 1/100°
    int end_cut_angle;   //结束角度1/100°

    double time_offset; ///< time in seconds added to each velodyne time stamp

    std::string raw_data_topic_; //原始点云topic名称
    std::string data_set_topic_; //处理后的点云topic名称

    std::string Matrix4f_1;

    double xmin; //矩形切割x正向
    double xmax; //矩形切割x反向
    double ymin; //矩形切割y正向
    double ymax; //矩形切割y反向

  } config_;

  bool isHandle;
};

LidarDataProcess::LidarDataProcess(ros::NodeHandle node) : nh_(node), raw_data_(new velodyne_driver::RawData())
{
  SPDLOG_DEBUG("LidarDataProcess process[{}] thread[{}]", getpid(), pthread_self());
  nh_.param("frame_id", config_.frame_id, std::string("odom"));
  nh_.param("npackets", config_.npackets, 76);

  nh_.param("begin_cut_angle", config_.begin_cut_angle, 0);
  nh_.param("end_cut_angle", config_.end_cut_angle, 90);

  config_.begin_cut_angle = config_.begin_cut_angle * 100;
  config_.end_cut_angle   = config_.end_cut_angle * 100;

  nh_.param("raw_data_topic_", config_.raw_data_topic_, std::string("/null"));
  nh_.param("data_set_topic_", config_.data_set_topic_, std::string("/null"));

  nh_.param("pub_raw_data", pub_raw_data_, 0);

  nh_.param("Matrix4f_1", config_.Matrix4f_1, std::string("0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"));

  nh_.param("xmin", config_.xmin, -1.6);
  nh_.param("xmax", config_.xmax, 1.6);
  nh_.param("ymin", config_.ymin, -7.6);
  nh_.param("ymax", config_.ymax, 7.6);

  SPDLOG_DEBUG("config_.frame_id={}", config_.frame_id);
  SPDLOG_DEBUG("config_.npackets={}", config_.npackets);
  SPDLOG_DEBUG("config_.begin_cut_angle={}", config_.begin_cut_angle);
  SPDLOG_DEBUG("config_.end_cut_angle={}", config_.end_cut_angle);
  SPDLOG_DEBUG("config_.raw_data_topic_={}", config_.raw_data_topic_);
  SPDLOG_DEBUG("config_.data_set_topic_={}", config_.data_set_topic_);

  SPDLOG_DEBUG("config_.Matrix4f_1={}", config_.Matrix4f_1);

  packets_count = 0;

  raw_data_->setup(nh_);

  std::vector< std::string > v1 = string_split(config_.Matrix4f_1, ",");

  Matrix4f_1_ << strToNum< float >(v1.at(0)), strToNum< float >(v1.at(1)), strToNum< float >(v1.at(2)),
      strToNum< float >(v1.at(3)), strToNum< float >(v1.at(4)), strToNum< float >(v1.at(5)),
      strToNum< float >(v1.at(6)), strToNum< float >(v1.at(7)), strToNum< float >(v1.at(8)),
      strToNum< float >(v1.at(9)), strToNum< float >(v1.at(10)), strToNum< float >(v1.at(11)),
      strToNum< float >(v1.at(12)), strToNum< float >(v1.at(13)), strToNum< float >(v1.at(14)),
      strToNum< float >(v1.at(15));

  std::cout << Matrix4f_1_ << endl;

  location_sub_ = nh_.subscribe("/localization/fusion_msg", 1, &LidarDataProcess::recvFusionLocationCallback, this);

  // ttt = 0;
  // t1  = 0;
  isHandle = true;
}

LidarDataProcess::~LidarDataProcess()
{
  // udp_.reset();
}
void LidarDataProcess::recvFusionLocationCallback(const location_msgs::FusionDataInfo::ConstPtr &location_msg)
{
  // SPDLOG_DEBUG("recvFusionLocationCallback thread=[{}]", pthread_self()); // boost::this_thread::get_id());
  pthread_mutex_lock(&location_mutex);
  temp_location_ = *location_msg;
  pthread_mutex_unlock(&location_mutex);
}
void LidarDataProcess::OnUdpProcessCallBack(unsigned char *data, int len, struct sockaddr_in addr_)
{
  // SPDLOG_DEBUG("OnUdpProcessCallBack thread=[{}]", pthread_self());
  //接收到76包数据后，组合成一帧数据 放入list中，如果list非空，清空list后放入，保证点云处理线程每次都处理当前最新数据
  velodyne_msgs::VelodynePacket tmp_packet;

  tmp_packet.stamp = ros::Time::now();
  // SPDLOG_DEBUG("tmp_packet stamp[{}]", tmp_packet.stamp.toSec());
  memcpy(&tmp_packet.data[0], ( char * )data, len);

  //放到原始数据帧 velodyne_msgs::VelodyneScan temp_raw_ 里面
  // if (keep_raw_data_ == 1)
  //   temp_raw_.packets.push_back(tmp_packet);

  //接收到一包后 获取角度 比较是需要裁切掉，如果是有效角度，就放到 切割数据 velodyne_msgs::VelodyneScan temp_cut_ 里面
  packets_count++;
  std::size_t azimuth_data_pos = 100 * 0 + 2;
  int azimuth                  = *(( u_int16_t * )(&tmp_packet.data[azimuth_data_pos]));
  // if (azimuth > ttt)
  // {
  //   t1 = t1 + (azimuth - ttt);
  //   SPDLOG_DEBUG("packets_count[{}] azimuth[{}] t1[{}]", packets_count, azimuth - ttt, t1);
  //   ttt = azimuth;
  // }
  // else
  // {
  //   t1  = t1 + (azimuth - 0);
  //   ttt = azimuth;
  // }

  temp_cut_.obj_scan_.packets.push_back(tmp_packet);

  // if (config_.begin_cut_angle > config_.end_cut_angle)
  // {
  //   if (azimuth > config_.end_cut_angle && azimuth < config_.begin_cut_angle)
  //   {
  //     temp_cut_.obj_scan_.packets.push_back(tmp_packet);
  //   }
  //   else
  //     temp_cut_.agv_scan_.packets.push_back(tmp_packet);
  // }
  // else
  // {
  //   if ((azimuth < config_.begin_cut_angle && azimuth > 0) || (azimuth > config_.end_cut_angle && azimuth < 36001))
  //   {
  //     temp_cut_.obj_scan_.packets.push_back(tmp_packet);
  //   }
  //   else
  //     temp_cut_.agv_scan_.packets.push_back(tmp_packet);
  // }

  if (packets_count == 1 || packets_count == config_.npackets)
  {
    pthread_mutex_lock(&location_mutex);
    if (packets_count == 1)
      temp_cut_.start_location_ = temp_location_;
    else
      temp_cut_.end_location_ = temp_location_;
    pthread_mutex_unlock(&location_mutex);
  }

  if (packets_count >= config_.npackets)
  {
    // SPDLOG_DEBUG("t1[{}]", t1);
    packets_count = 0;
    // t1            = 0;
    // SPDLOG_DEBUG("size[{}]", temp_cut_.packets.size());
    temp_cut_.obj_scan_.header.stamp    = temp_cut_.obj_scan_.packets.front().stamp;
    temp_cut_.obj_scan_.header.frame_id = config_.frame_id;
    // temp_cut_.agv_scan_.header.stamp    = temp_cut_.obj_scan_.packets.front().stamp;
    // temp_cut_.agv_scan_.header.frame_id = config_.frame_id;
    // SPDLOG_DEBUG("temp_cut_[{}]", temp_cut_.header.stamp.toSec());
    //申请线程锁
    pthread_mutex_lock(&list_mutex_cut);
    if (isHandle)
    {

      list_cut_VelodyneScan_.push_back(temp_cut_);
    }
    isHandle = !isHandle;

    temp_cut_.obj_scan_.packets.clear();
    // temp_cut_.agv_scan_.packets.clear();
    //释放线程锁
    pthread_mutex_unlock(&list_mutex_cut);

    // if (keep_raw_data_ == 1)
    // {
    //   temp_raw_.header.stamp    = temp_raw_.packets.front().stamp;
    //   temp_raw_.header.frame_id = config_.frame_id;
    //   //申请线程锁
    //   pthread_mutex_lock(&list_mutex_raw);
    //   list_raw_VelodyneScan_.push_back(temp_raw_);
    //   temp_raw_.packets.clear();
    //   //释放线程锁
    //   pthread_mutex_unlock(&list_mutex_raw);
    // }
  }
}

//发布话题
void LidarDataProcess::lidar_data_cut()
{
  SPDLOG_DEBUG("lidar_data_cut thread=[{}]", pthread_self());
  //  SPDLOG_DEBUG("process[{}] thread[{}] lidar_data_cut Start!", getpid(), pthread_self());

  ros::Publisher lidar_cut_point_pub_ =
      nh_.advertise< perception_sensor_msgs::LidarPointCloud >(config_.data_set_topic_.data(), 1);

  ros::Publisher raw_point_cloud_pub_;
  if (pub_raw_data_ == 1)
    raw_point_cloud_pub_ = nh_.advertise< sensor_msgs::PointCloud2 >(config_.raw_data_topic_.data(), 1);

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    loop_rate.sleep();
    if (!list_cut_VelodyneScan_.empty())
    {
      perception_sensor_msgs::LidarPointCloud temp_lidar_Point_set;
      // begin time
      ros::Time t1 = ros::Time::now();

      std::ostringstream oss;

      // SPDLOG_DEBUG("thread[{}] list_cut_ size[{}]", pthread_self(), list_cut_VelodyneScan_.size());
      velodyne_msgs::VelodyneScan cut_;
      // velodyne_msgs::VelodyneScan agv_;
      //切车体后的数据包

      //申请线程锁
      pthread_mutex_lock(&list_mutex_cut);
      cut_ = list_cut_VelodyneScan_.front().obj_scan_;
      // agv_                                = list_cut_VelodyneScan_.front().agv_scan_;
      temp_lidar_Point_set.location_start = list_cut_VelodyneScan_.front().start_location_;
      temp_lidar_Point_set.location_end   = list_cut_VelodyneScan_.front().end_location_;
      list_cut_VelodyneScan_.pop_front();
      //释放线程锁
      pthread_mutex_unlock(&list_mutex_cut);

      // copy data time
      ros::Time t2 = ros::Time::now();
      oss << "[packets=" << cut_.packets.size() << "] copydata[" << ((t2 - t1).toNSec() / 1000000.0) << "] ";

      //获取点云数据
      // cut data --> pcl
      velodyne_driver::PointcloudXYZIR velodyne_pl_cut;
      velodyne_pl_cut.pc.header.stamp    = pcl_conversions::toPCL(cut_.header).stamp;
      velodyne_pl_cut.pc.header.frame_id = cut_.header.frame_id;
      velodyne_pl_cut.pc.height          = 1;

      // double timestamp = 0;
      for (size_t i = 0; i < cut_.packets.size(); ++i)
      {
        double packetTime = raw_data_->unpack(cut_.packets[i], velodyne_pl_cut);
        // if (packetTime != 0)
        // {
        //   timestamp = packetTime;
        // }
      }
      // agv data --> pcl
      // velodyne_driver::PointcloudXYZIR velodyne_pl_agv;
      // velodyne_pl_agv.pc.header.stamp    = pcl_conversions::toPCL(agv_.header).stamp;
      // velodyne_pl_agv.pc.header.frame_id = agv_.header.frame_id;
      // velodyne_pl_agv.pc.height          = 1;

      // // double timestamp = 0;
      // for (size_t i = 0; i < agv_.packets.size(); ++i)
      // {
      //   double packetTime = raw_data_->unpack(agv_.packets[i], velodyne_pl_agv);
      //   // if (packetTime != 0)
      //   // {
      //   //   timestamp = packetTime;
      //   // }
      // }
      // data->pcl:XYZIR
      ros::Time t3 = ros::Time::now();
      oss << "data->pcl:XYZIR[" << ((t3 - t2).toNSec() / 1000000.0) << "] ";

      // pcl:XYZIR->pcl:XYZI
      // imageSegment.setMinAndMaxAngle(0.0,0.0);
      imageSegment.setPacketsSize(cut_.packets.size());
      imageSegment.setPointCloud(velodyne_pl_cut.pc);
      ros::Time t4 = ros::Time::now();
      oss << "pcl:XYZIR->pcl:XYZI[" << ((t4 - t3).toNSec() / 1000000.0) << "] ";

      //转换坐标系  tf lidar2->car
      imageSegment.tf(Matrix4f_1_);
      ros::Time t5 = ros::Time::now();
      oss << "tf lidar->car[" << ((t5 - t4).toNSec() / 1000000.0) << "] ";

      //分割
      // imageSegment.setPointCloud(new_pcl_);
      imageSegment.segment();
      pcl::PointCloud< pcl::PointXYZI > filtered = imageSegment.getFilteredCloud();
      pcl::PointCloud< pcl::PointXYZI > ground   = imageSegment.getGround();

      ros::Time t6 = ros::Time::now();
      oss << "segment[" << ((t6 - t5).toNSec() / 1000000.0) << "] ";

      //切割车体
      pcl::PointCloud< pcl::PointXYZI > car_cut;
      pcl::PointCloud< pcl::PointXYZI > agv_cut;
      for (pcl::PointXYZI point : filtered)
      {
        if (!(point.x < config_.xmax && point.x > config_.xmin && point.y < config_.ymax && point.y > config_.ymin))
        {
          car_cut.points.push_back(point);
        }
        else
          agv_cut.points.push_back(point);
      }
      ros::Time t7 = ros::Time::now();
      oss << "cut agv[" << ((t7 - t6).toNSec() / 1000000.0) << "] ";
      // sensor_msgs::PointCloud2 laserMsg;

      pcl::toROSMsg(ground, temp_lidar_Point_set.point_cloud_ground);
      temp_lidar_Point_set.point_cloud_ground.header.stamp    = cut_.header.stamp;
      temp_lidar_Point_set.point_cloud_ground.header.frame_id = cut_.header.frame_id;

      pcl::toROSMsg(car_cut, temp_lidar_Point_set.point_cloud_object);
      temp_lidar_Point_set.point_cloud_object.header.stamp    = cut_.header.stamp;
      temp_lidar_Point_set.point_cloud_object.header.frame_id = cut_.header.frame_id;

      pcl::toROSMsg(agv_cut, temp_lidar_Point_set.agv_cloud_object);
      temp_lidar_Point_set.agv_cloud_object.header.stamp    = cut_.header.stamp;
      temp_lidar_Point_set.agv_cloud_object.header.frame_id = cut_.header.frame_id;

      temp_lidar_Point_set.header.stamp = cut_.header.stamp;

      // toROSMsg
      ros::Time t8 = ros::Time::now();
      oss << "toROSMsg[" << ((t8 - t7).toNSec() / 1000000.0) << "] ";

      // pub
      if (lidar_cut_point_pub_.getNumSubscribers() > 0) // no one listening?// avoid much work
      {
        lidar_cut_point_pub_.publish(temp_lidar_Point_set);
      }
      ros::Time t9 = ros::Time::now();
      oss << "pub obj pc2[" << ((t9 - t8).toNSec() / 1000000.0) << "] ";

      ///////////////raw pc2
      if (pub_raw_data_ == 1)
      {
        sensor_msgs::PointCloud2 laserMsg;
        laserMsg.header.stamp    = cut_.header.stamp;
        laserMsg.header.frame_id = cut_.header.frame_id;
        pcl::toROSMsg(velodyne_pl_cut.pc, laserMsg);
        raw_point_cloud_pub_.publish(laserMsg);
      }
      // ros::Time t10 = ros::Time::now();
      // oss << "pub raw pc2[" << ((t10 - t9).toNSec() / 1000000.0) << "] ";

      // oss << "total[" << ((t10 - t1).toNSec() / 1000000.0) << "]";
      // if (((t10 - t1).toNSec() / 1000000.0) > 100.0)
      // {
      //   SPDLOG_ERROR("{}", oss.str());
      // }
      // else
      //   SPDLOG_DEBUG("{}", oss.str());

    } // list_VelodyneScan_.empty()
  }   // while (ros::ok())
}

void LidarDataProcess::start()
{
  boost::thread thrd(boost::bind(&LidarDataProcess::lidar_data_cut, this));
  thrd.detach();
  // if (pub_raw_data_ == 1)
  // {
  //   boost::thread thrd_raw(boost::bind(&LidarDataProcess::lidar_raw_pl2, this));
  //   thrd_raw.detach();
  // }
}

void LidarDataProcess::lidar_raw_pl2()
{
  SPDLOG_DEBUG("lidar_raw_pl2 thread=[{}]", pthread_self());
  ros::Publisher raw_point_cloud_pub_ = nh_.advertise< sensor_msgs::PointCloud2 >(config_.raw_data_topic_.data(), 1);

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    loop_rate.sleep();
    if (!list_raw_VelodyneScan_.empty())
    {

      velodyne_msgs::VelodyneScan raw_;
      //切车体后的数据包

      //申请线程锁
      pthread_mutex_lock(&list_mutex_raw);
      raw_ = list_raw_VelodyneScan_.front();
      list_raw_VelodyneScan_.pop_front();
      //释放线程锁
      pthread_mutex_unlock(&list_mutex_raw);

      // SPDLOG_DEBUG("thread[{}] list_raw size[{}] packets.size[{}]", pthread_self(), list_raw_VelodyneScan_.size(),
      // raw_.packets.size());
      //获取点云数据
      // data --> pcl
      velodyne_driver::PointcloudXYZIR out;
      out.pc.header.stamp    = pcl_conversions::toPCL(raw_.header).stamp;
      out.pc.header.frame_id = raw_.header.frame_id;
      out.pc.height          = 1;

      double timestamp = 0;
      for (size_t i = 0; i < raw_.packets.size(); ++i)
      {
        double packetTime = raw_data_->unpack(raw_.packets[i], out);
        if (packetTime != 0)
        {
          timestamp = packetTime;
        }
      }
      sensor_msgs::PointCloud2 laserMsg;
      laserMsg.header.stamp    = ros::Time().fromSec(timestamp);
      laserMsg.header.frame_id = raw_.header.frame_id;
      pcl::toROSMsg(out.pc, laserMsg);
      raw_point_cloud_pub_.publish(laserMsg);

    } // if .empty()
  }   // while (ros::ok())
}

int main(int argc, char *argv[])
{
  SPDLOG_DEBUG("ROS node is start, name is [{}], file name is {}", NODE_NAME, argv[0]);

  ros::init(argc, argv, NODE_NAME); // node name

  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  SPDLOG_DEBUG("node.getNamespace [{}], private_nh [{}]", node.getNamespace(), private_nh.getNamespace());

  //初始化线程锁
  pthread_mutex_init(&list_mutex_raw, NULL);
  pthread_mutex_init(&list_mutex_cut, NULL);
  pthread_mutex_init(&location_mutex, NULL);

  //////////////////yaml file
  //通过rosparam 获取配置文件路径（不采用）
  //通过rosparam 获取所有配置信息
  std::string log_dir_;
  std::string device_Name_;
  std::string sensor_name_;
  std::string device_ip_;
  int intput_port_;
  int packet_size_;

  int packet_size; //数据包大小

  node.param("log_dir", log_dir_, std::string("/work/log/"));
  node.param("sensor_name", sensor_name_, std::string("velodyne"));
  node.param("device_name", device_Name_, std::string("lidar1"));
  node.param("device_ip", device_ip_, std::string("192.168.2.200"));
  node.param("intput_port", intput_port_, 2370);
  node.param("packet_size", packet_size_, 1206);

  char *home_path = getenv("HOME");
  ostringstream log_dir_stream;
  log_dir_stream.fill('0');
  log_dir_stream << home_path << log_dir_;

  SPDLOG_DEBUG("main log_dir={}", log_dir_stream.str());
  SPDLOG_DEBUG("main sensor_name_={}", sensor_name_);
  SPDLOG_DEBUG("main device_Name_={}", device_Name_);
  SPDLOG_DEBUG("main device_ip_={}", device_ip_);
  SPDLOG_DEBUG("main intput_port_={}", intput_port_);
  SPDLOG_DEBUG("main packet_size_={}", packet_size_);

  //////////////////建立UDP接收通道，并启动数据处理///////////////////////

  boost::shared_ptr< UdpProcess > udp_(new UdpProcess());
  udp_->log_dir_     = log_dir_stream.str();
  udp_->sensor_name_ = sensor_name_;
  udp_->device_name_ = device_Name_;

  // radar data process class
  boost::shared_ptr< LidarDataProcess > lidar_(new LidarDataProcess(node));

  if (udp_->Initial(lidar_, device_ip_, intput_port_, packet_size_) == 1)
  {
    lidar_->start();
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  int pid = getpid();

  ros::Publisher pub_ = node.advertise< status_msgs::NodeStatus >("/node/node_status", 1, true);

  // ros::spin();
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Rate loop_rate(100);
  int icount = 0;
  while (ros::ok())
  {

    ros::spinOnce();
    icount++;
    if (icount > 100)
    {
      // SPDLOG_DEBUG("main thread=[{}]", pthread_self());
      icount = 1;

      status_msgs::NodeStatus ns;
      ns.node_name = ros::this_node::getName();
      ns.node_pid  = pid;
      ns.state_num = 1;

      ns.header.frame_id = "/odom";
      ns.header.stamp    = ros::Time::now();

      pub_.publish(ns);
    }

    loop_rate.sleep();
  }

  ros::waitForShutdown();

  //销毁线程锁
  pthread_mutex_destroy(&list_mutex_raw);
  pthread_mutex_destroy(&list_mutex_cut);
  pthread_mutex_destroy(&location_mutex);
  return 0;
}