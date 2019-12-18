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
//////////rslidar pointcloud////////////

#include <boost/foreach.hpp>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include "location_msgs/FusionDataInfo.h"
#include <perception_sensor_msgs/LidarPointCloud.h>
#include <rslidar_msgs/rslidarScan.h>
#include <sensor_msgs/PointCloud2.h>

#include "rawdata.h"

#include "ImageSegment_linh.h"
// 20191031 运动补偿
#include "location_interpolation.h"

using namespace std;
using namespace boost;

using namespace Eigen;
using namespace superg_agv::drivers;

#define NODE_NAME "rs_lidar_node"
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
  // rslidar_msgs::rslidarScan agv_scan_;
  rslidar_msgs::rslidarScan obj_scan_;

  location_msgs::FusionDataInfo start_location_;
  location_msgs::FusionDataInfo end_location_;

  std::vector< XYZShift > v_frame_points_shift_ver_;
  std::vector< double > v_nsec_;
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

  boost::shared_ptr< rslidar_rawdata::RawData > cut_data_;

  boost::shared_ptr< rslidar_rawdata::RawData > raw_data_;

  // list< rslidar_msgs::rslidarScan > list_raw_rslidarScan_;
  // rslidar_msgs::rslidarScan temp_raw_;

  std::list< lidar_info_struct > list_cut_rslidarScan_;
  lidar_info_struct temp_cut_;

  int packets_count;
  // int ttt;
  // int t1;
  int pub_raw_data_;
  int is_motion_compensation_;

  ImageSegment imageSegment;
  Eigen::Matrix4f Matrix4f_1_;
  // Eigen::Matrix4f Matrix4f_2_;

  location_msgs::FusionDataInfo temp_location_;
  //

  //帧 1帧75包 //包 1包12块 //块 1块2组  //组 16个点数据  //通道 每个激光器一通道
  // pcl::PointCloud< pcl::PointXYZI > frame_pcl_points;

  std::vector< double > bag_points_time_vec;

  std::vector< XYZShift > v_frame_points_shift_ver;
  int v_frame_points_shift_ver_offset;

  std::vector< XYZShift > bag_points_shift_ver;

  LocationMathInfo first_bag_location_info; //当一个扫描周期中 接收到第一包数据时的定位信息
  LocationMathInfo cur_location_info;       //定位回调触发后的当前定位信息
  LocationMathInfo last_location_info;      //上一组定位信息
  LocationMathInfo cur_location_coefficient;

  u_int64_t LocationMsg_count;
  u_int64_t cur_LocationMsg_count;

  int last_azimuth;

  bool isHandle;

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
};

LidarDataProcess::LidarDataProcess(ros::NodeHandle node)
    : nh_(node), cut_data_(new rslidar_rawdata::RawData()), raw_data_(new rslidar_rawdata::RawData())
{
  // ROS_DEBUG_ONCE("LidarDataProcess[1] Process ID: %d, thread ID %d\n", getpid(), syscall(__NR_gettid));
  nh_.param("frame_id", config_.frame_id, std::string("odom"));
  nh_.param("npackets", config_.npackets, 75);

  nh_.param("begin_cut_angle", config_.begin_cut_angle, 0);
  nh_.param("end_cut_angle", config_.end_cut_angle, 90);

  config_.begin_cut_angle = config_.begin_cut_angle * 100;
  config_.end_cut_angle   = config_.end_cut_angle * 100;

  nh_.param("raw_data_topic_", config_.raw_data_topic_, std::string("/null"));
  nh_.param("data_set_topic_", config_.data_set_topic_, std::string("/null"));

  nh_.param("pub_raw_data", pub_raw_data_, 0);
  nh_.param("is_motion_compensation", is_motion_compensation_, 0);

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

  cut_data_->loadConfigFile(nh_);
  // raw_data_->loadConfigFile(nh_);

  std::vector< std::string > v1 = string_split(config_.Matrix4f_1, ",");

  Matrix4f_1_ << strToNum< float >(v1.at(0)), strToNum< float >(v1.at(1)), strToNum< float >(v1.at(2)),
      strToNum< float >(v1.at(3)), strToNum< float >(v1.at(4)), strToNum< float >(v1.at(5)),
      strToNum< float >(v1.at(6)), strToNum< float >(v1.at(7)), strToNum< float >(v1.at(8)),
      strToNum< float >(v1.at(9)), strToNum< float >(v1.at(10)), strToNum< float >(v1.at(11)),
      strToNum< float >(v1.at(12)), strToNum< float >(v1.at(13)), strToNum< float >(v1.at(14)),
      strToNum< float >(v1.at(15));

  //  std::cout << Matrix4f_1_ << endl;

  location_sub_ = nh_.subscribe("/localization/fusion_msg", 1, &LidarDataProcess::recvFusionLocationCallback, this);

  temp_cut_.obj_scan_.packets.resize(config_.npackets);
  v_frame_points_shift_ver.resize(28800);

  LocationMsg_count     = 0;
  cur_LocationMsg_count = 0;

  last_azimuth = 0;
  isHandle     = true;
}

LidarDataProcess::~LidarDataProcess()
{
  // udp_.reset();
}
void LidarDataProcess::recvFusionLocationCallback(const location_msgs::FusionDataInfo::ConstPtr &location_msg)
{
  //  ROS_DEBUG_ONCE("FusionLocationCallback[1] Process ID: %d, thread ID %d\n", getpid(), syscall(__NR_gettid));
  ROS_DEBUG_ONCE("[%s]FusionLocationCallback Process[%d], Thread[%d]", ros::this_node::getName().c_str(),
                 ( int )getpid(), ( int )syscall(__NR_gettid));
  // ROS_DEBUG_ONCE("FusionLocationCallback[2] Process ID: %d, thread ID %d\n", getpid(), syscall(SYS_gettid));
  // SPDLOG_DEBUG("recvFusionLocationCallback thread=[{}]", pthread_self()); // boost::this_thread::get_id());
  ros::Time t = ros::Time::now();

  pthread_mutex_lock(&location_mutex);
  temp_location_ = *location_msg;

  cur_location_info.time = temp_location_.header.stamp.toSec();
  // cur_location_info.time  = t.toSec();
  cur_location_info.pos_x = temp_location_.pose.x;
  cur_location_info.pos_y = temp_location_.pose.y;
  cur_location_info.pos_z = temp_location_.pose.z;
  cur_location_info.yaw   = temp_location_.yaw;
  cur_location_info.pitch = temp_location_.pitch;
  cur_location_info.roll  = temp_location_.roll;

  //增加速度判断，当速度平方低于1.0的时候不使用速度补偿
  double agv_speed = temp_location_.velocity.linear.x * temp_location_.velocity.linear.x +
                     temp_location_.velocity.linear.y * temp_location_.velocity.linear.y;
  int i = 0;
  // i     = updateLineLocationCoefficient(last_location_info, cur_location_info, cur_location_coefficient);

  if (agv_speed < 1.0)
  {
    i = updateLineLocationCoefficient(cur_location_info, cur_location_info, cur_location_coefficient);
    // ROS_INFO("d_t %lf Cur p<%lf %lf %lf> a<%lf %lf %lf> las p<%lf %lf %lf> a<%lf %lf %lf> Coe p<%lf %lf %lf> a<%lf
    // %lf "
    //          "%lf> Tip %d",
    //          cur_location_info.time - last_location_info.time, cur_location_info.pos_x, cur_location_info.pos_y,
    //          cur_location_info.pos_z, cur_location_info.yaw, cur_location_info.pitch, cur_location_info.roll,
    //          last_location_info.pos_x, last_location_info.pos_y, last_location_info.pos_z, last_location_info.yaw,
    //          last_location_info.pitch, last_location_info.roll, cur_location_coefficient.pos_x,
    //          cur_location_coefficient.pos_y, cur_location_coefficient.pos_z, cur_location_coefficient.yaw,
    //          cur_location_coefficient.pitch, cur_location_coefficient.roll, cur_location_coefficient.updata_tip);
  }
  else
  {
    i = updateLineLocationCoefficient(last_location_info, cur_location_info, cur_location_coefficient);
    // ROS_WARN("d_t %lf Cur p<%lf %lf %lf> a<%lf %lf %lf> las p<%lf %lf %lf> a<%lf %lf %lf> Coe p<%lf %lf %lf> a<%lf
    // %lf "
    //          "%lf> Tip %d",
    //          cur_location_info.time - last_location_info.time, cur_location_info.pos_x, cur_location_info.pos_y,
    //          cur_location_info.pos_z, cur_location_info.yaw, cur_location_info.pitch, cur_location_info.roll,
    //          last_location_info.pos_x, last_location_info.pos_y, last_location_info.pos_z, last_location_info.yaw,
    //          last_location_info.pitch, last_location_info.roll, cur_location_coefficient.pos_x,
    //          cur_location_coefficient.pos_y, cur_location_coefficient.pos_z, cur_location_coefficient.yaw,
    //          cur_location_coefficient.pitch, cur_location_coefficient.roll, cur_location_coefficient.updata_tip);
  }
  last_location_info = cur_location_info;
  LocationMsg_count++;

  pthread_mutex_unlock(&location_mutex);

  // ros::Time tt = ros::Time::now();
  // SPDLOG_DEBUG("recvFusionLocationCallback time=[{}] i=[{}]", (tt - t).toNSec() / 1000000.0, i);
}
void LidarDataProcess::OnUdpProcessCallBack(unsigned char *data, int len, struct sockaddr_in addr_)
{
  ROS_DEBUG_ONCE("[%s]OnUdpProcessCallBack Process[%d], Thread[%d]", ros::this_node::getName().c_str(), ( int )getpid(),
                 ( int )syscall(__NR_gettid));

  //接收到76包数据后，组合成一帧数据 放入list中，如果list非空，清空list后放入，保证点云处理线程每次都处理当前最新数据
  ros::Time t           = ros::Time::now();
  double bag_start_time = t.toSec();

  //接收到一包后 获取角度 比较是需要裁切掉，如果是有效角度，就放到 切割数据 rslidar_msgs::rslidarScan temp_cut_ 里面
  // int azimuth = 256 * data[44] + data[45];

  // if (azimuth < last_azimuth)
  // {
  //   SPDLOG_DEBUG("packets[{}] azimuth[{}] last_azimuth[{}]", packets_count, azimuth, last_azimuth);
  //   packets_count = 0;
  // }
  // else
  // {
  //   packets_count++;
  // }
  // last_azimuth = azimuth;

  // rslidar_msgs::rslidarPacket tmp_packet;
  temp_cut_.obj_scan_.packets[packets_count].stamp = t;
  memcpy(&temp_cut_.obj_scan_.packets[packets_count].data[0], ( char * )data, len);
  packets_count++;

  pthread_mutex_lock(&location_mutex);
  if (packets_count == 1 || packets_count == config_.npackets)
  {
    if (packets_count == 1)
    {
      first_bag_location_info         = cur_location_info;
      temp_cut_.start_location_       = temp_location_;
      v_frame_points_shift_ver_offset = 0;
      //      v_frame_points_shift_ver.resize(0);
    }
    else
    {
      temp_cut_.end_location_ = temp_location_;
    }
  }

  //收集运动补偿数据
  //增加了按组修正的函数：
  // mathGroupsShift;
  // mathLidarGroupsTimeVec;
  //增加了度数取余及负数转正的函数
  // radianMod
  //修改了度数减法借位错误
  // subRadian

  if (is_motion_compensation_ > 0)
  {
    if (is_motion_compensation_ == 1)
    {
      mathLidarPointsTimeVec(bag_start_time, 16 * 24, bag_points_time_vec);
      mathPointsShift(first_bag_location_info, cur_location_info, cur_location_coefficient, bag_points_time_vec,
                      bag_points_shift_ver);
    }
    else
    {
      mathLidarGroupsTimeVec(bag_start_time, 24, bag_points_time_vec);
      mathGroupsShift(first_bag_location_info, cur_location_info, cur_location_coefficient, bag_points_time_vec,
                      bag_points_shift_ver);
    }

    v_frame_points_shift_ver_offset =
        addBagPointsShiftVer2Frame(bag_points_shift_ver, v_frame_points_shift_ver, v_frame_points_shift_ver_offset);
    //
  }
  pthread_mutex_unlock(&location_mutex);
  temp_cut_.v_nsec_.push_back((ros::Time::now() - t).toNSec() / 1000000.0);

  //最后一包
  if (packets_count >= config_.npackets)
  {
    // SPDLOG_DEBUG("size[{}]", temp_cut_.v_nsec_.size());
    packets_count = 0;
    // t1            = 0;
    // SPDLOG_DEBUG("size[{}]", temp_cut_.packets.size());
    temp_cut_.obj_scan_.header.stamp    = temp_cut_.obj_scan_.packets.front().stamp;
    temp_cut_.obj_scan_.header.frame_id = config_.frame_id;

    temp_cut_.v_frame_points_shift_ver_ = v_frame_points_shift_ver;

    // SPDLOG_DEBUG("temp_cut_[{}]", temp_cut_.header.stamp.toSec());

    //申请线程锁

    pthread_mutex_lock(&list_mutex_cut);
    if (isHandle)
    {
      list_cut_rslidarScan_.push_back(temp_cut_);
    }
    isHandle = !isHandle;

    // temp_cut_.obj_scan_.packets.clear();
    // temp_cut_.v_frame_points_shift_ver_.clear();
    temp_cut_.v_nsec_.clear();

    //释放线程锁
    pthread_mutex_unlock(&list_mutex_cut);

    // if (pub_raw_data_ == 1)
    // {
    //   temp_raw_.header.stamp    = temp_raw_.packets.front().stamp;
    //   temp_raw_.header.frame_id = config_.frame_id;
    //   //申请线程锁
    //   pthread_mutex_lock(&list_mutex_raw);
    //   list_raw_rslidarScan_.push_back(temp_raw_);
    //   temp_raw_.packets.clear();
    //   //释放线程锁
    //   pthread_mutex_unlock(&list_mutex_raw);
    // }
  }
}

//发布话题
void LidarDataProcess::lidar_data_cut()
{
  // ROS_DEBUG_ONCE("lidar_data_cut[1] Process ID: %d, thread ID %d\n", getpid(), syscall(__NR_gettid));
  ROS_DEBUG_ONCE("[%s]lidar_data_cut Process[%d], Thread[%d]", ros::this_node::getName().c_str(), ( int )getpid(),
                 ( int )syscall(__NR_gettid));

  ros::Publisher lidar_cut_point_pub_ =
      nh_.advertise< perception_sensor_msgs::LidarPointCloud >(config_.data_set_topic_.data(), 1);
  ros::Publisher raw_point_cloud_pub_;
  if (pub_raw_data_ == 1)
    raw_point_cloud_pub_ = nh_.advertise< sensor_msgs::PointCloud2 >(config_.raw_data_topic_.data(), 1);

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    loop_rate.sleep();
    if (!list_cut_rslidarScan_.empty())
    {
      perception_sensor_msgs::LidarPointCloud temp_lidar_Point_set;
      std::vector< XYZShift > temp_frame_points_shift_ver_;
      // begin time
      ros::Time t1 = ros::Time::now();

      std::ostringstream oss;

      // SPDLOG_DEBUG("thread[{}] list_cut_ size[{}]", pthread_self(), list_cut_rslidarScan_.size());
      rslidar_msgs::rslidarScan cut_;
      //切车体后的数据包

      //申请线程锁
      pthread_mutex_lock(&list_mutex_cut);
      cut_                                = list_cut_rslidarScan_.front().obj_scan_;
      temp_lidar_Point_set.location_start = list_cut_rslidarScan_.front().start_location_;
      temp_lidar_Point_set.location_end   = list_cut_rslidarScan_.front().end_location_;
      //运动补偿计算数据
      temp_frame_points_shift_ver_ = list_cut_rslidarScan_.front().v_frame_points_shift_ver_;

      // double t_d = 0.0;
      // for (size_t i = 0; i < list_cut_rslidarScan_.front().v_nsec_.size(); ++i)
      // {
      //   t_d += list_cut_rslidarScan_.front().v_nsec_[i];
      //   cout << fixed << setprecision(8) << list_cut_rslidarScan_.front().v_nsec_[i] << std::endl;
      // }
      // cout << fixed << setprecision(8) << t_d << std::endl;
      // 输出元素

      list_cut_rslidarScan_.pop_front();
      //释放线程锁
      pthread_mutex_unlock(&list_mutex_cut);

      // copy data time
      ros::Time t2 = ros::Time::now();
      oss << "[packets=" << cut_.packets.size() << "] copydata[" << ((t2 - t1).toNSec() / 1000000.0) << "] ";

      //获取点云数据
      // cut data --> pcl
      pcl::PointCloud< pcl::PointXYZI >::Ptr rs_cut_points(new pcl::PointCloud< pcl::PointXYZI >);
      rs_cut_points->header.stamp    = pcl_conversions::toPCL(cut_.header).stamp;
      rs_cut_points->header.frame_id = cut_.header.frame_id;
      rs_cut_points->clear();
      rs_cut_points->height   = 16;
      rs_cut_points->width    = 24 * ( int )cut_.packets.size();
      rs_cut_points->is_dense = false;
      rs_cut_points->resize(rs_cut_points->height * rs_cut_points->width);

      cut_data_->block_num = 0;
      for (size_t i = 0; i < cut_.packets.size(); i++)
      {
        // SPDLOG_ERROR("rs_cut_points i {}", i);
        cut_data_->unpack(cut_.packets[i], rs_cut_points);
      }

      // data->pcl:XYZIR
      ros::Time t3 = ros::Time::now();
      oss << "data->pcl:XYZIR[" << ((t3 - t2).toNSec() / 1000000.0) << "] ";

      // pcl:XYZIR->pcl:XYZI
      // imageSegment.setMinAndMaxAngle(0.0,0.0);
      imageSegment.setPacketsSize(cut_.packets.size());
      imageSegment.setPointCloud(*rs_cut_points);
      ros::Time t4 = ros::Time::now();
      oss << "pcl:XYZIR->pcl:XYZI[" << ((t4 - t3).toNSec() / 1000000.0) << "] ";

      //转换坐标系  tf lidar1->lidar2
      imageSegment.tf(Matrix4f_1_);

      ros::Time t5 = ros::Time::now();
      oss << "tf lidar->car[" << ((t5 - t4).toNSec() / 1000000.0) << "] ";

      //运行补偿
      // 75包数据后执行运动补偿 需要把原始点云转换到车体坐标系下 motion compensation
      if (is_motion_compensation_ > 0)
      {
        if (cur_LocationMsg_count < LocationMsg_count)
        {
          doLidarPointsCorrect(imageSegment.full_cloud_, temp_frame_points_shift_ver_);
          cur_LocationMsg_count = LocationMsg_count;
        }
      }

      ros::Time t51 = ros::Time::now();
      oss << "motion compensation[" << ((t51 - t5).toNSec() / 1000000.0) << "] ";
      //分割

      imageSegment.segment();
      pcl::PointCloud< pcl::PointXYZI > filtered = imageSegment.getFilteredCloud();
      pcl::PointCloud< pcl::PointXYZI > ground   = imageSegment.getGround();
      //       pcl::PointCloud< pcl::PointXYZI > ground = imageSegment.full_cloud_;

      // segment
      ros::Time t6 = ros::Time::now();
      oss << "segment[" << ((t6 - t51).toNSec() / 1000000.0) << "] ";

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
        {
          agv_cut.points.push_back(point);
        }
      }
      ros::Time t7 = ros::Time::now();
      oss << "cut agv[" << ((t7 - t6).toNSec() / 1000000.0) << "] ";

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

      ///////////////raw pc2
      if (pub_raw_data_ == 1)
      {
        sensor_msgs::PointCloud2 laserMsg;

        pcl::toROSMsg(*rs_cut_points, laserMsg);
        // pcl::toROSMsg(imageSegment.full_cloud_, laserMsg);
        laserMsg.header.stamp    = cut_.header.stamp;
        laserMsg.header.frame_id = cut_.header.frame_id;

        raw_point_cloud_pub_.publish(laserMsg);
      }

      // toROSMsg
      ros::Time t8 = ros::Time::now();
      oss << "toROSMsg[" << ((t8 - t7).toNSec() / 1000000.0) << "] ";

      // pub
      if (lidar_cut_point_pub_.getNumSubscribers() > 0) // no one listening?// avoid much work
      {
        lidar_cut_point_pub_.publish(temp_lidar_Point_set);
      }

      // ros::Time t9 = ros::Time::now();
      // oss << "pub obj pc2[" << ((t9 - t8).toNSec() / 1000000.0) << "] ";

      // ros::Time t10 = ros::Time::now();
      // oss << "pub raw pc2[" << ((t10 - t9).toNSec() / 1000000.0) << "] ";

      // oss << "total[" << ((t10 - t1).toNSec() / 1000000.0) << "]";
      // if (((t10 - t1).toNSec() / 1000000.0) > 100.0)
      // {
      //   SPDLOG_ERROR("{}", oss.str());
      // }
      // else
      //   SPDLOG_DEBUG("{}", oss.str());

    } // list_rslidarScan_.empty()
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
  // SPDLOG_DEBUG("lidar_raw_pl2 thread=[{}]", pthread_self());
  // ros::Publisher raw_point_cloud_pub_ = nh_.advertise< sensor_msgs::PointCloud2 >(config_.raw_data_topic_.data(),
  // 1);

  // ros::Rate loop_rate(100);
  // while (ros::ok())
  // {
  //   loop_rate.sleep();
  //   if (!list_raw_rslidarScan_.empty())
  //   {

  //     rslidar_msgs::rslidarScan raw_;
  //     //切车体后的数据包

  //     //申请线程锁
  //     pthread_mutex_lock(&list_mutex_raw);
  //     raw_ = list_raw_rslidarScan_.front();
  //     list_raw_rslidarScan_.pop_front();
  //     //释放线程锁
  //     pthread_mutex_unlock(&list_mutex_raw);

  //     // SPDLOG_DEBUG("thread[{}] list_raw size[{}] packets.size[{}]", pthread_self(),
  //     list_raw_rslidarScan_.size(),
  //     //              raw_.packets.size());
  //     //获取点云数据
  //     // data --> pcl

  //     pcl::PointCloud< pcl::PointXYZI >::Ptr outPoints(new pcl::PointCloud< pcl::PointXYZI >);
  //     outPoints->header.stamp    = pcl_conversions::toPCL(raw_.header).stamp;
  //     outPoints->header.frame_id = raw_.header.frame_id;
  //     outPoints->clear();
  //     outPoints->height   = 16;
  //     outPoints->width    = 24 * ( int )raw_.packets.size();
  //     outPoints->is_dense = false;
  //     outPoints->resize(outPoints->height * outPoints->width);

  //     raw_data_->block_num = 0;
  //     for (size_t i = 0; i < raw_.packets.size(); ++i)
  //     {
  //       raw_data_->unpack(raw_.packets[i], outPoints);
  //     }
  //     sensor_msgs::PointCloud2 laserMsg;
  //     pcl::toROSMsg(*outPoints, laserMsg);

  //     laserMsg.header.stamp    = raw_.header.stamp;
  //     laserMsg.header.frame_id = raw_.header.frame_id;

  //     raw_point_cloud_pub_.publish(laserMsg);

  //   } // if .empty()
  // }   // while (ros::ok())
}

class LidarDifopDataProcess : public UdpProcessCallBack
{
public:
  LidarDifopDataProcess(ros::NodeHandle node);
  ~LidarDifopDataProcess();
  void OnUdpProcessCallBack(unsigned char *data, int len, struct sockaddr_in addr_);

private:
  ros::NodeHandle nh_;
  ros::Publisher difop_output_;
  std::string output_difop_topic_;
};
LidarDifopDataProcess::LidarDifopDataProcess(ros::NodeHandle node) : nh_(node)
{
  //  ROS_DEBUG_ONCE("LidarDifopDataProcess[1] Process ID: %d, thread ID %d\n", getpid(), syscall(__NR_gettid));
  // ROS_DEBUG_ONCE("[%s]LidarDifopDataProcess Process[%d], Thread[%d]", ros::this_node::getName().c_str(),
  //                ( int )getpid(), ( int )syscall(__NR_gettid));
  nh_.param("output_difop_topic", output_difop_topic_, std::string("rslidar_packets_difop"));
  SPDLOG_DEBUG("output_difop_topic[{}]", output_difop_topic_);
  difop_output_ = node.advertise< rslidar_msgs::rslidarPacket >(output_difop_topic_, 1);
}

LidarDifopDataProcess::~LidarDifopDataProcess()
{
}
void LidarDifopDataProcess::OnUdpProcessCallBack(unsigned char *data, int len, struct sockaddr_in addr_)
{

  ROS_DEBUG_ONCE("[%s]LidarDifopDataProcess::OnUdpProcessCallBack| Process[%d], Thread[%d]",
                 ros::this_node::getName().c_str(), ( int )getpid(), ( int )syscall(__NR_gettid));

  rslidar_msgs::rslidarPacket tmp_packet;

  tmp_packet.stamp = ros::Time::now();
  // SPDLOG_DEBUG("tmp_packet stamp[{}]", tmp_packet.stamp.toSec());
  memcpy(&tmp_packet.data[0], ( char * )data, len);
  difop_output_.publish(tmp_packet);
}

int main(int argc, char *argv[])
{
  SPDLOG_DEBUG("ROS node is start, name is [{}], file name is {}", NODE_NAME, argv[0]);

  ros::init(argc, argv, NODE_NAME); // node name

  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

  ROS_DEBUG_ONCE("[%s] Process[%d], Thread[%d]", ros::this_node::getName().c_str(), ( int )getpid(),
                 ( int )syscall(__NR_gettid));

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
  int difop_port_;
  int packet_size_;

  int packet_size; //数据包大小

  node.param("log_dir", log_dir_, std::string("/work/log/"));
  node.param("sensor_name", sensor_name_, std::string("rslidar"));
  node.param("device_name", device_Name_, std::string("lidar1"));
  node.param("device_ip", device_ip_, std::string("192.168.2.200"));
  node.param("intput_port", intput_port_, 2370);
  node.param("difop_port", difop_port_, 7110);
  node.param("packet_size", packet_size_, 1248);

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

  //////////////////建立difop UDP接收通道，并启动数据处理///////////////////////

  boost::shared_ptr< UdpProcess > difop_udp_(new UdpProcess());
  difop_udp_->log_dir_     = log_dir_stream.str();
  difop_udp_->sensor_name_ = sensor_name_;
  difop_udp_->device_name_ = device_Name_;

  // radar data process class
  boost::shared_ptr< LidarDifopDataProcess > difop_lidar_(new LidarDifopDataProcess(node));

  if (difop_udp_->Initial(difop_lidar_, device_ip_, difop_port_, packet_size_) == 1)
  {
    // lidar_->start();
  }

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