#include <ros/ros.h>

#include <arpa/inet.h>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/predef/other/endian.h>
#include <boost/thread.hpp>

#include <iostream>
#include <math.h>
#include <netinet/in.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>     //use these to convert between PCL and ROS datatypes
#include <sensor_msgs/PointCloud2.h> //ROS message type to publish a pointCloud
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
//////////////////////////////////////////////////////////////

#include "ml16_view/TWColor.h"

#include "common_functions.h"
#include "glog_helper.h"
#include "udp_process.h"

#include <list>
#include <map>
#include <thread>

#include <yaml-cpp/yaml.h>

#include <sys/syscall.h>
#include <unistd.h> //getpid()

#define TANWAY_YAML_FILE_PATH "/work/superg_agv/src/drivers/tanway_lidar_driver/cfg/"
#define TANWAY_YAML_FILE_NAME "config.yaml"

using namespace std;

#define NODE_NAME "TW_m16_radar_node"

struct TanWay_Device
{
  std::string device_name;
  bool device_enable;
  std::string device_ip;
  int intput_port;
  std::string topic_name_1;
  double StartAngle;
  double EndAngle;
  double StaticQuantityFirst[16];
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
void operator>>(const YAML::Node &node, TanWay_Device &tanwayDevice)
{
  node["device_name"] >> tanwayDevice.device_name;
  node["device_enable"] >> tanwayDevice.device_enable;
  node["device_ip"] >> tanwayDevice.device_ip;
  node["intput_port"] >> tanwayDevice.intput_port;
  node["topic_name_1"] >> tanwayDevice.topic_name_1;
  // node["StaticQuantityFirst"] >> tanwayDevice.StaticQuantityFirst;
  for (size_t i = 0; i < 16; i++)
  {
    node["StaticQuantityFirst"][i] >> tanwayDevice.StaticQuantityFirst[i];
  }
}

class TWLidarDataProcess : public UdpProcessCallBack
{
public:
  float verticalChannels26[16] = {-13.0f, -11.27f, -9.53f, -7.80f, -6.07f, -4.33f, -2.60f, -0.87f,
                                  0.87f,  2.60f,   4.33f,  6.07f,  7.80f,  9.53f,  11.27f, 13.0f};

  float verticalChannels11[16] = {-5.50f, -4.77f, -4.03f, -3.30f, -2.57f, -1.83f, -1.10f, -0.37f,
                                  0.37f,  1.10f,  1.83f,  2.57f,  3.30f,  4.03f,  4.77f,  5.50f};

  bool recordRawData      = true;
  float horizonalPerAngle = 0.3f;
  int circleCounter       = 0;
  float RA                = ( float )(3.14159265f / 180.0f);
  float filter_L          = 0.01;
  float c                 = 2.99;
  float hAPre             = -1.0f;
  int myUDPCount          = 0;

  // std::string host      = "192.168.2.200";
  // std::string LiDARhost = "192.168.2.101";
  std::string frame_id = "odom";
  std::string topic    = "/ml16_cloud";
  std::string Color    = "Indoor";

  // int port                       = 5600;
  // int LiDARport                  = 5050;

  char SQ[17]                    = "";
  double StaticQuantityFirst[16] = {0};
  float verticalChannels[16];
  int VerticleAngle          = 11;
  double StartAngle          = 2;
  double EndAngle            = 350;
  bool needPublishCloud      = true;
  bool transformCloud_Status = false;
  double trans_x             = 0;
  double trans_y             = 0;
  double trans_z             = 0;
  double rotate_theta_xy     = 3.14;
  double rotate_theta_xz     = 3.14;
  double rotate_theta_yz     = 3.14;
  bool GPS_Status            = false;

  // sensor_msgs::PointCloud2 ros_cloud;
  // ros::Publisher pubCloud;

  // socklen_t sockfd, ret, addrlen;
  // struct sockaddr_in saddr, caddr;

  // pthread_t pid;
  // pthread_attr_t pida;
  // @brief Data tansform
public:
  TWLidarDataProcess(TanWay_Device &TanWay_Device_info);
  virtual ~TWLidarDataProcess();
  // void getParam(ros::NodeHandle node, ros::NodeHandle nh_private);

  int TwoHextoFourX(unsigned char x1, unsigned char x2);

  int HexToInt(unsigned char x1);

  pcl::PointCloud< pcl::PointXYZRGB >::Ptr transformCloud(pcl::PointCloud< pcl::PointXYZRGB >::Ptr pPointCloudIn,
                                                          double x, double y, double z, double rotate_theta_xy,
                                                          double rotate_theta_xz, double rotate_theta_yz);

  pcl::PointCloud< pcl::PointXYZRGB >::Ptr process_XYZ(pcl::PointCloud< pcl::PointXYZRGB >::Ptr point_cloud_clr_ptr,
                                                       float verticalChannels[], unsigned char *buf, float hA,
                                                       double *StaticQuantity, std::string Color);

  int publishCloud(pcl::PointCloud< pcl::PointXYZRGB >::Ptr point_cloud_clr_ptr_n1);

  void OnUdpProcessCallBack(unsigned char *data, int len, struct sockaddr_in addr_);

  void TW_lidar_Sender();

  void start();

  //  void setTanWayParams(TanWay_Device &TanWay_Device_info);

  list< sensor_msgs::PointCloud2 > list_ros_cloud_;

  // pcl::PointCloud< pcl::PointXYZRGB >::Ptr point_cloud_ptr1(new pcl::PointCloud< pcl::PointXYZRGB >);
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr point_cloud_clr_ptr_n1_;
};
TWLidarDataProcess::TWLidarDataProcess(TanWay_Device &TanWay_Device_info)
{
  topic = TanWay_Device_info.topic_name_1;

  StartAngle = TanWay_Device_info.StartAngle;
  EndAngle   = TanWay_Device_info.EndAngle;

  // for (size_t i = 0; i < 16; i++)
  // {
  //   StaticQuantityFirst[i] = TanWay_Device_info.StaticQuantityFirst[i];
  // }
  memcpy(StaticQuantityFirst, TanWay_Device_info.StaticQuantityFirst, sizeof(TanWay_Device_info.StaticQuantityFirst));

  point_cloud_clr_ptr_n1_.reset(new pcl::PointCloud< pcl::PointXYZRGB >);

  if (VerticleAngle == 11)
  {
    memcpy(verticalChannels, verticalChannels11, sizeof(verticalChannels11));
  }
  else if (VerticleAngle == 26)
  {
    memcpy(verticalChannels, verticalChannels26, sizeof(verticalChannels11));
  }
}
TWLidarDataProcess::~TWLidarDataProcess(){};
// void TWLidarDataProcess::getParam()
// {

//   if (VerticleAngle == 11)
//   {
//     memcpy(verticalChannels, verticalChannels11, sizeof(verticalChannels11));
//   }
//   else if (VerticleAngle == 26)
//   {
//     memcpy(verticalChannels, verticalChannels26, sizeof(verticalChannels11));
//   }
// }

int TWLidarDataProcess::TwoHextoFourX(unsigned char x1, unsigned char x2)
{
  int a;
  char chbuf_high[0x20];
  sprintf(chbuf_high, "%02X%02X", x1, x2);
  sscanf(chbuf_high, "%04X", &a);
  return a;
}

int TWLidarDataProcess::HexToInt(unsigned char x1)
{
  int a;
  char chbuf_high[0x10];
  sprintf(chbuf_high, "%02X", x1);
  sscanf(chbuf_high, "%02X", &a);
  return a;
}

pcl::PointCloud< pcl::PointXYZRGB >::Ptr
TWLidarDataProcess::transformCloud(pcl::PointCloud< pcl::PointXYZRGB >::Ptr pPointCloudIn, double x, double y, double z,
                                   double theta_xy, double theta_xz, double theta_yz)
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << x, y, z;
  transform.rotate(Eigen::AngleAxisf(theta_xy, Eigen::Vector3f::UnitZ()));
  transform.rotate(Eigen::AngleAxisf(theta_xz, Eigen::Vector3f::UnitY()));
  transform.rotate(Eigen::AngleAxisf(theta_yz, Eigen::Vector3f::UnitX()));
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr pPointCloudOut(new pcl::PointCloud< pcl::PointXYZRGB >());
  pcl::transformPointCloud(*pPointCloudIn, *pPointCloudOut, transform);
  return pPointCloudOut;
}

pcl::PointCloud< pcl::PointXYZRGB >::Ptr
TWLidarDataProcess::process_XYZ(pcl::PointCloud< pcl::PointXYZRGB >::Ptr point_cloud_clr_ptr, float verticalChannels[],
                                unsigned char *buf, float hA, double *StaticQuantity, std::string Color)
{
  float myHA    = (hA - 16.5) * 2;
  double cos_hA = cos(myHA * RA);
  double sin_hA = sin(myHA * RA);
  int xx        = 1;
  while (xx <= 16)
  {
    int index    = 4 + (xx - 1) * 6;
    int seq      = HexToInt(buf[index]);
    int color    = HexToInt(buf[index + 5]);
    int hexToInt = TwoHextoFourX(buf[index + 2], buf[index + 1]);
    float L      = hexToInt * c * 32 / 10000.f / 2;
    L            = ((L - StaticQuantity[seq - 1]) > 0) ? (L - StaticQuantity[seq - 1]) : 0;
    if (L > 0 && L < 300)
    {
      float vA         = verticalChannels[seq - 1];
      double cos_vA_RA = cos(vA * RA);
      double x         = L * cos_vA_RA * cos_hA;
      double y         = L * cos_vA_RA * sin_hA;
      double z         = L * sin(vA * RA);

      pcl::PointXYZRGB basic_point;
      basic_point.x = x;
      basic_point.y = y;
      basic_point.z = z;
      TWColor twcolor;
      if (Color == "None")
      {
        uint32_t rgb    = twcolor.ConstColor();
        basic_point.rgb = *reinterpret_cast< float * >(&rgb);
      }
      if (Color == "Indoor")
      {
        uint32_t rgb    = twcolor.IndoorColor(L);
        basic_point.rgb = *reinterpret_cast< float * >(&rgb);
      }
      if (Color == "Outdoor")
      {
        uint32_t rgb    = twcolor.OutdoorColor(L);
        basic_point.rgb = *reinterpret_cast< float * >(&rgb);
      }
      point_cloud_clr_ptr->points.push_back(basic_point);
    }
    xx++;
  }
  return point_cloud_clr_ptr;
}

void TWLidarDataProcess::OnUdpProcessCallBack(unsigned char *data, int len, struct sockaddr_in addr_)
{

  //计算hA
  float hA = TwoHextoFourX(data[8], data[7]) / 100.0f;

  if ((StartAngle <= hA && hA <= EndAngle))
  {
    // ROS_INFO("[%s](StartAngle <= hA && hA <= EndAngle) hA = %f", ns.data(), hA);
    needPublishCloud = true;
    point_cloud_clr_ptr_n1_ =
        process_XYZ(point_cloud_clr_ptr_n1_, verticalChannels, data, hA, StaticQuantityFirst, Color);
  }

  if (hA > EndAngle && needPublishCloud)
  {
    // ROS_INFO("[%s]hA > EndAngle && needPublishCloud hA = %f", ns.data(), hA);
    point_cloud_clr_ptr_n1_->width           = ( int )point_cloud_clr_ptr_n1_->points.size();
    point_cloud_clr_ptr_n1_->height          = 1;
    point_cloud_clr_ptr_n1_->header.frame_id = frame_id;
    ROS_DEBUG("Publish   num: [%d]", ( int )point_cloud_clr_ptr_n1_->points.size());
    if (transformCloud_Status == true)
      point_cloud_clr_ptr_n1_ = transformCloud(point_cloud_clr_ptr_n1_, trans_x, trans_y, trans_z, rotate_theta_xy,
                                               rotate_theta_xz, rotate_theta_yz);

    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*point_cloud_clr_ptr_n1_, ros_cloud);

    if (GPS_Status == true)
    {
      int GPSmin  = HexToInt(data[100]);
      int GPSsec  = HexToInt(data[101]);
      int GPSusec = TwoHextoFourX(data[103], data[102]);
      time_t time_now;
      time_t GPStime;
      time(&time_now);
      tm *GPSlocaltime            = localtime(&time_now);
      GPSlocaltime->tm_min        = GPSmin;
      GPSlocaltime->tm_sec        = GPSsec;
      GPStime                     = mktime(GPSlocaltime);
      ros_cloud.header.stamp      = ros::Time(GPStime);
      ros_cloud.header.stamp.nsec = GPSusec * 1000000;
    }
    else
    {
      ros_cloud.header.stamp = ros::Time::now();
    }
    list_ros_cloud_.push_back(ros_cloud);

    point_cloud_clr_ptr_n1_->points.clear();

    /* pthread_create(&pid, &pida, clearVector, &point_cloud_clr_ptr_n1_->points);*/
    needPublishCloud = false;
  }

  if (hAPre != hA)
  {
    hAPre = hA;
  }
  return;
}

//发布话题
void TWLidarDataProcess::TW_lidar_Sender()
{
  ROS_INFO("process[%d] thread[%u] TW_lidar_Sender Start!", getpid(), ( unsigned int )pthread_self());

  ros::NodeHandle nh;

  ros::Publisher pubCloud = nh.advertise< sensor_msgs::PointCloud2 >(topic, 1);

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    loop_rate.sleep();
    if (!list_ros_cloud_.empty())
    {
      sensor_msgs::PointCloud2 ros_cloud = list_ros_cloud_.front();

      pubCloud.publish(ros_cloud);

      list_ros_cloud_.pop_front();

    } // list_ros_cloud_.empty()
  }   // while (ros::ok())
}
void TWLidarDataProcess::start()
{
  boost::thread thrd(boost::bind(&TWLidarDataProcess::TW_lidar_Sender, this));
  thrd.detach();
}

int main(int argc, char *argv[])
{

  ROS_INFO("ROS node is star, name is [%s], file name is %s", NODE_NAME, argv[0]);

  // GLogHelper gh(argv[0]);

  ros::init(argc, argv, NODE_NAME); // node name
  ros::NodeHandle nh;

  // glog
  GLogHelper gh(argv[0]);
  gh.setLogDirectory(argv[0]);

  // get yaml file path
  char *home_path           = getenv("HOME");
  char yaml_file_name[1024] = {0};
  sprintf(yaml_file_name, "%s%s%s", home_path, TANWAY_YAML_FILE_PATH, TANWAY_YAML_FILE_NAME);
  SUPERG_INFO << "yaml_file_name:" << yaml_file_name;

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  std::string file_name = yaml_file_name;
  YAML::Node doc;
  doc = YAML::LoadFile(file_name);
  SUPERG_INFO << "config_num: " << doc.size();
  SUPERG_INFO << "doc[sensor_name]: " << doc["sensor_name"];

  SUPERG_INFO << "doc[device_list]: " << doc["device_list"].size();

  ostringstream log_dir_stream;
  log_dir_stream.fill('0');
  log_dir_stream << home_path << doc["log_dir"].as< string >();
  SUPERG_INFO << "log_dir_: " << log_dir_stream.str();

  // udp class
  UdpProcess udp_[doc["device_list"].size()];

  // radar data process class
  boost::shared_ptr< TWLidarDataProcess > lidar_[doc["device_list"].size()];

  for (unsigned i = 0; i < doc["device_list"].size(); i++)
  {
    TanWay_Device TanWay_Device_;
    doc["device_list"][i] >> TanWay_Device_;
    // SUPERG_INFO << "device_name: " << TanWay_Device_.device_name;
    // SUPERG_INFO << "device_ip: " << TanWay_Device_.device_ip;
    // SUPERG_INFO << "device_enable: " << TanWay_Device_.device_enable;
    // SUPERG_INFO << "intput_port: " << TanWay_Device_.intput_port;
    // SUPERG_INFO << "topic_name_1: " << TanWay_Device_.topic_name_1;

    if (TanWay_Device_.device_enable)
    {
      udp_[i].log_dir_     = log_dir_stream.str();
      udp_[i].sensor_name_ = doc["sensor_name"].as< string >();
      udp_[i].device_name_ = TanWay_Device_.device_name;

      lidar_[i].reset(new TWLidarDataProcess(TanWay_Device_));

      if (udp_[i].Initial(lidar_[i], TanWay_Device_.device_ip, TanWay_Device_.intput_port, 205) == 1)
      {
        lidar_[i]->start();
      }
    }
  }
  /////////////////////////////////////////////////////////////////////////

  ros::spin();
  return 0;
}