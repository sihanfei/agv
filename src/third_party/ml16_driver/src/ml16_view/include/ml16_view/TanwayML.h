/*
 * TanwayML.h
 *
 *  Created on: 11-01-2018
 *  Author: Elodie Shan
*/


#ifndef TANWAYML_H_
#define TANWAYML_H_
#include <ros/ros.h> //generic C++ stuff
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h> //ROS message type to publish a pointCloud
#include <pcl_ros/point_cloud.h> //use these to convert between PCL and ROS datatypes
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>   
#include <strings.h>
#include <sys/stat.h>


using namespace std;

class TanwayML
{
public:
  TanwayML();

  virtual ~TanwayML();

  void getParam(ros::NodeHandle node, ros::NodeHandle nh_private);

  float verticalChannels26[16] = { -13.0f, -11.27f,
 -9.53f, -7.80f, -6.07f, -4.33f, -2.60f, -0.87f, 0.87f,
 2.60f, 4.33f, 6.07f, 7.80f, 9.53f, 11.27f, 13.0f};

  float verticalChannels11[16] = { -5.50f, -4.77f,
 -4.03f, -3.30f, -2.57f, -1.83f, -1.10f, -0.37f, 0.37f,
 1.10f, 1.83f, 2.57f, 3.30f, 4.03f, 4.77f, 5.50f};

  bool recordRawData = true;
  float horizonalPerAngle = 0.3f;
  int circleCounter = 0;
  float RA = (float) (3.14159265f / 180.0f);
  float filter_L = 0.01;
  float c = 2.99;
  float hAPre = -1.0f;
  int myUDPCount=0;

  std::string host = "192.168.111.204" ;
  std::string LiDARhost = "192.168.111.30" ;
  std::string frame_id = "TanwayML16" ;
  std::string topic = "/ml16_cloud" ;
  std::string Color = "Indoor" ;

  int port = 5600;
  int LiDARport = 5050;
  char SQ[17] = "";
  double StaticQuantityFirst[16]={0};
  float verticalChannels[16];
  int VerticleAngle = 11;
  double StartAngle = 2;
  double EndAngle = 350;
  bool needPublishCloud = true;
  bool transformCloud_Status = false;
  double trans_x = 0;
  double trans_y = 0;
  double trans_z = 0;
  double rotate_theta_xy = 0;
  double rotate_theta_xz = 0;
  double rotate_theta_yz = 0;
  bool GPS_Status = false;

  char buf[205];

  sensor_msgs::PointCloud2 ros_cloud; 
  ros::Publisher pubCloud;

  socklen_t sockfd, ret, addrlen;
  struct sockaddr_in saddr,caddr;

  pthread_t pid;
  pthread_attr_t pida;
  // @brief Data tansform
  int TwoHextoFourX(unsigned char x1,unsigned char x2);

  int HexToInt(unsigned char x1);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloudIn, double x, double y, double z, double rotate_theta_xy, double rotate_theta_xz, double rotate_theta_yz);  

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr process_XYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_clr_ptr,float verticalChannels[], char* buf,float hA,double* StaticQuantity,std::string Color);

  int publishCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_clr_ptr_n1);

  int Connect_Valid();

};


#endif
