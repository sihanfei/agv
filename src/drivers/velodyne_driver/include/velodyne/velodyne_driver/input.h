#ifndef __VELODYNE_DRIVER_INPUT_H
#define __VELODYNE_DRIVER_INPUT_H

#include <netinet/in.h>
#include <stdio.h>
#include <unistd.h>

#include "velodyne_msgs/VelodynePacket.h"
#include <ros/ros.h>

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include "spdlog/logger.h"

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

#include "spdlog/async.h"

#include "spdlog/fmt/bin_to_hex.h"

#include <fstream>
#include <iomanip>  // std::setfill, std::setw
#include <iostream> // std::cout, std::ios
#include <sstream>  // std::ostringstream

using namespace std;

namespace velodyne_driver
{
static uint16_t DATA_PORT_NUMBER     = 2368;
static uint16_t POSITION_PORT_NUMBER = 8308;

class UdpData_Output
{
public:
  UdpData_Output(std::string dir_path, std::string sensor_name, std::string device_name, std::string device_ip,
                 uint32_t m6_rec_port);
  ~UdpData_Output();

  std::string base_dir_path_; //日志目录
  std::string sensor_name_;   //传感器名称
  std::string device_name_;   //设备名称
  std::string device_ip_;     //设备IP
  uint32_t M6_rec_port_;      // M6接收端口

  std::string logger_name_;

  std::string log_file_name;                   // log文件名
  std::shared_ptr< spdlog::logger > my_logger; //创建的logger指针

  // std::shared_ptr< spdlog::logger > console;

  bool log_file_created; // Log文件已创建标志，以免有进程访问了不存在的Log file
  bool logger_droped;    //循环创建log文件时使用，此测试程序中暂未使用

  bool create_log_file(std::string dir_path);
  void destroy_logger();

  void write_log(uint8_t *buf, int len);

  int my_mkdir(std::string muldir, mode_t mode);

  bool MinHasChanged();

  bool HourHasChanged();

  int32_t g_main_min;

  int32_t g_main_hour;

  int islog;

  ros::NodeHandle pn;
};

class Input
{
public:
  Input(ros::NodeHandle &node, uint16_t port);
  virtual ~Input()
  {
  }

  virtual int getPacket(velodyne_msgs::VelodynePacket *ptk, const double time_offset) = 0;

  std::string log_dir_;
  std::string device_name_;
  std::string sensor_name_;
  boost::shared_ptr< UdpData_Output > udplog_;

protected:
  ros::NodeHandle private_nh_;
  uint16_t port_;
  std::string devip_str_;
};

class InputSocket : public Input
{
public:
  InputSocket(ros::NodeHandle &node, uint16_t port = DATA_PORT_NUMBER);
  virtual ~InputSocket();

  virtual int getPacket(velodyne_msgs::VelodynePacket *pkt, const double time_offset);
  void setDeviceIP(const std::string &ip);

private:
  int sockfd_;
  in_addr devip_;
};
}

#endif
