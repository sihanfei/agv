/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *	Copyright (C) 2017, Robosense, Tony Zhang
 *
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  Input classes for the RSLIDAR RS-16 3D LIDAR:
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump
 */

#ifndef __RSLIDAR_INPUT_H_
#define __RSLIDAR_INPUT_H_

#include <ros/ros.h>

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <pcap.h>
#include <poll.h>
#include <ros/ros.h>
#include <rslidar_msgs/rslidarPacket.h>
#include <sensor_msgs/TimeReference.h>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <string>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>

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

namespace rslidar_driver
{

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
static uint16_t MSOP_DATA_PORT_NUMBER  = 6699; // rslidar default data port on PC
static uint16_t DIFOP_DATA_PORT_NUMBER = 7788; // rslidar default difop data port on PC
                                               /**
                                                *  从在线的网络数据或离线的网络抓包数据（pcap文件）中提取出lidar的原始数据，即packet数据包
                                                * @brief The Input class,
                                                    *
                                                    * @param private_nh  一个NodeHandled,用于通过节点传递参数
                                                    * @param port
                                                    * @returns 0 if successful,
                                                    *          -1 if end of file
                                                    *          >0 if incomplete packet (is this possible?)
                                                */
class Input
{
public:
  Input(ros::NodeHandle private_nh, uint16_t port);

  virtual ~Input()
  {
  }

  virtual int getPacket(rslidar_msgs::rslidarPacket *pkt, const double time_offset) = 0;

  int getRpm(void);
  int getReturnMode(void);
  bool getUpdateFlag(void);
  void clearUpdateFlag(void);

  std::string log_dir_;
  std::string device_name_;
  std::string sensor_name_;
  boost::shared_ptr< UdpData_Output > udplog_;

protected:
  ros::NodeHandle private_nh_;
  uint16_t port_;
  std::string devip_str_;
  int cur_rpm_;
  int return_mode_;
  bool npkt_update_flag_;
};

/** @brief Live rslidar input from socket. */
class InputSocket : public Input
{
public:
  InputSocket(ros::NodeHandle private_nh, uint16_t port = MSOP_DATA_PORT_NUMBER);

  virtual ~InputSocket();

  virtual int getPacket(rslidar_msgs::rslidarPacket *pkt, const double time_offset);

private:
private:
  int sockfd_;
  in_addr devip_;
};
}
#endif
