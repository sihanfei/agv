#ifndef TCP_PROCESS_H
#define TCP_PROCESS_H

#include "pthread.h"

#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <netinet/in.h>
#include <pthread.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <fstream>
#include <iomanip>  // std::setfill, std::setw
#include <iostream> // std::cout, std::ios
#include <sstream>  // std::ostringstream

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

#include <sys/stat.h>
#include <sys/types.h>

#include <poll.h>

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include "spdlog/logger.h"

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

#include "spdlog/async.h"

#include "spdlog/fmt/bin_to_hex.h"

using namespace std;

// pthread_mutex_t rec_mutex;        //创建线程锁
// pthread_mutex_lock(&rec_mutex);     //加锁

// pthread_mutex_unlock(&rec_mutex);    //解锁

class TcpData_Output
{
public:
  TcpData_Output(string dir_path, string sensor_name, string device_name, string device_ip, uint32_t m6_rec_port);
  ~TcpData_Output();

  string base_dir_path_; //日志目录
  string sensor_name_;   //传感器名称
  string device_name_;   //设备名称
  string device_ip_;     //设备IP
  uint32_t M6_rec_port_; // M6接收端口

  std::string logger_name_;

  string log_file_name;                        // log文件名
  std::shared_ptr< spdlog::logger > my_logger; //创建的logger指针

  // std::shared_ptr< spdlog::logger > console;

  bool log_file_created; // Log文件已创建标志，以免有进程访问了不存在的Log file
  bool logger_droped;    //循环创建log文件时使用，此测试程序中暂未使用

  bool create_log_file(string dir_path);
  void destroy_logger();

  void write_log(uint8_t *buf, int len);

  int my_mkdir(string muldir, mode_t mode);

  bool MinHasChanged();

  bool HourHasChanged();

  int32_t g_main_min;

  int32_t g_main_hour;
};

class TcpProcessCallBack
{
  //回调类，这里自定义所有回调函数
public:
  virtual void OnTcpProcessCallBack(unsigned char *data_, int len_, struct sockaddr_in addr_) = 0;
  virtual void OnTcpStatusCallBack(int istatus, struct sockaddr_in addr_)                     = 0;
};

class TcpProcess
{
public:
  TcpProcess();
  ~TcpProcess();

  int Initial(boost::shared_ptr< TcpProcessCallBack > p_DLCallBack, const std::string dev_ip, const int intput_port,
              const int tcp_data_len);

  void recTcpInfo();
  void sendTcpInfo(unsigned char *data, int datalen);

  std::string log_dir_;
  std::string device_name_;
  std::string sensor_name_;

private:
  int initTcp();

  boost::shared_ptr< TcpData_Output > tcplog_;

  TcpProcessCallBack *m_DLCallBack;
  boost::shared_ptr< TcpProcessCallBack > p_DLCallBack_;
  pthread_t m_threadID;

  int sockfd_;

  std::string devip_str_;
  int server_port_;
  struct sockaddr_in server_ip_;
  socklen_t addr_len_;

  int isOpen_;

  int tcp_data_size_;
};
#endif // TCP_PROCESS_H