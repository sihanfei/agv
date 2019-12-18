#ifndef UDP_PROCESS_H
#define UDP_PROCESS_H

#include <ros/ros.h>

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

using namespace std;

// pthread_mutex_t rec_mutex;        //创建线程锁
// pthread_mutex_lock(&rec_mutex);     //加锁

// pthread_mutex_unlock(&rec_mutex);    //解锁

struct udpSendInfo
{
  unsigned char *data; //发送的数据
  int datalen = 0;     //数据长度
  char *addr;          //发送的地址
  int port;            //发送的端口
};

class UdpProcessCallBack
{
  //回调类，这里自定义所有回调函数
public:
  virtual void OnUdpProcessCallBack(unsigned char *data_, int len_, struct sockaddr_in addr_) = 0;
};

class UdpProcess
{
public:
  UdpProcess();
  ~UdpProcess();

  int Initial(boost::shared_ptr< UdpProcessCallBack > p_DLCallBack, const std::string dev_ip, const int intput_port,
              const int udp_data_len);

  void recUdpInfo();
  void sendUdpInfo(udpSendInfo *udp_send_info_);

  std::string log_dir_;
  std::string device_name_;
  std::string sensor_name_;

private:
  int initUdp();

  UdpProcessCallBack *m_DLCallBack;
  boost::shared_ptr< UdpProcessCallBack > p_DLCallBack_;
  pthread_t m_threadID;

  int sockfd_;

  std::string devip_str_;
  int intput_port_;
  in_addr devip_;

  int isOpen_;

  int udp_data_size_;
};
#endif // UDP_PROCESS_H