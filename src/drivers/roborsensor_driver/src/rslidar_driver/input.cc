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
#include "input.h"
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sstream>
#include <string>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>

extern volatile sig_atomic_t flag;
namespace rslidar_driver
{

void SplitString(const std::string &s, vector< std::string > &v, const std::string &c)
{
  string::size_type pos1, pos2;
  pos2 = s.find(c);
  pos1 = 0;
  string str;
  while (string::npos != pos2)
  {
    str = s.substr(pos1, pos2 - pos1);
    if (str != "")
      v.push_back(str);

    pos1 = pos2 + c.size();
    pos2 = s.find(c, pos1);
  }
  if (pos1 != s.length())
  {
    str = s.substr(pos1);
    if (str != "")
      v.push_back(str);
  }
}

// #include <iostream>
UdpData_Output::UdpData_Output(std::string dir_path, std::string sensor_name, std::string device_name,
                               std::string device_ip, uint32_t m6_rec_port)
    : base_dir_path_(dir_path), sensor_name_(sensor_name), device_name_(device_name), device_ip_(device_ip),
      M6_rec_port_(m6_rec_port)
{
  log_file_created = false;
  logger_droped    = false;
  g_main_min       = 0;
  g_main_hour      = 0;

  logger_name_ = sensor_name_ + "-" + device_name_ + "-" + std::to_string(M6_rec_port_);

  SPDLOG_DEBUG(logger_name_);
  pn.param< int >("datalog_param", islog, 0);

  pn.getParam("datalog_param", islog);
  SPDLOG_ERROR("velodyne datalog_param:{}", islog);
}

UdpData_Output::~UdpData_Output()
{
  destroy_logger();
}

int UdpData_Output::my_mkdir(std::string muldir, mode_t mode)
{
  vector< std::string > v;
  SplitString(muldir, v, "/"); //可按多个字符来分隔;

  SPDLOG_DEBUG("v.size():{}", v.size());

  int iRet;

  ostringstream temp_dir;
  temp_dir.fill('0');

  for (vector< std::string >::size_type i = 0; i != v.size(); ++i)
  {
    if (v[i] == "")
      continue;
    temp_dir << '/' << v[i];

    SPDLOG_DEBUG("temp_dir:{}", temp_dir.str().c_str());

    if (access(temp_dir.str().c_str(), 0) != 0)
    {
      iRet = mkdir(temp_dir.str().c_str(), mode);
      if (iRet < 0)
        return iRet;
    }
  }

  return 0;
}

bool UdpData_Output::MinHasChanged()
{
  time_t raw_time;
  struct tm *tm_info;

  time(&raw_time);
  tm_info = localtime(&raw_time);

  if (tm_info->tm_min != g_main_min)
  {
    g_main_min = tm_info->tm_min;
    return true;
  }

  return false;
}
bool UdpData_Output::HourHasChanged()
{
  time_t raw_time;
  struct tm *tm_info;

  time(&raw_time);
  tm_info = localtime(&raw_time);

  if (tm_info->tm_hour != g_main_hour)
  {
    g_main_hour = tm_info->tm_hour;
    return true;
  }

  return false;
}

bool UdpData_Output::create_log_file(std::string dir_path)
{
  stringstream log_full_path;
  log_full_path << dir_path << log_file_name;

  // if (access(dir_path.c_str(), F_OK) != 0)
  // {
  //   //首先检查log目录是否存在，不存在则创建
  //   if (my_mkdir(dir_path.c_str(), 0777) < 0)
  //   {
  //     SPDLOG_DEBUG("mkdir={} msg={}", dir_path.data(), strerror(errno));
  //   }
  // }

  if (access(dir_path.c_str(), F_OK) != 0)
  {
    SPDLOG_ERROR("dir_path:{} not find", dir_path.c_str());
    return false;
  }
  else
  {
    SPDLOG_DEBUG("dir_path:{}", dir_path.c_str());
    SPDLOG_DEBUG("log_full_path:{}", log_full_path.str().c_str());

    //创建basic_logger，注意该函数创建的是支持多线程的文件输出
    // my_logger = spdlog::basic_logger_mt("basic_logger", log_full_path.str());

    my_logger = spdlog::basic_logger_mt< spdlog::async_factory >(logger_name_, log_full_path.str());

    // Change format pattern to all loggers
    // spdlog::set_pattern(" **** %Y-%m-%d %H:%M:%S.%e %l **** %v");
    my_logger->set_pattern("%Y-%m-%d-%H:%M:%S.%e,%n,%v"); //设置logger的输出格式
    spdlog::flush_every(std::chrono::seconds(1));
    return true;
  }
}

void UdpData_Output::destroy_logger()
{
  SPDLOG_DEBUG("destroy_logger");
  spdlog::drop(logger_name_); // logger使用完成后，要执行drop操作，否则不能循环创建同一类型的logger
}

void UdpData_Output::write_log(uint8_t *buff, int len)
{

  if (MinHasChanged() || !log_file_created)
  {
    //传感器IP+M6udp数据接收端口+时间
    if (logger_droped)
      destroy_logger();
    time_t raw_time;
    struct tm *tm_info;

    time(&raw_time);
    tm_info = localtime(&raw_time);

    ostringstream time_pid_stream;
    time_pid_stream.fill('0');
    time_pid_stream << device_name_ << '-' << device_ip_ << '-' << M6_rec_port_ << '-' << 1900 + tm_info->tm_year
                    << setw(2) << setfill('0') << 1 + tm_info->tm_mon << setw(2) << setfill('0') << tm_info->tm_mday
                    << '-' << setw(2) << setfill('0') << tm_info->tm_hour << setw(2) << setfill('0') << tm_info->tm_min
                    << setw(2) << setfill('0') << tm_info->tm_sec << ".csv";

    ostringstream dir_path_stream;
    dir_path_stream.fill('0');
    dir_path_stream << base_dir_path_ << '/' << sensor_name_ << '/' << 1900 + tm_info->tm_year << setw(2)
                    << setfill('0') << 1 + tm_info->tm_mon << setw(2) << setfill('0') << tm_info->tm_mday << '-'
                    << setw(2) << setfill('0') << tm_info->tm_hour << '/';

    log_file_name    = time_pid_stream.str();
    logger_droped    = create_log_file(dir_path_stream.str());
    log_file_created = true;
  }
  // my_logger->info();
  // my_logger->info("{0:x} + {1:X} = {2:x}", buff[0], buff[1], buff[2]);
  pn.getParam("datalog_param", islog);

  if (logger_droped && islog)
  // if (logger_droped)
  {
    std::vector< unsigned char > uvchar;
    for (size_t i = 0; i < len; i++)
    {
      uvchar.push_back(buff[i]);
    }
    // log binary data as hex.
    // many types of std::container<char> types can be used.
    // ranges are supported too.
    // format flags:
    // {:X} - print in uppercase.
    // {:s} - don't separate each byte with space.
    // {:p} - don't print the position on each line start.
    // {:n} - don't split the output to lines.

    // my_logger->info("Another binary example:{:n}", spdlog::to_hex(std::begin(uvchar), std::begin(uvchar) + len));

    my_logger->info("{:Xspn}", spdlog::to_hex(uvchar));
  }
}
static const size_t packet_size = sizeof(rslidar_msgs::rslidarPacket().data);

////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number.
 */
Input::Input(ros::NodeHandle private_nh, uint16_t port) : private_nh_(private_nh), port_(port)
{
  npkt_update_flag_ = false;
  cur_rpm_          = 600;
  return_mode_      = 1;

  private_nh.param("device_ip", devip_str_, std::string(""));
  if (!devip_str_.empty())
    ROS_INFO_STREAM("Only accepting packets from IP address: " << devip_str_);
}

int Input::getRpm(void)
{
  return cur_rpm_;
}

int Input::getReturnMode(void)
{
  return return_mode_;
}

bool Input::getUpdateFlag(void)
{
  return npkt_update_flag_;
}

void Input::clearUpdateFlag(void)
{
  npkt_update_flag_ = false;
}
////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
*/
InputSocket::InputSocket(ros::NodeHandle private_nh, uint16_t port) : Input(private_nh, port)
{
  sockfd_ = -1;

  if (!devip_str_.empty())
  {
    inet_aton(devip_str_.c_str(), &devip_);
  }

  ROS_INFO_STREAM("Opening UDP socket: port " << port);
  sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd_ == -1)
  {
    perror("socket"); // TODO: ROS_ERROR errno
    return;
  }

  int opt = 1;
  if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, ( const void * )&opt, sizeof(opt)))
  {
    perror("setsockopt error!\n");
    return;
  }

  sockaddr_in my_addr;                   // my address information
  memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
  my_addr.sin_family      = AF_INET;     // host byte order
  my_addr.sin_port        = htons(port); // port in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

  if (bind(sockfd_, ( sockaddr * )&my_addr, sizeof(sockaddr)) == -1)
  {
    perror("bind"); // TODO: ROS_ERROR errno
    return;
  }

  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
  {
    perror("non-block");
    return;
  }

  // udp log
  char *home_path          = getenv("HOME");
  char log_file_name[1024] = {0};
  sprintf(log_file_name, "%s/work/log/", home_path);
  log_dir_ = log_file_name;

  SPDLOG_DEBUG("log_dir_ ={}", log_dir_);

  sensor_name_ = "rslidar";

  // private_nh_.param("frame_id", device_name_, std::string("velodyne"));

  device_name_ = private_nh.getNamespace();

  vector< std::string > v;
  SplitString(device_name_, v, "/"); //可按多个字符来分隔;
  if (v.size() > 0)
  {

    device_name_ = v[v.size() - 1];
  }
  else
    device_name_ = "device_name";

  SPDLOG_DEBUG("device_name_ ={}", device_name_);

  udplog_.reset(new UdpData_Output(log_dir_, sensor_name_, device_name_, devip_str_, port));

  ROS_DEBUG("Velodyne socket fd is %d\n", sockfd_);
}

/** @brief destructor */
InputSocket::~InputSocket(void)
{
  ( void )close(sockfd_);
}

/** @brief Get one rslidar packet. */
int InputSocket::getPacket(rslidar_msgs::rslidarPacket *pkt, const double time_offset)
{
  double time1 = ros::Time::now().toSec();
  struct pollfd fds[1];
  fds[0].fd                     = sockfd_;
  fds[0].events                 = POLLIN;
  static const int POLL_TIMEOUT = 1000; // one second (in msec)

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);
  while (flag == 1)
  {
    // Receive packets that should now be available from the
    // socket using a blocking read.
    // poll() until input available
    do
    {
      int retval = poll(fds, 1, POLL_TIMEOUT);
      if (retval < 0) // poll() error?
      {
        if (errno != EINTR)
          ROS_ERROR("poll() error: %s", strerror(errno));
        return 1;
      }
      if (retval == 0) // poll() timeout?
      {
        ROS_WARN("Rslidar poll() timeout");

        char buffer_data[8] = "re-con";
        memset(&sender_address, 0, sender_address_len);                // initialize to zeros
        sender_address.sin_family      = AF_INET;                      // host byte order
        sender_address.sin_port        = htons(MSOP_DATA_PORT_NUMBER); // port in network byte order, set any value
        sender_address.sin_addr.s_addr = devip_.s_addr;                // automatically fill in my IP
        sendto(sockfd_, &buffer_data, strlen(buffer_data), 0, ( sockaddr * )&sender_address, sender_address_len);
        return 1;
      }
      if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL)) // device error?
      {
        ROS_ERROR("poll() reports Rslidar error");
        return 1;
      }
    } while ((fds[0].revents & POLLIN) == 0);
    ssize_t nbytes =
        recvfrom(sockfd_, &pkt->data[0], packet_size, 0, ( sockaddr * )&sender_address, &sender_address_len);

    if (nbytes < 0)
    {
      if (errno != EWOULDBLOCK)
      {
        perror("recvfail");
        ROS_INFO("recvfail");
        return 1;
      }
    }
    else if (( size_t )nbytes == packet_size)
    {
      if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr)
        continue;
      else
        break; // done
    }

    // ROS_DEBUG_STREAM("incomplete rslidar packet read: " << nbytes << " bytes");
  }
  if (flag == 0)
  {
    abort();
  }

  if (pkt->data[0] == 0xA5 && pkt->data[1] == 0xFF && pkt->data[2] == 0x00 && pkt->data[3] == 0x5A)
  { // difop
    int rpm  = (pkt->data[8] << 8) | pkt->data[9];
    int mode = 1;

    if ((pkt->data[45] == 0x08 && pkt->data[46] == 0x02 && pkt->data[47] >= 0x09) || (pkt->data[45] > 0x08) ||
        (pkt->data[45] == 0x08 && pkt->data[46] > 0x02))
    {
      if (pkt->data[300] != 0x01 && pkt->data[300] != 0x02)
      {
        mode = 0;
      }
    }

    if (cur_rpm_ != rpm || return_mode_ != mode)
    {
      cur_rpm_     = rpm;
      return_mode_ = mode;

      npkt_update_flag_ = true;
    }
  }

  udplog_->write_log(&pkt->data[0], packet_size);
  // Average the times at which we begin and end reading.  Use that to
  // estimate when the scan occurred. Add the time offset.
  double time2 = ros::Time::now().toSec();
  pkt->stamp   = ros::Time((time2 + time1) / 2.0 + time_offset);

  return 0;
}
}
