#include <ros/ros.h>

#include <sys/time.h>

#include "tcp_process.h"

//#define SPDLOG_ACTIVE_LEVEL 5

void SplitString(const string &s, vector< string > &v, const string &c)
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
TcpData_Output::TcpData_Output(string dir_path, string sensor_name, string device_name, string device_ip,
                               uint32_t m6_rec_port)
    : base_dir_path_(dir_path), sensor_name_(sensor_name), device_name_(device_name), device_ip_(device_ip),
      M6_rec_port_(m6_rec_port)
{
  log_file_created = false;
  logger_droped    = false;
  g_main_min       = 0;
  g_main_hour      = 0;

  logger_name_ = sensor_name_ + "-" + device_name_ + "-" + std::to_string(M6_rec_port_);

  // base_dir_path_ = "/work/test";

  SPDLOG_DEBUG(logger_name_);
}

TcpData_Output::~TcpData_Output()
{
  destroy_logger();
}

int TcpData_Output::my_mkdir(string muldir, mode_t mode)
{
  vector< string > v;
  SplitString(muldir, v, "/"); //可按多个字符来分隔;

  SPDLOG_DEBUG("v.size():{}", v.size());

  int iRet;

  ostringstream temp_dir;
  temp_dir.fill('0');

  for (vector< string >::size_type i = 0; i != v.size(); ++i)
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

bool TcpData_Output::MinHasChanged()
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
bool TcpData_Output::HourHasChanged()
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

bool TcpData_Output::create_log_file(string dir_path)
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
    spdlog::flush_every(std::chrono::seconds(3));
    return true;
  }
}

void TcpData_Output::destroy_logger()
{
  SPDLOG_DEBUG("destroy_logger");
  spdlog::drop(logger_name_); // logger使用完成后，要执行drop操作，否则不能循环创建同一类型的logger
}

void TcpData_Output::write_log(uint8_t *buff, int len)
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

  if (logger_droped)
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
///////////////////////////////////////////////////////////////////////////////////////////////
void *thread_DLProcess(void *arg)
{
  TcpProcess *pDLProcess = ( TcpProcess * )arg;
  pDLProcess->recTcpInfo();
  pthread_exit(( void * )0);
  return 0;
}

TcpProcess::TcpProcess()
{
}

TcpProcess::~TcpProcess()
{
  if (isOpen_)
  {
    SPDLOG_DEBUG("close(sockfd_)]");
    ( void )close(sockfd_);
  }
}

int TcpProcess::Initial(boost::shared_ptr< TcpProcessCallBack > p_DLCallBack, const std::string dev_ip,
                        const int intput_port, const int tcp_data_len)
{
  SPDLOG_DEBUG("TcpProcess::Initial: dev_ip:{} port {}", dev_ip.c_str(), intput_port);

  p_DLCallBack_ = p_DLCallBack;

  server_port_   = intput_port;
  tcp_data_size_ = tcp_data_len;
  devip_str_     = dev_ip;
  if (devip_str_.empty())
  {
    SPDLOG_ERROR("devip_str_ is empty!");
    return 0;
  }

  addr_len_ = sizeof(server_ip_);
  memset(&server_ip_, 0, sizeof(server_ip_));
  server_ip_.sin_family      = AF_INET;                       // Use IPV4
  server_ip_.sin_port        = htons(server_port_);           //
  server_ip_.sin_addr.s_addr = inet_addr(devip_str_.c_str()); // inet_addr(ip_);

  tcplog_.reset(new TcpData_Output(log_dir_, sensor_name_, device_name_, dev_ip, intput_port));

  isOpen_ = initTcp();

  if (isOpen_)
  {
    if (pthread_create(&m_threadID, NULL, thread_DLProcess, ( void * )this) != 0)
    {
      SPDLOG_ERROR("create thread of DLProcess failed!");
      return -1;
    }
    return 1;
  }
  else
  {
    SPDLOG_ERROR("tcp not open ip[{}] port[{}]", dev_ip, server_port_);
    return 0;
  }
}

int TcpProcess::initTcp()
{
  /* Create Socket*/

  if (isOpen_)
  {
    ( void )close(sockfd_);
  }
  sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (-1 == sockfd_)
  {
    return 0;
    SPDLOG_ERROR("Failed to create socket ");
  }
  /*Config Socket Addr*/
  // struct sockaddr_in addr;
  // socklen_t addr_len = sizeof(addr);
  // memset(&addr, 0, sizeof(addr));
  // addr.sin_family      = AF_INET;                       // Use IPV4
  // addr.sin_port        = htons(intput_port_);           //
  // addr.sin_addr.s_addr = inet_addr(devip_str_.c_str()); // inet_addr(ip_);
  /* Time out*/
  //    struct timeval tv;
  //    tv.tv_sec  = 0;
  //    tv.tv_usec = 200000;  // 200 ms
  //    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(struct timeval));
  /* Bind Socket*/
  if (connect(sockfd_, ( struct sockaddr * )(&server_ip_), addr_len_) == -1)
  // if (bind(sockfd_, ( struct sockaddr * )&addr, addr_len) == -1)
  { //收数据才需要bind
    SPDLOG_ERROR("Failed to bind tcp socket on port {}", server_port_);
    close(sockfd_);
    return 0;
  }
  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
  {
    SPDLOG_ERROR("non-block");
    return 0;
  }
  SPDLOG_DEBUG("Opening TCP socket: dev_ip:{} port {}", inet_ntoa(server_ip_.sin_addr), server_port_);
  return sockfd_;
}

void TcpProcess::recTcpInfo()
{

  unsigned char rebuf[2048];

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);

  while (1)
  {
    struct pollfd fds[1];
    fds[0].fd                     = sockfd_;
    fds[0].events                 = POLLIN;
    static const int POLL_TIMEOUT = -1; // timeout==-1，永远等待；timeout==0，不等待；timeout>0，等待timeout毫秒。
    // double time1 = ros::Time::now().toSec();
    do
    {
      poll(fds, 1, POLL_TIMEOUT);

    } while ((fds[0].revents & POLLIN) == 0);

    int nbytes = recv(sockfd_, &rebuf[0], tcp_data_size_, 0);
    //SPDLOG_DEBUG("nbytes[{}]", nbytes);
    if (nbytes == 0)
    {
      p_DLCallBack_->OnTcpStatusCallBack(0, server_ip_);
      return;
    }

    if (nbytes > 0)
    {
      // read successful,
      tcplog_->write_log(&rebuf[0], nbytes);
      // pthread_mutex_lock(&rec_mutex);
      // m_DLCallBack->OnUdpProcessCallBack(rebuf, udp_data_size_, sender_address);
      p_DLCallBack_->OnTcpProcessCallBack(rebuf, nbytes, server_ip_);
      // pthread_mutex_unlock(&rec_mutex);

    } // if (nbytes == udp_data_size_)

    // if (nbytes == tcp_data_size_)
    // {
    //   // read successful,
    //   tcplog_->write_log(&rebuf[0], nbytes);
    //   // pthread_mutex_lock(&rec_mutex);
    //   // m_DLCallBack->OnUdpProcessCallBack(rebuf, udp_data_size_, sender_address);
    //   p_DLCallBack_->OnTcpProcessCallBack(rebuf, tcp_data_size_, server_ip_);
    //   // pthread_mutex_unlock(&rec_mutex);

    // } // if (nbytes == udp_data_size_)
  } // while end
}

// 发送数据
void TcpProcess::sendTcpInfo(unsigned char *data, int datalen)
{
  // struct sockaddr_in dest;
  // socklen_t dest_len = sizeof(dest);
  // memset(&dest, 0, sizeof(dest));
  // dest.sin_family      = AF_INET;
  // dest.sin_port        = htons(tcp_send_info_->port);
  // dest.sin_addr.s_addr = inet_addr(tcp_send_info_->addr);

  if (datalen > 0) //如果数组有数则发送
  {
    send(sockfd_, data, datalen, 0);
    // sendto(sockfd_, tcp_send_info_->data, tcp_send_info_->datalen, 0, ( sockaddr * )&dest, dest_len);
  }
}
