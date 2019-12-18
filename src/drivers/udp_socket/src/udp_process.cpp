

#include <sys/time.h>

#include "udp_process.h"

//#define SPDLOG_ACTIVE_LEVEL 5

#define PATHNAME "/tmp"
#define PROJ_ID 0x6666

static int CommShm(int size, int flags)
{
  key_t key = ftok(PATHNAME, PROJ_ID);
  printf("key_t: %08x\n", key);
  if (key < 0)
  {
    perror("ftok");
    return -1;
  }
  int shmid = 0;

  if ((shmid = shmget(key, size, flags)) < 0)
  {
    perror("shmget");
    return -2;
  }
  printf("shmid: %d\n", shmid);
  return shmid;
}
int DestroyShm(int shmid)
{
  if (shmctl(shmid, IPC_RMID, NULL) < 0)
  {
    perror("DestroyShm");
    return -1;
  }
  return 0;
}
int CreateShm(int size)
{
  return CommShm(size, IPC_CREAT | IPC_EXCL | 0666);
}
int GetShm(int size)
{
  return CommShm(size, IPC_CREAT | 0666);
}

int get_shm_nattch(int shmid_)
{
  printf("get_shm_nattch shmid: %d\n", shmid_);
  shmid_ds ds_;
  if (shmctl(shmid_, IPC_STAT, &ds_) < 0)
  {
    perror("get_shm_nattch");
    return -1;
  }
  return ( int )ds_.shm_nattch;
  // return 0;
}

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
UdpData_Output::UdpData_Output(string dir_path, string sensor_name, string device_name, string device_ip,
                               uint32_t m6_rec_port)
    : base_dir_path_(dir_path), sensor_name_(sensor_name), device_name_(device_name), device_ip_(device_ip),
      M6_rec_port_(m6_rec_port)
{
  log_file_created = false;
  logger_droped    = false;
  g_main_min       = 0;
  g_main_hour      = 0;

  logger_name_ = sensor_name_ + "-" + device_name_ + "-" + std::to_string(M6_rec_port_);
  SPDLOG_DEBUG(logger_name_);
  // base_dir_path_ = "/work/test";

  shmid    = GetShm(1024);
  shm_addr = ( char * )shmat(shmid, NULL, 0);

  SPDLOG_DEBUG("shm_addr:{}", ( int )shm_addr[0]);
}

UdpData_Output::~UdpData_Output()
{
  shmdt(shm_addr);
  if (get_shm_nattch(shmid) == 0)
  {
    DestroyShm(shmid);
  }
  destroy_logger();
}

int UdpData_Output::my_mkdir(string muldir, mode_t mode)
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

bool UdpData_Output::create_log_file(string dir_path)
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

  if (logger_droped && shm_addr[0] == 1)
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
  UdpProcess *pDLProcess = ( UdpProcess * )arg;
  pDLProcess->recUdpInfo();
  pthread_exit(( void * )0);
  return 0;
}

UdpProcess::UdpProcess()
{
}

UdpProcess::~UdpProcess()
{
  if (isOpen_)
  {
    ( void )close(sockfd_);
  }
}

int UdpProcess::Initial(boost::shared_ptr< UdpProcessCallBack > p_DLCallBack, const std::string dev_ip,
                        const int intput_port, const int udp_data_len)
{
  SPDLOG_DEBUG("UdpProcess::Initial: dev_ip:{} port {}", dev_ip.c_str(), intput_port);

  p_DLCallBack_ = p_DLCallBack;

  intput_port_   = intput_port;
  udp_data_size_ = udp_data_len;
  devip_str_     = dev_ip;
  SPDLOG_DEBUG("IP address: {}", devip_str_.c_str());

  udplog_.reset(new UdpData_Output(log_dir_, sensor_name_, device_name_, dev_ip, intput_port));

  isOpen_ = initUdp();

  if (!devip_str_.empty())
  {
    inet_aton(devip_str_.c_str(), &devip_);
  }
  else
    SPDLOG_ERROR("devip_str_ is empty!");

  ROS_DEBUG_STREAM("devip_:" << inet_ntoa(devip_));

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
    SPDLOG_ERROR("udp not open");
    return 0;
  }
}

int UdpProcess::initUdp()
{

  /* Create Socket*/
  SPDLOG_DEBUG("Opening UDP socket: port {}", intput_port_);
  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (-1 == sockfd_)
  {
    return 0;
    SPDLOG_ERROR("Failed to create socket");
  }
  /*Config Socket Addr*/
  struct sockaddr_in addr;
  socklen_t addr_len = sizeof(addr);
  memset(&addr, 0, sizeof(addr));
  addr.sin_family      = AF_INET;             // Use IPV4
  addr.sin_port        = htons(intput_port_); //
  addr.sin_addr.s_addr = INADDR_ANY;          // inet_addr(ip_);
  /* Time out*/
  //    struct timeval tv;
  //    tv.tv_sec  = 0;
  //    tv.tv_usec = 200000;  // 200 ms
  //    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(struct timeval));
  /* Bind Socket*/
  if (bind(sockfd_, ( struct sockaddr * )&addr, addr_len) == -1)
  { //收数据才需要bind
    SPDLOG_ERROR("Failed to bind udp socket on port {}", intput_port_);
    close(sockfd_);
    return 0;
  }
  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
  {
    SPDLOG_ERROR("non-block");
    return 0;
  }
  return sockfd_;
}

void UdpProcess::recUdpInfo()
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

    int nbytes = recvfrom(sockfd_, &rebuf[0], udp_data_size_, 0, ( sockaddr * )&sender_address, &sender_address_len);

    if (nbytes == udp_data_size_)
    {

      // read successful,
      // if packet is not from the lidar scanner we selected by IP,
      // continue otherwise we are done
      if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr)
      {

        continue;
      }
      else
      {
        udplog_->write_log(&rebuf[0], nbytes);
        // pthread_mutex_lock(&rec_mutex);
        // m_DLCallBack->OnUdpProcessCallBack(rebuf, udp_data_size_, sender_address);
        p_DLCallBack_->OnUdpProcessCallBack(rebuf, udp_data_size_, sender_address);
        // pthread_mutex_unlock(&rec_mutex);
      }
    } // if (nbytes == udp_data_size_)
  }   // while end
}

// udp 发送数据
void UdpProcess::sendUdpInfo(udpSendInfo *udp_send_info_)
{
  struct sockaddr_in dest;
  socklen_t dest_len = sizeof(dest);
  memset(&dest, 0, sizeof(dest));
  dest.sin_family      = AF_INET;
  dest.sin_port        = htons(udp_send_info_->port);
  dest.sin_addr.s_addr = inet_addr(udp_send_info_->addr);

  if (udp_send_info_->datalen != 0) //如果数组有数则发送
  {
    sendto(sockfd_, udp_send_info_->data, udp_send_info_->datalen, 0, ( sockaddr * )&dest, dest_len);
  }
}
