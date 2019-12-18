#include <sys/time.h>

#include "udp_process.h"

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
  ROS_DEBUG("UdpProcess");
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
  ROS_DEBUG_STREAM("UdpProcess::Initial: dev_ip:" << dev_ip.c_str() << " port:" << intput_port);

  p_DLCallBack_ = p_DLCallBack;

  intput_port_   = intput_port;
  udp_data_size_ = udp_data_len;
  devip_str_     = dev_ip;
  ROS_DEBUG_STREAM("IP address: " << devip_str_.c_str());

  isOpen_ = initUdp();

  if (!devip_str_.empty())
  {
    inet_aton(devip_str_.c_str(), &devip_);
  }
  else
    ROS_ERROR("devip_str_ is empty!");

  ROS_DEBUG_STREAM("devip_:" << inet_ntoa(devip_));

  if (isOpen_)
  {
    if (pthread_create(&m_threadID, NULL, thread_DLProcess, ( void * )this) != 0)
    {
      ROS_ERROR("create thread of DLProcess failed!");
      return -1;
    }
    return 1;
  }
  else
  {
    ROS_ERROR("udp not open");
    return 0;
  }
}

int UdpProcess::initUdp()
{

  /* Create Socket*/
  ROS_DEBUG_STREAM("Opening UDP socket: port " << intput_port_);
  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (-1 == sockfd_)
  {
    return 0;
    ROS_ERROR("Failed to create socket");
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
    ROS_ERROR_STREAM("Failed to bind udp socket on port " << intput_port_);
    close(sockfd_);
    return 0;
  }
  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
  {
    ROS_ERROR("non-block");
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
    // ROS_DEBUG_STREAM("nbytes!" << nbytes << " - " << udp_data_size_);
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
