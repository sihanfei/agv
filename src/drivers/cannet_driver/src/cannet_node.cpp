#include <ros/ros.h>

#include "udp_process.h"
#include <iostream>

#include "common_functions.h"
// #include <sys/types.h>

#include <thread>

#include <list>
#include <map>

#include "glog_helper.h"

using namespace std;

#define NODE_NAME "cannet_node"

#define RADAR1_IP "172.168.103.56"
#define RADAR1_PORT 8001

typedef struct
{
  uint8_t ide;                  /*!< 0: standard frame, 1: extended frame */
  uint32_t id; /*!< identify */ // 2365469190
  uint8_t can_data[8];          /*!< datas */
} * p_myCanData, myCanData;

class CannetDataProcess : public UdpProcessCallBack
{
public:
  CannetDataProcess(std::string ns_);
  void OnUdpProcessCallBack(unsigned char *data, int len, struct sockaddr_in addr_);

  void dataProcessing();
  std::string ns;
};
CannetDataProcess::CannetDataProcess(std::string ns_)
{
  ns = ns_;
}

void CannetDataProcess::OnUdpProcessCallBack(unsigned char *data, int len, struct sockaddr_in addr_)
{
  ostringstream ost;
  for (int ii = 0; ii < len; ii++)
  {
    ost << "0x" << hex << uppercase << setw(2) << setfill('0') << ( int )data[ii] << " ";
  }
  ROS_INFO("[%s] ip[%s] port[%d] rec = %s", ns.data(), inet_ntoa(addr_.sin_addr), htons(addr_.sin_port),
           ost.str().c_str());
}
//数据处理线程----测试
void CannetDataProcess::dataProcessing()
{
  ROS_INFO("[%s]thread dataProcessingStart!", ns.data());

  // while (ros::ok())
  // {
  //   myCanData my_;

  //   if (!list_myCanData_.empty())
  //   {
  //     ROS_INFO("!list_myCanData_.empty()");
  //     my_ = list_myCanData_.front();
  //     printf("id = %X ", my_.id);
  //     for (int i = 0; i < 8; i++)
  //     {
  //       printf("%X ", my_.can_data[i]);
  //     }
  //     printf("\n");

  //     list_myCanData_.pop_front();
  //   }
  //   sleep(1);
  // }
}
int main(int argc, char *argv[])
{
  // ROS_INFO("ROS node is start, name is [%s], file name is %s", NODE_NAME, argv[0]);
  // ros::init(argc, argv, NODE_NAME); // node name
  // ros::NodeHandle nh;

  // GLogHelper gh(argv[0]);
  // gh.setLogDirectory("/work/log/cannet_node");

  // // 建立 第一个 udp 连接
  // UdpProcess UdpProcess_1_;
  // SUPERG_INFO << "UdpProcess_1_";

  // CannetDataProcess radar1_("radar1");

  // SUPERG_INFO << "radar1_";

  // // udp连接初始化 要传入包含回调函数的类，以及IP地址和端口号 最后的13是一次接收UDP数据的长度 正好是一个CAN帧的长度
  // 13
  // int res = UdpProcess_1_.Initial(&radar1_, RADAR1_IP, RADAR1_PORT, 13);
  // SUPERG_ERROR_IF(res != 1) << "Initial err";

  // udpSendInfo udp_info_;
  // udp_info_.addr = ( char * )RADAR1_IP;
  // SUPERG_INFO << "udp_info_.addr " << udp_info_.addr;

  // udp_info_.port = 4001;
  // SUPERG_INFO << "udp_info_.port " << udp_info_.port;

  // udp_info_.datalen = 13;
  // SUPERG_INFO << "udp_info_.datalen " << udp_info_.datalen;

  // std::string str = "888CFE32061122334455667788";

  // std::vector< unsigned char > v_data_ = hexStringToByte(str);

  // udp_info_.data = &v_data_[0];

  // ostringstream ost;
  // for (int ii = 0; ii < udp_info_.datalen; ii++)
  // {
  //   ost << "0x" << hex << uppercase << setw(2) << setfill('0') << ( int )udp_info_.data[ii] << " ";
  // }
  // SUPERG_INFO << "udp_info_.data " << ost.str().c_str();

  // ros::Rate loop_rate(100);
  // while (ros::ok())
  // {
  //   SUPERG_INFO_EVERY(1000) << "sendUdpInfo ";
  //   UdpProcess_1_.sendUdpInfo(&udp_info_);
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }
  return 0;
}