#include <ros/ros.h>

#include "common_functions.h"

#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <vector>

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define DEST_PORT1 6109
#define DEST_PORT2 6110
#define DSET_IP_ADDRESS "192.168.2.46"

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include "spdlog/logger.h"

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

#include "spdlog/async.h"

#include "spdlog/fmt/bin_to_hex.h"
////////////////////
#include "rslidar_msgs/rslidarPacket.h"

using namespace std;

#define DATA_PATH "/work/agvdata"
#define DATA_NAME1 "/rs1-192.168.2.109-6109-20190821-110000.csv"
#define DATA_NAME2 "/rs2-192.168.2.110-6110-20190821-110000.csv"

#define NODE_NAME "sim_rslidar_node"

pthread_t ReadThread;

int hexStringToBytes(const std::string &hex, unsigned char *dest)
{
  std::string str = trimString(hex);
  int len         = hex.size();

  int icount = 0;
  for (decltype(len) i = 0; i < len; i += 2)
  {
    unsigned int element;
    std::istringstream strHex(hex.substr(i, 2));
    strHex >> std::hex >> element;
    dest[icount] = static_cast< unsigned char >(element);
    // dest.push_back(static_cast< unsigned char >(element));
    icount++;
  }
  return icount;
}

int main(int argc, char **argv)
{
  SPDLOG_DEBUG("ROS node is star, name is {}, file name is {}", NODE_NAME, argv[0]);
  ros::init(argc, argv, NODE_NAME); // node name
  ros::NodeHandle nh;

  /* socket文件描述符 */
  int sock_fd;

  /* 建立udp socket */
  sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd < 0)
  {
    SPDLOG_DEBUG("socket");
    return 0;
  }

  /* 设置address */
  struct sockaddr_in addr_serv;
  int len;
  memset(&addr_serv, 0, sizeof(addr_serv));
  addr_serv.sin_family      = AF_INET;
  addr_serv.sin_addr.s_addr = inet_addr(DSET_IP_ADDRESS);
  addr_serv.sin_port        = htons(DEST_PORT1);
  len                       = sizeof(addr_serv);
  int send_num;

  //读取 雷达数据文件
  char *line = NULL;

  //获取routing data path
  char *home_path                 = getenv("HOME");
  char route_date_path_name[1024] = {0};
  sprintf(route_date_path_name, "%s%s%s", home_path, DATA_PATH, DATA_NAME1);
  SPDLOG_DEBUG("radar1 data path name:{}", route_date_path_name);

  ros::Time t1 = ros::Time::now();
  SPDLOG_DEBUG("start...");
  FILE *fp;
  size_t slen = 0;
  ssize_t read;
  fp = fopen(route_date_path_name, "r");

  if (fp == NULL)
  {
    SPDLOG_DEBUG("fail to read");
    return 0;
  }
  std::vector< rslidar_msgs::rslidarPacket > list_packets_1;

  int irows1 = 0;

  while (!feof(fp))
  {
    // velodyne_msgs::VelodynePacket tmp_packet;
    rslidar_msgs::rslidarPacket tmp_packet;
    irows1++;
    //逐行读取
    read                               = getline(&line, &slen, fp);
    std::vector< std::string > v_line_ = string_split(line, ",");
    int sendlen                        = hexStringToBytes(v_line_.at(2), &tmp_packet.data[0]);
    list_packets_1.push_back(tmp_packet);
  }
  fclose(fp);

  size_t packet_size = sizeof(rslidar_msgs::rslidarPacket().data);
  ros::Time t2       = ros::Time::now();
  ros::Duration d    = t2 - t1;
  SPDLOG_DEBUG("read file1 Sec = [{}]", d.toSec());
  //////////////////////////////////////////////////////////////////////////////
  /* socket文件描述符 */
  int sock_fd2;

  /* 建立udp socket */
  sock_fd2 = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd2 < 0)
  {
    SPDLOG_DEBUG("socket2");
    return 0;
  }

  /* 设置address */
  struct sockaddr_in addr_serv2;
  int len2;
  memset(&addr_serv2, 0, sizeof(addr_serv2));
  addr_serv2.sin_family      = AF_INET;
  addr_serv2.sin_addr.s_addr = inet_addr(DSET_IP_ADDRESS);
  addr_serv2.sin_port        = htons(DEST_PORT2);
  len2                       = sizeof(addr_serv2);
  int send_num2;
  sprintf(route_date_path_name, "%s%s%s", home_path, DATA_PATH, DATA_NAME2);
  SPDLOG_DEBUG("radar2 data path name:{}", route_date_path_name);

  fp = fopen(route_date_path_name, "r");

  if (fp == NULL)
  {
    SPDLOG_DEBUG("fail to read");
    return 0;
  }
  std::vector< rslidar_msgs::rslidarPacket > list_packets_2;

  int irows2 = 0;

  while (!feof(fp))
  {
    // velodyne_msgs::VelodynePacket tmp_packet;
    rslidar_msgs::rslidarPacket tmp_packet;
    irows2++;
    //逐行读取
    read                               = getline(&line, &slen, fp);
    std::vector< std::string > v_line_ = string_split(line, ",");
    int sendlen                        = hexStringToBytes(v_line_.at(2), &tmp_packet.data[0]);
    list_packets_2.push_back(tmp_packet);
  }
  // size_t packet_size = sizeof(velodyne_msgs::VelodynePacket().data);
  ros::Time t3 = ros::Time::now();
  d            = t3 - t2;
  SPDLOG_DEBUG("read file2 Sec = [{}]", d.toSec());

  ros::Rate loop_rate(840);
  int icount = 0;
  int rows   = 0;
  if (irows1 > irows2)
    rows = irows2;
  else
    rows = irows1;
  while (ros::ok())
  {

    // SPDLOG_DEBUG("{}: time= {}", irows, v_line_.at(0).c_str());
    send_num =
        sendto(sock_fd, &list_packets_1.at(icount).data[0], packet_size, 0, ( struct sockaddr * )&addr_serv, len);
    send_num =
        sendto(sock_fd2, &list_packets_2.at(icount).data[0], packet_size, 0, ( struct sockaddr * )&addr_serv2, len);

    if (send_num < 0)
    {
      SPDLOG_DEBUG("sendto error:");
      return 0;
    }
    icount++;
    if (icount >= rows)
      // if (icount > 75)
      icount = 0;
    ros::spinOnce();

    loop_rate.sleep();
  }
  t2 = ros::Time::now();
  d  = t2 - t1;
  SPDLOG_DEBUG("while end Sec = [{}]", d.toSec());
  return 0;
}
