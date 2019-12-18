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

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include "spdlog/logger.h"

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

#include "spdlog/async.h"

#include "spdlog/fmt/bin_to_hex.h"
////////////////////
#include "rslidar_msgs/rslidarPacket.h"
#include "velodyne_msgs/VelodynePacket.h"

#include <yaml-cpp/yaml.h>

#include <thread>

#include <boost/thread.hpp>

#include <fstream>
#include <ios>

using namespace std;

#define CONTI_YAML_FILE_PATH "/work/superg_agv/src/monitor/rviz_monitor/cfg"
#define CONTI_YAML_FILE_NAME "/lidar_info_list.yaml"

#define NODE_NAME "sim_lidar_node"

pthread_t ReadThread;

struct Lidar_Info
{
  std::string file_path;
  std::string file_name;
  std::string device_ip;
  int intput_port;
  int packet_size;
  std::string liadr_name;
};

class MyThread
{
public:
  Lidar_Info info_;

  MyThread(const Lidar_Info info) : info_(info)
  {
  }
  void readfile()
  {
    ros::Time t1 = ros::Time::now();

    //获取routing data path
    char *home_path                 = getenv("HOME");
    char route_date_path_name[1024] = {0};
    sprintf(route_date_path_name, "%s%s%s", home_path, info_.file_path.c_str(), info_.file_name.c_str());
    SPDLOG_DEBUG("[{}] data path name:{}", getThreadId(), route_date_path_name);

    //读取 雷达数据文件

    ifstream fin(route_date_path_name);

    stringstream buffer;
    buffer << fin.rdbuf();
    fin.close();

    string line;
    rslidar_msgs::rslidarPacket tmp_packet;
    list_strings_.clear();
    //逐行读取
    while (getline(buffer, line))
    {
      irows++;

      std::vector< std::string > v_line_ = string_split(line, ",");
      // std::vector< unsigned char > v_char_ = str_to_hex(v_line_.at(2));
      // vector全部转到数组
      // memcpy(&tmp_packet.data[0], &v_char_[0], info_.packet_size);
      // int sendlen                        = hexStringToBytes(v_line_.at(2), &tmp_packet.data[0]);

      list_strings_.push_back(v_line_.at(2));
    }

    ros::Time t2    = ros::Time::now();
    ros::Duration d = t2 - t1;
    SPDLOG_DEBUG("[{}] read file Sec = [{}]", info_.liadr_name, d.toSec());
  }

  void start()
  {

    SPDLOG_DEBUG("[{}] start [rows={}] [rate={}] ", info_.liadr_name, irows, ( int )(irows / 60));
    /* socket文件描述符 */
    int sock_fd;

    /* 建立udp socket */
    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0)
    {
      SPDLOG_DEBUG("socket");
      return;
    }

    /* 设置address */
    struct sockaddr_in addr_serv;
    int len;
    memset(&addr_serv, 0, sizeof(addr_serv));
    addr_serv.sin_family      = AF_INET;
    addr_serv.sin_addr.s_addr = inet_addr(info_.device_ip.c_str());
    addr_serv.sin_port        = htons(info_.intput_port);
    len                       = sizeof(addr_serv);
    int send_num;

    int icount = 0;
    ros::Rate loop_rate(( int )(irows / 60));
    ros::Time t1 = ros::Time::now();

    unsigned char send_buf_[1300];
    SPDLOG_DEBUG("[{}]send start...", info_.liadr_name);
    while (ros::ok())
    {

      send_num = sendto(sock_fd, &str_to_hex(list_strings_.at(icount))[0], info_.packet_size, 0,
                        ( struct sockaddr * )&addr_serv, len);

      // send_num = sendto(sock_fd, &list_packets_.at(icount).data[0], info_.packet_size, 0,
      //                   ( struct sockaddr * )&addr_serv, len);
      icount++;
      if (icount >= irows)
      {
        // if (icount > 75)
        ros::Time t2    = ros::Time::now();
        ros::Duration d = t2 - t1;
        t1              = t2;
        SPDLOG_DEBUG("[{}] NSec = [{}]", info_.liadr_name, d.toNSec());
        icount = 0;
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
    SPDLOG_DEBUG("[{}] end...", info_.liadr_name);
  }

private:
  std::vector< std::string > list_strings_;

  int irows          = 0;
  size_t packet_size = sizeof(velodyne_msgs::VelodynePacket().data);

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
  std::vector< unsigned char > str_to_hex(const std::string &str)
  {
    // if (str.size() != str.size() / 2 * 2)
    //   str = "0" + str;
    std::vector< unsigned char > vec;
    std::stringstream ss;
    ss << std::hex;
    for (std::string::size_type ix = 0; ix != str.size() / 2; ++ix)
    {
      int val = 0;
      ss << str.substr(ix * 2, 2);
      ss >> val;
      vec.push_back(val);
      ss.clear();
    }
    return vec;
  }

  unsigned long getThreadId()
  {
    std::string threadId       = boost::lexical_cast< std::string >(boost::this_thread::get_id());
    unsigned long threadNumber = 0;
    threadNumber               = std::stoul(threadId, nullptr, 16);
    // sscanf(threadId.c_str(), "%lx", &threadNumber); //也可以使用这个句子完成转换
    return threadNumber;
  }
};

namespace YAML
{
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template < typename T > void operator>>(const YAML::Node &node, T &i)
{
  i = node.as< T >();
}
} /* YAML */

// now the extraction operators for these types //重载 >> 预算符。。。。
void operator>>(const YAML::Node &node, Lidar_Info &lidar_info)
{
  node["file_path"] >> lidar_info.file_path;
  node["file_name"] >> lidar_info.file_name;
  node["device_ip"] >> lidar_info.device_ip;
  node["intput_port"] >> lidar_info.intput_port;
  node["packet_size"] >> lidar_info.packet_size;
  node["liadr_name"] >> lidar_info.liadr_name;
}
int main(int argc, char **argv)
{
  SPDLOG_DEBUG("ROS node is star, name is {}, file name is {}", NODE_NAME, argv[0]);
  ros::init(argc, argv, NODE_NAME); // node name
  ros::NodeHandle nh;

  boost::thread_group group;
  //////////////////yaml file
  char *home_path           = getenv("HOME");
  char yaml_file_name[1024] = {0};
  sprintf(yaml_file_name, "%s%s%s", home_path, CONTI_YAML_FILE_PATH, CONTI_YAML_FILE_NAME);
  SPDLOG_DEBUG("yaml_file_name {}", yaml_file_name);

  std::string file_name = yaml_file_name;
  YAML::Node doc;
  doc = YAML::LoadFile(file_name);

  boost::shared_ptr< MyThread > mythread_[doc["lidar_info_list"].size()];

  for (size_t i = 0; i < doc["lidar_info_list"].size(); i++)
  {
    Lidar_Info info_;
    doc["lidar_info_list"][i] >> info_;
    mythread_[i].reset(new MyThread(info_));

    group.create_thread(boost::bind(&MyThread::readfile, mythread_[i]));

    // boost::thread t(&MyThread::readfile, mythread_[i]);
    // t.detach();
  }
  group.join_all(); //等待所有线程执行结束

  for (size_t i = 0; i < doc["lidar_info_list"].size(); i++)
  {
    group.create_thread(boost::bind(&MyThread::start, mythread_[i]));
  }

  ros::spin();
  SPDLOG_DEBUG("[{}] end", NODE_NAME);
  group.join_all();
  return 0;
}
