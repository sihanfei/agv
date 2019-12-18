#include "planning_sender_pub.h"
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

#define TESTMODE1 0 // test route
#define TEST_NEW_ROUTE 1

std::string routing_ip_;
int routing_port_;

namespace YAML
{
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template <typename T>
void operator>>(const YAML::Node &node, T &i)
{
  i = node.as<T>();
}
} // namespace YAML

namespace superg_agv
{
namespace map
{
#define SERV_PORT 19001
#define MAXDATASIZE 2048
//#define SERVER_IP "172.168.103.171"
// #define SERVER_IP "192.168.67.158"
//#define SERVER_IP "172.20.10.5"
// #define SERVER_IP "192.168.1.100"
// #define SERVER_IP "192.168.2.16"
#define SERVER_IP "192.168.2.17"
// #define SERVER_IP "192.168.2.200"
// #define SERVER_IP "192.168.2.65"
// #define SERVER_IP "192.168.1.16"
// #define SERVER_IP "192.168.11.60"

#define BEGAINIDERROR 11
#define ENDIDERROR 12

void TaskSender::recvClickCallback(const geometry_msgs::PointStampedConstPtr &msg)
{
  task_click.x = msg->point.x;
  task_click.y = msg->point.y;
  task_click.laneID = (int)msg->point.z;
  task_click_recv_tip = 1;
  // ROS_INFO("rcev rviz click: (%lf,%lf) in lane %d", task_click.x, task_click.y, task_click.laneID);
}

void TaskSender::recvVCULocationCallback(const geometry_msgs::PointStampedConstPtr &msg)
{
  vcu_location.x = msg->point.x;
  vcu_location.y = msg->point.y;
  vcu_location.laneID = (int)msg->point.z;
  vcu_location_recv_tip = 1;
  // ROS_INFO("rcev vcu location: (%lf,%lf) in lane %d", vcu_location.x, vcu_location.y, vcu_location.laneID);
}

TaskSender::TaskSender(ros::NodeHandle &nh, int &s_laneID, int &e_laneID) : nh_(nh)
{
  ROS_INFO("TaskSender establish!");
  task_click_recv_tip = 0;
  vcu_location_recv_tip = 0;

  vcu_location.x = 0.0;
  vcu_location.y = 0.0;
  vcu_location.laneID = s_laneID;

  task_click.x = 0.0;
  task_click.y = 0.0;
  task_click.laneID = e_laneID;

  if (s_laneID > 0 && e_laneID > 0)
  {
    task_click_recv_tip = 1;
    vcu_location_recv_tip = 1;
  }

  route_laneID_pub_tip = 0;
  first_position_tip = 0;

  vcu_ID_ = 40;

  task_click_sub_ = nh.subscribe("/monitor/rviz_click_lane", 10, &TaskSender::recvClickCallback, this);
  vcu_locatio_sub_ = nh.subscribe("/map/vcu_location_lane", 10, &TaskSender::recvVCULocationCallback, this);

  route_laneID_array_pub_ = nh.advertise<std_msgs::Int64MultiArray>("/map/route_laneID_arry", 10, true);

  taskSender();
}

TaskSender::~TaskSender()
{
  ROS_INFO("TaskSender release!");
}

int TaskSender::socketLink()
{
  struct sockaddr_in servaddr;
  struct in_addr addr; //存放ip字符串//:
  int on = 1;          // setsockopt

  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  // servaddr.sin_port = htons(SERV_PORT);
  // servaddr.sin_addr.s_addr = inet_addr(SERVER_IP);
  servaddr.sin_port = htons(routing_port_);
  servaddr.sin_addr.s_addr = inet_addr(routing_ip_.c_str());

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
  int ret = connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));
  if (ret < 0)
  {
    ROS_INFO("can not link server");
  }
  ROS_INFO("ret = %d", ret);
  return ret;
}

void TaskSender::socketClose()
{
  close(sockfd);
  ROS_INFO("socket closed");
}

int TaskSender::socketSendAndRecv(std::string &data_buf_)
{
  u_int8_t socket_recv_buf[MAXDATASIZE]; //建议改为SocketBuf
  std::string sendbuf;
  std::string recvbuf;
  std::string b_str = "170170";
  std::string e_str = "204204";
  std::stringstream ss;

  double cuv = 3.0;
  double high = 15.5;

#if TEST_NEW_ROUTE
  ss << 40 << "," << high << "," << cuv << "," << vcu_location.laneID << "," << task_click.laneID;
#else
  ss << 40 << "," << vcu_location.laneID << "," << task_click.laneID;
#endif

  //  ss << 40 << "," << high << "," << cuv << "," << vcu_location.laneID << "," << task_click.laneID;
  sendbuf = ss.str();
  ss.str("");
  //  ROS_INFO("%s",sendbuf.c_str());
  int recvret = 0;

  sleep(1);
  int ret = send(sockfd, sendbuf.c_str(), sendbuf.length(), MSG_NOSIGNAL);
  ROS_INFO("send:%s", sendbuf.c_str());
  if (ret == -1)
  {
    ROS_INFO("ret = %d send error", ret);
  }
  else
  {
    ROS_INFO("ret = %d send ok", ret);
    ROS_INFO("wait route data...");
    sleep(1);
    ss.str("");
    int num = 0;
    while (1)
    {
      recvret = recv(sockfd, socket_recv_buf, sizeof(socket_recv_buf), 0);
      if (recvret > 0)
      {
        ss << socket_recv_buf;
        memset(socket_recv_buf, 0, sizeof(socket_recv_buf));
        recvbuf = ss.str();
        //判断包头
        int b_tip = recvbuf.find(b_str);
        //判断包尾
        int e_tip = recvbuf.find(e_str);
        //截取有效数据
        if (b_tip != std::string::npos && e_tip != std::string::npos)
        {
          data_buf_ = recvbuf.substr(b_tip + b_str.length() + 1, e_tip - b_tip - b_str.length() - 1);
          ROS_INFO("b_tip:%d e_tip:%d", b_tip + b_str.length(), e_tip);
          ROS_INFO("recv:%s", data_buf_.c_str());
          break;
        }
      }
    }
  }
  return recvret;
}

int TaskSender::getRouteDataFromString(std::string &data_buf_)
{
  std::stringstream ss;
  ss << data_buf_.c_str();
  int ss_i;
  char ss_c;
  int loop_i = 0;

  ss >> route_data.vcu_ID >> ss_c >> route_data.data_status >> ss_c >> route_data.data_length >> ss_c;

  if (route_data.data_length > 1 && route_data.data_status == 1)
  {
    for (size_t loop_i = 0; loop_i < route_data.data_length; loop_i++)
    {
      ss >> ss_i >> ss_c;
      route_data.data.push_back(ss_i);
      ROS_INFO("%d", route_data.data[loop_i]);
      ss_i = 0;
      ss_c = 0;
    }
  }
  else
  {
    ROS_INFO("route data length is error!");
  }
  ss.str("");

#if TESTMODE1 // test route
  route_data.data.clear();
  int ss_i_buf[] = {41, 22, 40, 11, 52, 53, 54, 55, 56, 15, 37, 23, 45, 16, 66, 26, 25, 27, 28, 29,
                    48, 6, 62, 3, 32, 2, 63, 7, 49, 48, 47, 46, 10, 38, 20, 30, 31, 32, 33, 34,
                    21, 35, 36, 37, 23, 45, 44, 43, 18, 60, 13, 54, 24, 65, 17, 43, 42, 41};
  for (size_t i = 0; i < 58; i++)
  {
    // ss_i = ss_i_buf[i];
    route_data.data.push_back(ss_i_buf[i]);
  }
  route_data.data_length = 58;
  ROS_INFO("route data length is %d!", route_data.data_length);
#endif

  return route_data.data_status;
}

int TaskSender::routeLaneIDArrayPub()
{
  std_msgs::Int64MultiArray int_array;
  std_msgs::MultiArrayDimension m_a_dim;
  m_a_dim.label = "40 route lane ID";
  m_a_dim.size = route_data.data_length;
  m_a_dim.stride = 0;
  int_array.layout.dim.push_back(m_a_dim);
  for (size_t loop_i = 0; loop_i < route_data.data.size(); loop_i++)
  {
    int_array.data.push_back(route_data.data[loop_i]);
  }
  route_laneID_array_pub_.publish(int_array);

  ROS_INFO("route laneID num %d published!", route_data.data.size());
  route_data.data.clear();
  return 1;
}

void TaskSender::taskSender()
{
  //判断是否收到了两个点的位置
  //然后启动线程,连接主机,并发送,然后收到全局数据后,发送全局数据,断开连接,销毁线程
  std::string data_buf;
  int ans = 0;

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    if (task_click_recv_tip == 1 && vcu_location_recv_tip == 1)
    {
      task_click_recv_tip = 0;
      vcu_location_recv_tip = 0;

      ROS_INFO("rcev vcu location: (%lf,%lf) in lane %d", vcu_location.x, vcu_location.y, vcu_location.laneID);
      ROS_INFO("rcev rviz click: (%lf,%lf) in lane %d", task_click.x, task_click.y, task_click.laneID);

      //第一次运行判断
      if (first_position_tip == 0)
      {
        task_click_last_ = task_click;
        vcu_location_last_ = vcu_location;
        route_laneID_pub_tip = 1;
      }
      else
      {
        if (task_click.laneID == task_click_last_.laneID)
        {
          route_laneID_pub_tip = 0;
        }
        else
        {
          route_laneID_pub_tip = 1;
        }
      }
      route_laneID_pub_tip = 1;

      //获取数据，发布数据
      if (route_laneID_pub_tip == 1)
      {
        socketLink();
        socketSendAndRecv(data_buf);
        socketClose();
        int r_status = getRouteDataFromString(data_buf);
        if (r_status == 1)
        {
          ans = routeLaneIDArrayPub();
        }
        else
        {
          if (r_status == BEGAINIDERROR)
          {
            ROS_INFO("Begain ID error!");
          }
          else if (r_status == ENDIDERROR)
          {
            ROS_INFO("END ID error!");
          }
          else
          {
            ROS_INFO("Other error!");
          }
        }
        if (ans > 0)
        {
          route_laneID_pub_tip = 0;
        }
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

} // namespace map
} // namespace superg_agv

using namespace superg_agv::map;
using namespace std;

void getSocketIPAndPort()
{
  //////////////////yaml file
  std::string path_ = ros::package::getPath("map");

  std::string yaml_file_name = path_ + "/cfg/config.yaml";
  ROS_DEBUG("ROS package path [%s]", yaml_file_name.c_str());

  YAML::Node doc;
  doc = YAML::LoadFile(yaml_file_name);

  //发送UDP数据的设备IP，这里就是WINDOWS电脑的IP地址
  routing_ip_ = doc["routing_ip"].as<string>();
  //接收UDP数据的端口
  routing_port_ = doc["routing_port"].as<int>();

  ROS_INFO_STREAM("IP address:" << routing_ip_.c_str() << " intput_port:" << routing_port_);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "planling_sender");
  ros::NodeHandle n;

  getSocketIPAndPort();

  int s_laneID = 0;
  int e_laneID = 0;
  if (argc == 3)
  {
    s_laneID = atoi(argv[1]);
    e_laneID = atoi(argv[2]);
  }
  ROS_INFO("lane %d -> lane %d", s_laneID, e_laneID);

  ROS_INFO("Goto Task Sender.");
  TaskSender task_sender_node(n, s_laneID, e_laneID);

  //主程序休眠
  ros::spin();
  return 0;
}