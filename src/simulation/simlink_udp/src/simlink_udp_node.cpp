#include <ros/package.h>
#include <ros/ros.h>

#include "udp_process.h"
#include <iostream>

#include <thread>

#include <list>
#include <map>

#include <yaml-cpp/yaml.h>

#include <sys/syscall.h>
#include <unistd.h> //getpid()

#include <boost/thread.hpp>

#include "common_msgs/DetectionInfo.h"

#include "perception_msgs/FusionDataInfo.h"

using namespace std;

#define NODE_NAME "simlink_udp_node"

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

std::string byteToHexString(const unsigned char *data, size_t size)
{
  std::ostringstream strHex;
  strHex << std::hex << std::setfill('0');
  for (size_t i = 0; i < size; ++i)
  {
    strHex << std::setw(2) << static_cast<unsigned int>(data[i]);
  }
  return strHex.str();
}
//将字节中pos位置开始的len位的二进制数转换为整数
unsigned int getbitu(const unsigned char *buff, int pos, int len)
{
  unsigned int bits = 0;
  int i;
  for (i = pos; i < pos + len; i++)
  {
    bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u); //从高位到低位逐位计算
  }
  return bits;
}

double HexToDouble(const unsigned char *buf)
{
  double value = 0;
  unsigned int i = 0;
  unsigned int num, temp;
  int num2;
  bool flags1 = true;

  num = getbitu(buf, i, 1); //标志位
  i += 1;
  // double型规定偏移量为1023，其表示范围为-1024-1023
  num2 = getbitu(buf, i, 11) - 1023;
  i += 11;

  while (1)
  {
    if (flags1)
    {
      flags1 = false;
      value += 1 * pow(2, num2);
      num2--;
    }
    temp = getbitu(buf, i, 1);
    i += 1;
    value += temp * pow(2, num2);
    num2--;
    if (i == 64)
      break;
  }
  if (num == 1)
    value *= -1;

  return value;
}

////////////////////////////
void DoubleToHex(double m, unsigned char *buf)
{
  int i;
  int m1;        // double型数据的整数部分
  double m2, m3; // double型数据的小数部分
  int num1;      //阶数的值
  int num2;      //尾数的位数
  long long int temp1 = 0;
  long long int temp2 = 0;

  //符号位判断
  if (m >= 0)
  {
    buf[0] = 0;
  }
  else
  {
    buf[0] = 128;
  }
  m = fabs(m);

  //将double型数分解成整数部分和小数部分
  m1 = floor(m);
  m2 = m - m1;

  //阶数计算
  if (m1 > 0)
  {
    for (num1 = 0, temp1 = 1; 2 * temp1 <= m1; num1++)
    {
      temp1 *= 2;
    }
    num1 += 1023;
  }
  else
  {
    for (num1 = 0, m3 = m2; m3 <= 1; num1++)
    {
      m3 *= 2;
    }
    num1 = 1023 - num1;
  }

  buf[0] += ((num1 >> 4) & 0x0ff);
  buf[1] = (num1 & 0x0f) << 4;

  //尾数计算
  if (m1 > 0)
  {
    for (num1 = 0; m1 / 2 > 0; num1++)
    {
      temp1 = (temp1 << 1) + m1 % 2;
      m1 = m1 / 2;
    }
    for (i = 0; i < num1; i++)
    {
      temp2 = (temp2 << 1) + (temp1 & (0x01));
      temp1 = temp1 >> 1;
    }
    for (i = 0; i < 52 - num1; i++)
    {
      temp2 = (temp2 << 1) + floor(m2 * 2);
      m2 = m2 * 2 - floor(m2 * 2);
    }
  }
  else
  {
    do
    {
      m2 *= 2;
    } while (m2 < 1);
    m2 -= floor(m2);
    for (i = 0; i < 52; i++)
    {
      temp2 = (temp2 << 1) + floor(m2 * 2);
      m2 = m2 * 2 - floor(m2 * 2);
    }
  }

  //将转换后的数据存放到数组中
  for (i = 7; i > 1; i--)
  {
    buf[i] = temp2 & 0x0ff;
    temp2 = temp2 >> 8;
  }
  buf[1] += temp2 & 0x0f;

  ROS_INFO(byteToHexString(buf, 8).c_str());
}

class UdpDataProcess : public UdpProcessCallBack
{
public:
  UdpDataProcess(ros::NodeHandle node, int carnum);
  ~UdpDataProcess();
  void OnUdpProcessCallBack(unsigned char *data, int len, struct sockaddr_in addr_);

  void topic_Sender();

  void start();

  ros::NodeHandle nh;

  ros::Publisher obstacle_pub;

  int car_num;
};

UdpDataProcess::UdpDataProcess(ros::NodeHandle node, int carnum) : nh(node), car_num(carnum)
{
  ROS_DEBUG("UdpDataProcess");
  obstacle_pub = nh.advertise<perception_msgs::FusionDataInfo>("/perception/obstacle_info", 1);
}

UdpDataProcess::~UdpDataProcess()
{
}

void UdpDataProcess::OnUdpProcessCallBack(unsigned char *data, int len, struct sockaddr_in addr_)
{
  // ROS_INFO(byteToHexString(data, len).c_str());
  // ros::Time t1 = ros::Time::now();
  perception_msgs::FusionDataInfo per_obstacle;
  per_obstacle.header.stamp = ros::Time::now();
  per_obstacle.obstacle_num = car_num;

  double obstacle_data_lat = 0.0;
  double obstacle_data_lon = 0.0;
  double obstacle_data_height = 0.0;
  double obstacle_data_velocity = 0.0;
  double obstacle_data_angle_x = 0.0;

  double obstacle_data_l = 0.0;
  double obstacle_data_w = 0.0;
  double obstacle_data_h = 0.0;
  //在这里解析数据 解析
  //解析type 和 time
  int i = 0, j = 0, n = 0;
  for (i = 0; i < 2; i++)
  {
    unsigned char tmp_[8];
    for (j = 0; j < 8; j++)
    {
      tmp_[j] = data[i * 8 + (7 - j)];
    }
    double d_ = HexToDouble(tmp_);
    // ROS_INFO("header i = [%d], value =  [%f]", i, d_);
  }
  for (n = 0; n < car_num; n++)
  {
    int m = (2 * 8) + (n * 8 * 8);
    common_msgs::ObstacleInfo obstacle_;
    double d_[8] = {0.0};
    for (i = 0; i < 8; i++)
    {
      unsigned char tmp_[8];
      for (j = 0; j < 8; j++)
      {
        tmp_[j] = data[m + (i * 8) + (7 - j)];
      }
      d_[i] = HexToDouble(tmp_);
      // ROS_INFO("car[%d] i = [%d], value =  [%f]", n, i, d_[i]);
    }
    obstacle_data_lat = d_[0];    // x
    obstacle_data_lon = d_[1];    // y
    obstacle_data_height = d_[2]; // z

    obstacle_data_l = d_[3];        // l
    obstacle_data_w = d_[4];        // w
    obstacle_data_h = d_[5];        // h
    obstacle_data_velocity = d_[6]; // v
    obstacle_data_angle_x = d_[7];  // a

    obstacle_.id = n + 1;
    obstacle_.velocity = obstacle_data_velocity;
    obstacle_.theta = obstacle_data_angle_x;
    float test_angle_x = -(obstacle_data_angle_x - 90);

    if (abs(test_angle_x) > 360.0)
    {
      test_angle_x = fmod((test_angle_x * 180 / M_PI), 360.0);
    }
    float radian = test_angle_x * M_PI / 180;
    // ROS_INFO("radian = %f  and angle = %f and change angle = %f", radian, obstacle_data.angle_x, test_angle_x);

    obstacle_.peak[0].x = obstacle_data_lat + obstacle_data_l / 2 * cos(radian) - obstacle_data_w / 2 * sin(radian);
    obstacle_.peak[0].y = obstacle_data_lon + obstacle_data_l / 2 * sin(radian) + obstacle_data_w / 2 * cos(radian);

    obstacle_.peak[1].x = obstacle_data_lat + obstacle_data_l / 2 * cos(radian) + obstacle_data_w / 2 * sin(radian);
    obstacle_.peak[1].y = obstacle_data_lon + obstacle_data_l / 2 * sin(radian) - obstacle_data_w / 2 * cos(radian);

    obstacle_.peak[2].x = obstacle_data_lat - obstacle_data_l / 2 * cos(radian) + obstacle_data_w / 2 * sin(radian);
    obstacle_.peak[2].y = obstacle_data_lon - obstacle_data_l / 2 * sin(radian) - obstacle_data_w / 2 * cos(radian);

    obstacle_.peak[3].x = obstacle_data_lat - obstacle_data_l / 2 * cos(radian) - obstacle_data_w / 2 * sin(radian);
    obstacle_.peak[3].y = obstacle_data_lon - obstacle_data_l / 2 * sin(radian) + obstacle_data_w / 2 * cos(radian);

    per_obstacle.obstacles.push_back(obstacle_);
  }

  //解析后的数据 转换为TOPIC，可以在这里直接发送出去，
  obstacle_pub.publish(per_obstacle);

  // ros::Time t2 = ros::Time::now();
  // ROS_INFO_STREAM("time [" << ((t2 - t1).toNSec() / 1000000.0) << "] ");

  //也可以放到list里面，然后使用下面的topic_Sender线程函数发送出去
}

//发布话题
void UdpDataProcess::topic_Sender()
{
  //这里可以定时发布消息

  // ros::Publisher topic_pub_ = nh.advertise< perception_sensor_msgs::ObjectList >("/prescan/obstacle_location", 2);

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    loop_rate.sleep();
    // if (!list_.empty())
    // {

    // } // list_.empty()
  } // while (ros::ok())
}

void UdpDataProcess::start()
{
  ROS_DEBUG("start");
  boost::thread thrd(boost::bind(&UdpDataProcess::topic_Sender, this));
  thrd.detach();
}

int main(int argc, char *argv[])
{
  ROS_INFO("ROS node is start, name is [%s], file name is %s", NODE_NAME, argv[0]);
  ros::init(argc, argv, NODE_NAME); // node name
  ros::NodeHandle nh_;

  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

  //////////////////yaml file
  std::string path_ = ros::package::getPath("simlink_udp");

  std::string yaml_file_name = path_ + "/cfg/config.yaml";
  ROS_DEBUG("ROS package path [%s]", yaml_file_name.c_str());

  YAML::Node doc;
  doc = YAML::LoadFile(yaml_file_name);

  //发送UDP数据的设备IP，这里就是WINDOWS电脑的IP地址
  std::string device_ip_ = doc["device_ip"].as<string>();
  //接收UDP数据的端口
  int intput_port_ = doc["intput_port"].as<int>();
  //数据包中包含的车辆数目
  int car_num_ = doc["car_num"].as<int>();

  ROS_INFO_STREAM("IP address:" << device_ip_.c_str() << " intput_port:" << intput_port_ << " car_num:" << car_num_);

  // udp数据包长度 (车辆数目*每辆车的指标数+包头（TYPE+TIME）)*8(一个double占8个字节)
  int data_size_ = (car_num_ * 8 + 2) * 8;

  boost::shared_ptr<UdpProcess> udp_(new UdpProcess());

  // radar data process class
  boost::shared_ptr<UdpDataProcess> data_process_(new UdpDataProcess(nh_, car_num_));

  if (udp_->Initial(data_process_, device_ip_, intput_port_, data_size_) == 1)
  {
    data_process_->start();
  }
  else
  {
    ROS_DEBUG_STREAM("IP address:" << device_ip_.c_str());
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////

  ros::Rate loop_rate(100);

  int icount = 0;

  while (ros::ok())
  {
    icount++;

    ros::spinOnce();

    loop_rate.sleep();
  }
  // ros::spin();
  return 0;
}