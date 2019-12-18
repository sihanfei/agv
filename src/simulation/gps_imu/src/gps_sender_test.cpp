#include "nav_msgs/Path.h"
#include "novalet_gps/NovatelPosition.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <map>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#define DATA_PATH "/work/superg_agv/src/data/data_test"
//#define DATA_NAME "/new_gps_data.csv"  //,时间-秒,航向角-度,84坐标Y-米,84坐标X-米,障碍物距离,速度-公里/时
#define DATA_NAME "/test2circle.csv" //时间\目标加速度\经度\纬度\航向\预瞄点偏差\84-Y\84-X\目标转向角度\障碍物距离\车速
//#define DATA_NAME "/map_source.csv"

#define FRE 100
#define BUF_LEN 10

//#define M_PI acos(-1)

#define OFFSET_84_Y 22942
#define OFFSET_84_X 33436

using namespace std;

int split(char dst[][80], char *str, const char *spl)
{
  int n        = 0;
  char *result = NULL;
  result       = strtok(str, spl);
  while (result != NULL)
  {
    strcpy(dst[n++], result);
    result = strtok(NULL, spl);
  }
  return n;
}

int main(int argc, char **argv)
{

  double xx = atan2(2, 0);

  printf("atan(0, 0) %lf\n", xx);
  std::map< double, double > test;
  for (size_t i = 0; i < 10; i++)
  {
    test.insert(make_pair(i * 1.1, i * i * 100.001));
  }
  std::map< double, double >::iterator it;
  for (it = test.begin(); it != test.end(); ++it)
  {
    printf("%lf -> %lf \n", it->first, it->second);
  }
  it = test.begin();
  printf("TEST:%lf\n", it->first);
  test.erase(it->first);
  it = test.begin();
  printf("TEST:%lf\n", it->first);

  sleep(1000);

  ros::init(argc, argv, "gps_sender");

  ros::NodeHandle n;

  //设置GPS消息格式

  ros::Publisher n_gps_pub = n.advertise< novalet_gps::NovatelPosition >("novatel_position", 1, true);
  ros::Publisher g_gps_pub = n.advertise< nav_msgs::Odometry >("odom_msg", BUF_LEN, true);
  //  ros::Publisher p_gps_pub = n.advertise<nav_msgs::Path>("path_msg",1, true);

  //修改循环周期
  ros::Rate loop_rate(FRE);

  int count               = 0;
  uint32_t count_CHC_send = 0;

  //读取GPS文件:$GPGGA 纯卫导数据 和 $GPCHC 组合导航数据
  char *line = NULL;

  //获取routing data path
  char *home_path                 = getenv("HOME");
  char route_date_path_name[1024] = {0};
  sprintf(route_date_path_name, "%s" DATA_PATH "" DATA_NAME, home_path);
  printf("route date path name:%s\n", route_date_path_name);

  FILE *fp;
  size_t len = 0;
  ssize_t read;
  fp = fopen(route_date_path_name, "r");

  if (fp == NULL)
  {
    printf("fail to read");
    exit(1);
  }

  ros::Time current_time, last_time;

  last_time = ros::Time::now();

  double last_Z = 0.0;

  uint32_t count_CHC_send_new = 0;

  geometry_msgs::TransformStamped odom_trans;
  nav_msgs::Odometry odom;
  tf::TransformBroadcaster odom_broadcaster;

  while (ros::ok())
  {

    read = getline(&line, &len, fp);
    // printf("Retrieved line of length %zu %d:\n",read,count);
    ++count;

    if (feof(fp))
    {
      break;
    }
    if (read > 0)

    // if(read > 10 && count > 20000)
    {
      // CHC
      // if(line[3] == 'C' && line[4] == 'H' && line[5] == 'C' )
      if (1)
      {
        ++count_CHC_send;
        char dst[26][80];
        int cnt = split(dst, line, ",");

#if 0 //华测数据处理发布
        
        novalet_gps::NovatelPosition n_gps_p;
        
        n_gps_p.header.stamp = ros::Time::now();
        n_gps_p.header.frame_id = "odom";
        n_gps_p.novatel_msg_header.sequence_num = count_CHC_send;
        n_gps_p.novatel_msg_header.message_name = dst[0];
        n_gps_p.novatel_msg_header.gps_week_num = strtoul(dst[1],NULL,0);
        n_gps_p.novatel_msg_header.gps_seconds = atof(dst[2]);
        n_gps_p.solution_status = 1;
        n_gps_p.position_type = 1;
        n_gps_p.lat = atof(dst[12]);//纬度
        n_gps_p.lon = atof(dst[13]);//经度
        n_gps_p.height = atof(dst[14]);//高度
        n_gps_p.undulation = atof(dst[3]);//偏航角
        n_gps_p.lat_sigma = atof(dst[4]);//俯仰角
        n_gps_p.lon_sigma = atof(dst[15]);//东向速度
        n_gps_p.height_sigma = atof(dst[16]);//北向速度

        n_gps_pub.publish(n_gps_p);

#endif

        novalet_gps::NovatelPosition n_gps_p;

        n_gps_p.header.stamp                    = ros::Time::now();
        n_gps_p.header.frame_id                 = "odom";
        n_gps_p.novatel_msg_header.sequence_num = count_CHC_send;
        n_gps_p.novatel_msg_header.message_name = "$GPCHC";
        n_gps_p.novatel_msg_header.gps_week_num = 0;
        n_gps_p.novatel_msg_header.gps_seconds  = atof(dst[0]);
        n_gps_p.solution_status                 = 1;
        n_gps_p.position_type                   = 1;
        n_gps_p.lat                             = atof(dst[6]) - OFFSET_84_Y;                      //纬度
        n_gps_p.lon                             = atof(dst[7]) - OFFSET_84_X;                      //经度
        n_gps_p.height                          = 0.0;                                             //高度
        n_gps_p.undulation                      = M_PI / 2 - atof(dst[4]);                         //偏航角
        n_gps_p.lat_sigma                       = 0.0;                                             //俯仰角
        n_gps_p.lon_sigma                       = (atof(dst[10]) / 3.6) * cos(n_gps_p.undulation); //东向速度
        n_gps_p.height_sigma                    = (atof(dst[10]) / 3.6) * sin(n_gps_p.undulation); //北向速度

        n_gps_pub.publish(n_gps_p);

#if 0

        double gps_lat = atof(dst[2]); //纬度
        double gps_lon = atof(dst[3]); //经度
        double gps_speed =  atof(dst[5])/3.6; //速度.换算为m/s
        double gps_time = atof(dst[0]);//时间
        ROS_INFO("%f %f %f %f %f",gps_time,gps_lat,gps_lon,gps_speed,cos(3.14));

        double X = 0.0;//横滚角  R
        double Y = 0.0;//俯仰角  P
        double Z = M_PI/2 - atof(dst[1]);//偏航角  Y
#endif

        //时间\目标加速度\经度\纬度\航向\预瞄点偏差\84-Y\84-X\目标转向角度\障碍物距离\车速

        double gps_time  = atof(dst[0]);               //时间
        double gps_lat   = atof(dst[6]) - OFFSET_84_Y; //纬度 84-Y
        double gps_lon   = atof(dst[7]) - OFFSET_84_X; //经度 84-X
        double gps_speed = atof(dst[10]) / 3.6;        //速度.换算为m/s
        double X         = 0.0;                        //横滚角  R
        double Y         = 0.0;                        //俯仰角  P
        double Z         = M_PI / 2 - atof(dst[4]);    //偏航角  Y
        //        ROS_INFO("%f %f %f %f %f",gps_time,gps_lat,gps_lon,gps_speed,Z);

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(Z);

        //        goal_quat.x = cos(Z/2)*cos(Y/2)*cos(X/2) - sin(Z/2)*sin(Y/2)*sin(X/2);
        //        goal_quat.y = cos(Z/2)*cos(Y/2)*sin(X/2) + sin(Z/2)*sin(Y/2)*cos(X/2);
        //        goal_quat.z = cos(Z/2)*sin(Y/2)*cos(X/2) - sin(Z/2)*cos(Y/2)*sin(X/2);
        //        goal_quat.w = sin(Z/2)*cos(Y/2)*cos(X/2) + cos(Z/2)*sin(Y/2)*sin(X/2);
        current_time = ros::Time::now();

        odom_trans.header.stamp    = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id  = "base_link";

        odom_trans.transform.translation.x = gps_lat; //纬度
        odom_trans.transform.translation.y = gps_lon; //经度
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation      = goal_quat;

        //        odom_broadcaster.sendTransform(odom_trans);

        odom.header.stamp    = current_time;
        odom.header.frame_id = "odom";
        // set the position
        odom.pose.pose.position.x  = gps_lat; //纬度;
        odom.pose.pose.position.y  = gps_lon; //经度;
        odom.pose.pose.position.z  = 0.0;
        odom.pose.pose.orientation = goal_quat;

        // set the velocity
        odom.child_frame_id        = "base_link";
        odom.twist.twist.linear.x  = gps_speed * cos(Z); //东向速度
        odom.twist.twist.linear.y  = gps_speed * sin(Z); //北向速度
        odom.twist.twist.angular.z = Z - last_Z;

        //        g_gps_pub.publish(odom);

        last_Z    = Z;
        last_time = current_time;
#if 0
        nav_msgs::Path g_gps_p;
        geometry_msgs::PoseStamped this_pose_stamped;

        double Y = atof(dst[4]);//俯仰角
        double Z = atof(dst[5]);//横滚角
        /*
        this_pose_stamped.pose.orientation.x = sin(Y/2)*sin(Z/2)*cos(X/2)+cos(Y/2)*cos(Z/2)*sin(X/2); 
        this_pose_stamped.pose.orientation.y = sin(Y/2)*cos(Z/2)*cos(X/2)+cos(Y/2)*sin(Z/2)*sin(X/2); 
        this_pose_stamped.pose.orientation.z = cos(Y/2)*sin(Z/2)*cos(X/2)-sin(Y/2)*cos(Z/2)*sin(X/2); 
        this_pose_stamped.pose.orientation.w = cos(Y/2)*cos(Z/2)*cos(X/2)-sin(Y/2)*sin(Z/2)*sin(X/2);
        */

        current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec(); 
        double delta_th = vth * dt;
        th += delta_th;
        //geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th);

        
        

        

        nav_msgs::Path path;
        geometry_msgs::PoseStamped this_pose_stamped;

        this_pose_stamped.header.stamp = current_time;
        this_pose_stamped.header.frame_id = "odom";
        this_pose_stamped.pose.position.x = atof(dst[12])*89760 - 2890800;//纬度
        this_pose_stamped.pose.position.y = atof(dst[13])*111120 - 13264882;//经度

        //this_pose_stamped.pose.orientation = goal_quat;
        
        this_pose_stamped.pose.orientation.x = 0;
			  this_pose_stamped.pose.orientation.y = 0;
			  this_pose_stamped.pose.orientation.z = 0;
        this_pose_stamped.pose.orientation.w = 1;

        path.poses.push_back(this_pose_stamped);

        path.header.frame_id = "odom";
        path.header.stamp = current_time;

      //  p_gps_pub.publish(path);

        last_time = current_time;
#endif
        ros::spinOnce();
        loop_rate.sleep();
        //        printf("CHC: %u\n",count_CHC_send);
        /*
        printf("CHC:%f - %f - %f\n",n_gps_p.novatel_msg_header.gps_seconds,
                    n_gps_p.lon,
                    n_gps_p.lat);
        */
      }
    }
  }
  fclose(fp);

  printf("\nCHC: %d\n", count_CHC_send);
  ROS_INFO("shutting down!\n");
  ros::shutdown();

  return 0;
}