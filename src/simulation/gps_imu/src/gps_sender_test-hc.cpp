#include "ros/ros.h"
#include "std_msgs/String.h"
#include "novalet_gps/NovatelPosition.h"
#include "nav_msgs/Path.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <iostream>


#define DATA_PATH "/Data"
#define DATA_NAME "/c-3"

using namespace std;

int split(char dst[][80], char* str, const char* spl)
{
  int n = 0;
  char *result = NULL;
  result = strtok(str, spl);
  while( result != NULL )
  {
    strcpy(dst[n++], result);
    result = strtok(NULL, spl);
  }
  return n;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "gps_sender");

  ros::NodeHandle n;

  //设置GPS消息格式

  ros::Publisher n_gps_pub = n.advertise<novalet_gps::NovatelPosition>("novatel_position",1, true);
  ros::Publisher g_gps_pub = n.advertise<nav_msgs::Odometry>("odom_msg",1, true);
//  ros::Publisher p_gps_pub = n.advertise<nav_msgs::Path>("path_msg",1, true);
  
  //修改循环周期
  ros::Rate loop_rate(1000);

  int count_send = 0;
  int count = 0;
  uint32_t count_CHC_send = 0;
  int count_GGA_send = 0;

  //读取GPS文件:$GPGGA 纯卫导数据 和 $GPCHC 组合导航数据
  char *line = NULL;
  //char * file_name = "/home/wrll/Data/c-3";

  //获取routing data path
  char *home_path = getenv("HOME");
  char route_date_path_name[1024] = {0};
  sprintf(route_date_path_name,"%s"DATA_PATH""DATA_NAME,home_path);
  printf("route date path name:%s\n", route_date_path_name);


  FILE *fp;
  size_t len = 0;
  ssize_t read;
  fp = fopen(route_date_path_name,"r");
  
  if(fp  == NULL)
  {
    printf("fail to read");
    exit (1) ;
  }
  
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  double th = 0.0;
  double vth = 0.0;

  uint32_t count_CHC_send_new = 0;


  while(ros::ok())
  {

    read = getline(&line, &len, fp);
    //printf("Retrieved line of length %zu %d:\n",read,count);
    ++count;

    if(feof(fp)){
      break;
    }

    if(read > 10 && count > 20000)
    {
      //CHC
      if(line[3] == 'C' && line[4] == 'H' && line[5] == 'C' )
      {
        ++count_CHC_send;
        char dst[26][80];
        char ddd[1024]; 
        int cnt = split(dst,line,",");
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

//        n_gps_pub.publish(n_gps_p);

      //  nav_msgs::Path g_gps_p;
      //  geometry_msgs::PoseStamped this_pose_stamped;
 
        
//        double Y = 0.0;//俯仰角
//        double Z = 0.0;//横滚角


        double X = atof(dst[5]);//横滚角  R
        double Y = atof(dst[4]);//俯仰角  P
        double Z = atof(dst[3]);//偏航角  Y
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
        geometry_msgs::Quaternion goal_quat;
        
        
        goal_quat.x = cos(Z/2)*cos(Y/2)*cos(X/2) - sin(Z/2)*sin(Y/2)*sin(X/2);
        goal_quat.y = cos(Z/2)*cos(Y/2)*sin(X/2) + sin(Z/2)*sin(Y/2)*cos(X/2); 
        goal_quat.z = cos(Z/2)*sin(Y/2)*cos(X/2) - sin(Z/2)*cos(Y/2)*sin(X/2); 
        goal_quat.w = sin(Z/2)*cos(Y/2)*cos(X/2) + cos(Z/2)*sin(Y/2)*sin(X/2);
        
        
        geometry_msgs::TransformStamped odom_trans;
        nav_msgs::Odometry odom;
        tf::TransformBroadcaster odom_broadcaster;

        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = atof(dst[12])*89760 - 2890820;//纬度
        odom_trans.transform.translation.y = atof(dst[13])*111120 - 13264882;//经度
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = goal_quat;

        odom_broadcaster.sendTransform(odom_trans);

        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        //set the position
        odom.pose.pose.position.x = atof(dst[12])*89760 - 2890820;//纬度;
        odom.pose.pose.position.y = atof(dst[13])*111120 - 13264882;//经度;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = goal_quat;
 
        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = atof(dst[15]);//东向速度
        odom.twist.twist.linear.y = atof(dst[16]);//北向速度
        odom.twist.twist.angular.z = vth;       

        g_gps_pub.publish(odom);
        

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
        
        ros::spinOnce();
        loop_rate.sleep();  
        printf("CHC: %u\n",count_CHC_send);
        /*      
        printf("CHC:%f - %f - %f\n",n_gps_p.novatel_msg_header.gps_seconds,
                    n_gps_p.lon,
                    n_gps_p.lat);
        */
      }
      //GGA
      else if (line[3] == 'G' && line[4] == 'G' && line[5] == 'A' )
      {
        ++count_GGA_send;
        char dst[26][80]; 
        int cnt = split(dst,line,",");

        //printf("GGA:%d:%d:%s\n",count_GGA_send,cnt,dst[0]);
      }
    }
  }
  fclose(fp);

  printf("\nCHC: %d\n",count_CHC_send);
  ROS_INFO("shutting down!\n");
	ros::shutdown();

  return 0;
}