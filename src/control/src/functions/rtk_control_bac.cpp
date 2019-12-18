#include "control.h"

#define PI 3.141592
#define F_KP 2.58  // P constant for PSD translation controller
#define F_KD 0.047  // D constant for PSD translation controller
#define F_KI 0.0  // S constant for PSD translation controller
#define R_KP 0.75  // P constant for PSD rotation controller
#define R_KD 0.1  // D constant for PSD rotation controller
#define R_KI -0.0000001  // S constant for PSD rotation controller

//#define R_KD 0.0  // D constant for PSD rotation controller
//#define R_KI -0.0000000  // S constant for PSD rotation controller



#define tolerance_latteral_offset 0.1
#define tolerance_speed 0.1
#define tolerance_station 0.1

#define max_speed 20.0
#define max_angle 2.0

#define FRE 100
#define BUF_LEN 10



using namespace std;
using namespace control;

test_msgs::TestType testmsg;

typedef struct position
{
  u_int32_t n_gps_sequence_num;
  float lon;
  float lat;
  float height;
  float velocity_x;
  float velocity_y;
  float heading;
  float pitch;
  float gps_seconds;
}PositionInfo;

typedef struct PID_parameter
{
  double station_error;
  double speed_error;
  double latteral_offset_preview;
  float gps_seconds;
}pidp;

typedef struct PID_sum_error
{
  double latteral_offset;
  double speed;
  double station;
}pidse;


typedef  struct shm_buf
{
    char read_tip;
    char gps_data[sizeof(novalet_gps::NovatelPosition)];
}shmb;

//实际位姿
PositionInfo real_position = {0};

//开始位置
pidp start_parameter = {0};
//上一次位置
pidp last_parameter = {0};
//累计误差
pidse sum_error = {0};

//收到GPS数据后开始启动判断
int start_tip = 1;

//PID计算启动标志
int PID_start_tip = 0;

//最大小值初始化
float real_position_x_MAX = 0;
float real_position_y_MAX = 0;
float real_position_x_MIN = 180*89760;
float real_position_y_MIN = 90*111120;

//判断是否有地图
int route_tip = 0;

//定义参考点长度 5000*5的数组  
vector <PositionInfo> route_data;

u_int32_t num = 0;


//关闭
void controlShutDown(){
  ROS_INFO("shutting down!\n");
  ros::shutdown();
}


void saveRouteData()
{
  FILE * route_data_save_fp = NULL;
  char *home_path = getenv("HOME");
  char route_date_path_name[1024] = {0};
  sprintf(route_date_path_name,"%s"DATA_PATH""DATA_NAME,home_path);

  printf("route date path name:%s\n", route_date_path_name);

	char end1 =0x0d;// "/n"
  char end2 =0x0a;// "/r" 

  route_data_save_fp = fopen(route_date_path_name,"w+");

  if(route_data_save_fp  == NULL)
  {
    printf("fail to read");
    exit (1) ;
  }

  ROS_INFO("save file size is: %d",route_data.size());

  for(int i = 0; i < route_data.size(); i++)
  {
    
		//fprintf(sentipsave,"%d",LeaveTip[sennum]);
		fprintf(route_data_save_fp,"%u %f %f %f %f %f %f %f%c%c",
                                route_data[i].n_gps_sequence_num,
                                route_data[i].lon,
                                route_data[i].lat,
                                route_data[i].height,//
                                route_data[i].velocity_x,
                                route_data[i].velocity_y,
                                route_data[i].heading,
                                route_data[i].pitch,//
                                end1,
                                end2
           );
	}

  fclose(route_data_save_fp);
}

double last_ans_PID = 0;

double calculatePID(
                    double new_error, 
                    double last_error, 
                    float new_time, 
                    float last_time,
                    double kP, 
                    double kD, 
                    double kI,
                    double *sum)
{
 
  double ans_PID = 0;
  float dt = new_time - last_time;

  if (dt <= 0) {
    ROS_INFO("dt <= 0, control = 0");
    return ans_PID;
  }
  
  double diff = (new_error - last_error)/dt;
  *sum = *sum + new_error * dt * kI;
  //sum_error.latteral_offset += new_error * dt * kI;

  ans_PID = new_error * kP + *sum + diff * kD;
  
//  ROS_INFO("dt: %f > 0, ans = %f, a_speed = %f\n",dt,ans_PID*180/M_PI,(ans_PID - last_ans_PID)*180/M_PI);

  last_ans_PID = ans_PID;

  return ans_PID;
}

double PID_control(pidp &actual_parameter, PositionInfo &actual_position)
{
  static double out_angle = 0;
  
  //ROS_INFO("PID calculate~~~");
  if (PID_start_tip = 0) {
    start_parameter.gps_seconds = actual_parameter.gps_seconds;
    start_parameter.speed_error = actual_parameter.speed_error;
    start_parameter.station_error = actual_parameter.station_error;
    start_parameter.latteral_offset_preview = actual_parameter.latteral_offset_preview;
    
    last_parameter.gps_seconds = actual_parameter.gps_seconds;
    last_parameter.speed_error = actual_parameter.speed_error;
    last_parameter.station_error = actual_parameter.station_error;
    last_parameter.latteral_offset_preview = actual_parameter.latteral_offset_preview;

    sum_error.station = 0;
    sum_error.latteral_offset = 0;
    sum_error.speed = 0;

  }
  PID_start_tip++;
  
  // 

  if (fabs(actual_parameter.latteral_offset_preview)> tolerance_latteral_offset) {
    //out_angle = actual_position.heading + calculatePID(
    out_angle = calculatePID(
                                                        actual_parameter.latteral_offset_preview,
                                                        last_parameter.latteral_offset_preview,
                                                        actual_parameter.gps_seconds,
                                                        last_parameter.gps_seconds,
                                                        R_KP,
                                                        R_KD,
                                                        R_KI,
                                                        &sum_error.latteral_offset
                                                        );
//    ROS_INFO("Control angle: %lf\t now angle: %lf\t",out_angle*180/M_PI,actual_position.heading*180/M_PI);
  }
  

  if (fabs(actual_parameter.speed_error)> tolerance_speed) {
    
  }

  if (fabs(actual_parameter.station_error)> tolerance_station) {
    
  }

  last_parameter.gps_seconds = actual_parameter.gps_seconds;
  last_parameter.speed_error = actual_parameter.speed_error;
  last_parameter.station_error = actual_parameter.station_error;
  last_parameter.latteral_offset_preview = actual_parameter.latteral_offset_preview;

  return out_angle;
  
}

void gpsCallback(const novalet_gps::NovatelPositionConstPtr& msg)
{
  if(msg->novatel_msg_header.sequence_num > 2700)
  {
    //ROS_INFO("receive: %u %lu",msg->novatel_msg_header.sequence_num,route_data.size());
    if (route_tip == 0) {
      ROS_INFO("save route data!\n");
      saveRouteData();    
    }
    controlShutDown();
  }
  else
  {
    real_position.n_gps_sequence_num = msg->novatel_msg_header.sequence_num;
    real_position.lon = msg->lon; //经 
    real_position.lat = msg->lat;//纬 
//    real_position.height = msg->height;    //高
    real_position.height = 0.0;    //高



    real_position.velocity_x = msg->lon_sigma;//东向速度
    real_position.velocity_y = msg->height_sigma;//北向速度
//    real_position.heading = msg->undulation;//偏航角度
//    real_position.pitch = msg->lat_sigma;//俯仰角
    real_position.heading = 0.0;//偏航角度
    real_position.pitch = 0.0;//俯仰角
    real_position.gps_seconds = msg->novatel_msg_header.gps_seconds;//gps秒时间

//    ROS_INFO("receive %u %lf %lf",msg->novatel_msg_header.sequence_num,real_position.lon,real_position.lat);

    
    num++;
  }
  start_tip = 2;
}

void gps_test_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{


    real_position.n_gps_sequence_num = msg->header.seq;
    real_position.lon = msg->pose.pose.position.y; //经 
    real_position.lat = msg->pose.pose.position.x;//纬 
    real_position.height = 0.0;    //高
    real_position.velocity_x = msg->twist.twist.linear.x;//东向速度
    real_position.velocity_y = msg->twist.twist.linear.y;//北向速度
    real_position.heading = 0.0;//偏航角度 变化量
    real_position.pitch = 0.0;//俯仰角
    real_position.gps_seconds = msg->header.stamp.sec;//gps秒时间

    if(msg->header.seq >21100)
    {
      ROS_INFO("receive %u %u %u %u",num,msg->header.seq,msg->header.stamp.sec,msg->header.stamp.nsec);
        //ROS_INFO("receive: %u %lu",msg->novatel_msg_header.sequence_num,route_data.size());
        if (route_tip == 0) {
          route_tip = 1;
          ROS_INFO("save route data!\n");
          saveRouteData();    
        }
        controlShutDown();
    }
    num++; 
  start_tip = 2;
}

// Squared distance from the point to (x, y).
double pointDistanceSquare(PositionInfo &pre_point, PositionInfo &route_point) 
{
  const double dx = pre_point.lon - route_point.lon;
  const double dy = pre_point.lat - route_point.lat;
  return dx * dx + dy * dy;
}

double speedSquare(const double dx , const double dy)
{
  return dx * dx + dy * dy;
}

double norm2(const double dx , const double dy)
{
  return sqrt(dx * dx + dy * dy);
}


void jointPub(ros::Publisher &j_pub, double &control_alex_angle)
{
      sensor_msgs::JointState joint_state; 
      joint_state.header.stamp = ros::Time::now(); 
      joint_state.name.resize(2); 
      joint_state.position.resize(2); 
      //f
      joint_state.name[0]="base_to_front_axle_steering_shaft"; 
      joint_state.position[0] = control_alex_angle; 
      //s
      joint_state.name[1]="base_to_back_axle_steering_shaft"; 
      joint_state.position[1] = control_alex_angle; 
      
      j_pub.publish(joint_state);
}

void odomPub(ros::Publisher &odom_pub,
             geometry_msgs::TransformStamped &odom_tf,
             nav_msgs::Odometry &odom_nav,
             tf::TransformBroadcaster &odom_tf_b,
             PositionInfo &rpt
             )
{

        double gps_lat = rpt.lat; //纬度 84-Y
        double gps_lon = rpt.lon; //经度 84-X
        double gps_speed_x =  rpt.velocity_x; //速度.换算为m/s
        double gps_speed_y =  rpt.velocity_y; //速度.换算为m/s
        double X = 0.0;//横滚角  R
        double Y = 0.0;//俯仰角  P
        double Z = rpt.heading;//偏航角  Y

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(Z);
        
        ros::Time current_time = ros::Time::now();

        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";

        odom_tf.transform.translation.x = gps_lat;//纬度
        odom_tf.transform.translation.y = gps_lon ;//经度
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation = goal_quat;

        odom_tf_b.sendTransform(odom_tf);

        odom_nav.header.stamp = current_time;
        odom_nav.header.frame_id = "odom";
        //set the position
        odom_nav.pose.pose.position.x = gps_lat;//纬度;
        odom_nav.pose.pose.position.y = gps_lon;//经度;
        odom_nav.pose.pose.position.z = 0.0;
        odom_nav.pose.pose.orientation = goal_quat;
 
        //set the velocity
        odom_nav.child_frame_id = "base_link";
        odom_nav.twist.twist.linear.x = gps_speed_x;//东向速度
        odom_nav.twist.twist.linear.y = gps_speed_y;//北向速度
        odom_nav.twist.twist.angular.z = Z;       

        odom_pub.publish(odom_nav);

}


//
int main(int argc, char **argv)
{
  //printf("GGA:%d\n");
  
  ros::init(argc, argv, "RTK_control");
  //启动多线程

  ros::NodeHandle n;

  //PID初始化
//  PIDControl pid_control_angle;    
  
//  PIDConf pid_conf_angle;
//  PIDConf pid_conf_speed;

  //获取routing data path
  char *home_path = getenv("HOME");
  char route_date_path_name[1024] = {0};
  sprintf(route_date_path_name,"%s"DATA_PATH""DATA_NAME,home_path);
  printf("route date path name:%s\n", route_date_path_name);

  //读取地图 route_tip = 0 无图 route_tip = 1 有图
  FILE * route_data_read_fp = NULL;
  route_data_read_fp = fopen(route_date_path_name,"r");
	PositionInfo read_position = {0};
  if(route_data_read_fp == NULL){
		printf("Open route_data.bin error!\n");
    route_tip = 0;
	}
	else{
		printf("route_date.bin is open!\n");
    while (!feof(route_data_read_fp)){
      fscanf(route_data_read_fp,"%u %f %f %f %f %f %f %f",
                                &read_position.n_gps_sequence_num,
                                &read_position.lon,
                                &read_position.lat,
                                &read_position.height,
                                &read_position.velocity_x,
                                &read_position.velocity_y,
                                &read_position.heading,
                                &read_position.pitch
                                );
      route_data.push_back(read_position);
    }  
      
		fclose(route_data_read_fp);
    printf("read %lu row, route_date is ok!\n",route_data.size());
    route_tip = 1;
  }
  
  ros::Subscriber n_gps_sub = n.subscribe("novatel_position", BUF_LEN, gpsCallback);
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
  ros::Publisher g_gps_pub = n.advertise<nav_msgs::Odometry>("odom_msg",BUF_LEN, true);
  ros::Publisher pre_point_pub = n.advertise<geometry_msgs::PointStamped>("point_msg",BUF_LEN, true);
  ros::Publisher pre_point_pub_gps = n.advertise<geometry_msgs::PointStamped>("point_msg_gps",BUF_LEN, true);

//  ros::Subscriber n_gps_sub = n.subscribe("odom_msg", BUF_LEN, gps_test_Callback);

  geometry_msgs::TransformStamped odom_trans;
  nav_msgs::Odometry odom;
  tf::TransformBroadcaster odom_broadcaster;  

  ros::Rate loop_rate(FRE);

  u_int32_t call_back_num = 0;

  while(ros::ok()){
    if(route_tip == 0 && start_tip == 2){
      //采集地图 按照每m一个点存储,以x为基础存储,每个不同的xy存储一次,x经度为1m
      if (route_data.size() == 0) {
        route_data.push_back(real_position);
        printf("route: %f %f\n",route_data.back().lon,route_data[0].lat);
        //写入文件 
        //printf("route: %d\n",route_data.size()); 
      }
      else
      {
        if (real_position.lon != route_data.back().lon || real_position.lat != route_data.back().lat) {
          route_data.push_back(real_position);
//          printf("real:  %f %f\n",real_position.lon,real_position.lat);
//          printf("route: %f %f\n",route_data.back().lon,route_data.back().lat);
//          printf("route:%d %f %f\n",route_data.size(),route_data.back().lon - route_data.front().lon,route_data.back().lat - route_data.front().lat);

        }
      }
      start_tip = 1;
    }
    else if (route_tip == 1 && start_tip == 2)
    {
      start_tip = 2;
      //start_tip = 1;
      //ROS_INFO("goto location");
      PositionInfo real_pos_temp = {0};
      //车辆质心真实点坐标
      real_pos_temp.lon = real_position.lon;
      real_pos_temp.lat = real_position.lat;
      real_pos_temp.heading = real_position.heading;
      real_pos_temp.velocity_x = real_position.velocity_x;
      real_pos_temp.velocity_y = real_position.velocity_y;
      real_pos_temp.gps_seconds = real_position.gps_seconds;
      real_pos_temp.n_gps_sequence_num = real_position.n_gps_sequence_num;

      //寻找预瞄点  参考线上loc_num的下一个点,且heading方向一致
      //ROS_INFO("find pre point");
      float delta_T = 1.2;
      float pre_length_consant = 0;
      float pre_length_y = max(3*cos(real_pos_temp.heading),
                              min(15*cos(real_pos_temp.heading),
                                  delta_T*real_pos_temp.velocity_x 
                                  + 
                                  pre_length_consant*cos(real_pos_temp.heading)
                                  )
                              );
      float pre_length_x = max(3*sin(real_pos_temp.heading),
                              min(15*sin(real_pos_temp.heading),
                                  delta_T*real_pos_temp.velocity_y 
                                  + 
                                  pre_length_consant*sin(real_pos_temp.heading)
                                  )
                              );
      //预瞄点
      PositionInfo pre_pos_temp = {0};
      pre_pos_temp.lon = real_pos_temp.lon + pre_length_x;
      pre_pos_temp.lat = real_pos_temp.lat + pre_length_y;
      pre_pos_temp.heading = real_pos_temp.heading;
      pre_pos_temp.gps_seconds = real_pos_temp.gps_seconds;
      pre_pos_temp.n_gps_sequence_num = real_pos_temp.n_gps_sequence_num;

      //最小索引 预瞄点在规划路径上的最接近点 按照最近位置计算,需要增加按照时间计算
      int min_index = findClosestRefPoint(pre_pos_temp);
      //规划点
      PositionInfo desire_pos_temp = {0};
      desire_pos_temp.lon = route_data[min_index].lon;
      desire_pos_temp.lat = route_data[min_index].lat;
      desire_pos_temp.heading = route_data[min_index].heading;
      desire_pos_temp.velocity_x = route_data[min_index].velocity_x;
      desire_pos_temp.velocity_y = route_data[min_index].velocity_y;
      //最小距离 距离的平方
      double min_distance = pointDistanceSquare(pre_pos_temp,desire_pos_temp);
      
      //纵向控制, 预瞄点与参考线上点的误差,现阶段的误差,用于PID的输入误差,用于PID的输出控制
      double dy = desire_pos_temp.lon - pre_pos_temp.lon;
      double dx = desire_pos_temp.lat - pre_pos_temp.lat;
      double dh = desire_pos_temp.heading - pre_pos_temp.heading;
      
      double cos_ref_heading = cos(desire_pos_temp.heading);
      double sin_ref_heading = sin(desire_pos_temp.heading);

      //位置偏差
      double station_error = dx * cos_ref_heading + dy * sin_ref_heading;
      
      //速度偏差
      double speed_error_x = desire_pos_temp.velocity_x - real_pos_temp.velocity_x;
      double speed_error_y = desire_pos_temp.velocity_y - real_pos_temp.velocity_y;
      double speed_error = sqrt(speedSquare(speed_error_x,speed_error_y));  
      if (speedSquare(desire_pos_temp.velocity_x,desire_pos_temp.velocity_y) 
          < 
          speedSquare(real_pos_temp.velocity_x,real_pos_temp.velocity_y)) 
      {
        speed_error = -1 * speed_error;
      }
      
      //横向控制
      //double latteral_error = dy*cos(dh) - dx*sin(dh);
      double dx_1,dy_1,dx_2,dy_2;
      double latteral_offset_preview = 0.0;
      int latteral_control_tip = 0;
      //目标点
      PositionInfo target_pos_temp = {0};
      
      if (min_index + 1 > route_data.size()) {
        ROS_INFO("a new loop");
        min_index = 0;
      }


        target_pos_temp.lon = route_data[min_index+1].lon;
        target_pos_temp.lat = route_data[min_index+1].lat;
        target_pos_temp.heading = route_data[min_index+1].heading;
        target_pos_temp.velocity_x = route_data[min_index+1].velocity_x;
        target_pos_temp.velocity_y = route_data[min_index+1].velocity_y;
  
        dx_1 = target_pos_temp.lat - desire_pos_temp.lat;
        dy_1 = target_pos_temp.lon - desire_pos_temp.lon;

        dy_2 = pre_pos_temp.lon - desire_pos_temp.lon;
        dx_2 = pre_pos_temp.lat - desire_pos_temp.lat;

        latteral_offset_preview = abs(dx_1*dy_2 - dy_1*dx_2)/norm2(dx_1,dy_1);
        
        if ((-1 * dy_2*dx_1  + dx_2 * dy_1) < 0) {
          latteral_offset_preview = -1 * latteral_offset_preview;         
        }
        latteral_control_tip = 1;
      
      //ROS_INFO("sta: %f -- speed: %f -- lat: %f",station_error,speed_error,latteral_offset_preview);

      double speed_tip = (15/3.6)*(15/3.6); // m/s

      int steer_gain = -200;

      if (speedSquare(real_pos_temp.velocity_x,real_pos_temp.velocity_y) < speed_tip) {
        steer_gain = -95;
      }
      
      //LKA

      //横向控制 PID  PSD
      //距离
      //速度
      pidp actual_parameter = {0};
      actual_parameter.latteral_offset_preview = latteral_offset_preview;
      actual_parameter.speed_error = speed_error;
      actual_parameter.station_error = station_error;
      actual_parameter.gps_seconds = real_pos_temp.gps_seconds;

      
      double control_angle;

      control_angle = PID_control(actual_parameter,real_pos_temp);

      ROS_INFO("Route ID :  %d  Position ID : %u",min_index,real_pos_temp.n_gps_sequence_num);
      ROS_INFO("Control angle: %lf\t now angle: %lf\t",control_angle*180/M_PI,real_pos_temp.heading*180/M_PI);
 
      //控制轴转向
      jointPub(joint_pub,control_angle);

      odomPub(g_gps_pub,odom_trans,odom,odom_broadcaster,real_pos_temp);

if (min_index > 1 && min_index < route_data.size())
{

      geometry_msgs::PointStamped psmsg;
      
      psmsg.header.stamp = ros::Time::now();
      psmsg.header.frame_id = "odom";

      psmsg.point.x = route_data[min_index+1].lat;
      psmsg.point.y = route_data[min_index+1].lon;
      psmsg.point.z = 0;

      pre_point_pub.publish(psmsg);


      geometry_msgs::PointStamped psmsg_gps;
      
      psmsg_gps.header.stamp = ros::Time::now();
      psmsg_gps.header.frame_id = "odom";

//      psmsg_gps.point.x = real_pos_temp.lat;
//      psmsg_gps.point.y = real_pos_temp.lon;

      psmsg_gps.point.x = pre_pos_temp.lat;
      psmsg_gps.point.y = pre_pos_temp.lon;

      psmsg_gps.point.z = 0;

      pre_point_pub_gps.publish(psmsg_gps);

}
      //纵向控制 转角
      if (latteral_control_tip == 1) {
        
      }
      

      //控制标记&控制命令

    }



    //发布控制命令

    ros::spinOnce();
    loop_rate.sleep();    
  }
  


  return 0;
}