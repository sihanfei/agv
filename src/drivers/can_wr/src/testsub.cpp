#include "testsub.h"
// #include <glog/logging.h>
// #include <event2/event.h>
#include <signal.h>

#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<unistd.h>
#include<iostream>
#include <sstream>

#include "std_msgs/UInt8MultiArray.h"
#include "status_msgs/NodeStatus.h"
#include "power_control_msgs/PowerControlCmd.h"
// #include "t_pool.h"
#include "threadpool.h"
#include <Eigen/Dense>

#include <list>
#include <queue>
#include <mutex>

#include <nav_msgs/GridCells.h>
#include <sys/time.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <time.h>
// #include <TLHelp.h>
#include <event.h>
#include "Apath.h"
#include "control_msgs/AGVStatus.h"
#include "perception_sensor_msgs/UltrasonicInfo.h"
#include "perception_sensor_msgs/ObjectList.h"
#include <math.h>
#include <vector>
#include <algorithm>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "k_means_1.h"

#define TIME_ZONE_H_D 8
#define TIME_ZONE_M_D 37
#define TIME_ZONE_S_D 30

using Eigen::MatrixXd;

ros::Publisher obstacle_cam_lidar_pub;

string getCmdResult(const string &strCmd)
{	
	char buf[10240] = {0};	
	FILE *pf = NULL;		
    cout << "strCmd:" << strCmd << endl;
	if( (pf = popen(strCmd.c_str(), "r")) == NULL )	
	{		
		return "";	
	} 	
	string strResult;	
	while(fgets(buf, sizeof buf, pf))	
	{		
		strResult += buf;	
	}		
	pclose(pf); 	
	unsigned int iSize =  strResult.size();	
	if(iSize > 0 && strResult[iSize - 1] == '\n')  // linux	
	{		
		strResult = strResult.substr(0, iSize - 1);	
	} 	
		return strResult;
}

void SplitString(const string &s,vector< string > &v_str,const string &c)
{
    string::size_type pos1,pos2;
    pos2 = s.find(c);
    pos1 = 0;
    string str;
    while(string::npos != pos2)
    {
        str = s.substr(pos1,pos2 - pos1);
        if(str != "")
        {
            v_str.push_back(str);
        }
        pos1 = pos2 + c.size();
        pos2 = s.find(c,pos1);
    }
    if(pos1 != s.length())
    {
        str = s.substr(pos1);
        if(str != "")
        {
            v_str.push_back(str);
        }
    }
}

string monthstr2num(string &str)
{
    if(!str.compare("Jan"))
    {
        return "01";
    }
    else if(!str.compare("Feb"))
    {
        return "02";
    }
    else if(!str.compare("Mar"))
    {
        return "03";
    }
    else if(!str.compare("Apr"))
    {
        return "04";
    }
    else if(!str.compare("May"))
    {
        return "05";
    }
    else if(!str.compare("Jun"))
    {
        return "06";
    }
    else if(!str.compare("Jul"))
    {
        return "07";
    }
    else if(!str.compare("Aug"))
    {
        return "08";
    }
    else if(!str.compare("Sept"))
    {
        return "09";
    }
    else if(!str.compare("Oct"))
    {
        return "10";
    }
    else if(!str.compare("Nov"))
    {
        return "11";
    }
    else if(!str.compare("Dec"))
    {
        return "12";
    }
    else
    {
        return str;
    }
}

string daystr2num(string &str)
{
    if(1 == str.size())
    {
        str = "0" + str;
    }
    return str;
}

logdata_output::logdata_output(string dir_path, string sensor_name, string device_name,uint32_t m6_rec_port)
                    : base_dir_path_(dir_path),sensor_name_(sensor_name),device_name_(device_name),channel_(m6_rec_port)
{
    log_file_created = false;
    log_droped       = false;
    g_main_min       = 0;

    logger_name_ = sensor_name_ + "-" + device_name_ + "-" + std::to_string(channel_);

    cout << logger_name_ << endl;

    SPDLOG_DEBUG(logger_name_);
}

logdata_output::~logdata_output()
{
    destroy_logger();
}

int logdata_output::my_mkdir(string muldir,mode_t mode)
{
    vector< string > v_str;
    SplitString(muldir,v_str,"/");
    
    SPDLOG_DEBUG("v_str.size():{}",v_str.size());
    
    int iRet;
    
    ostringstream temp_dir;
    temp_dir.fill('0');
    for(vector< string >::size_type i = 0;i != v_str.size();++i)
    {
        if(v_str[i] == "")
        {
            continue;
        }
        temp_dir << '/' << v_str[i];
        SPDLOG_DEBUG("temp_dir:{}",temp_dir.str().c_str());
        if(access(temp_dir.str().c_str(),0) != 0)
        {
            iRet = mkdir(temp_dir.str().c_str(),mode);
            if(iRet < 0)
            {
                return iRet;
            }
        }
    }
    return 0;
}

bool logdata_output::MinHasChanged()
{
    time_t raw_time;
    struct tm *tm_info;
    time(&raw_time);
    tm_info = localtime(&raw_time);
    
    if(tm_info->tm_min != g_main_min)
    {
        g_main_min = tm_info->tm_min;
        return true;
    }
    return false;
}

bool logdata_output::create_log_file(string dir_path)
{
    stringstream log_full_path;
    log_full_path << dir_path << log_file_name;

    if(access(dir_path.c_str(),F_OK) != 0)//log目录不存在
    {
        cout << "access != 0" << endl;
        if(my_mkdir(dir_path,0777) < 0)
        {
            cout << "my_mkdir < 0" << endl;
            SPDLOG_DEBUG("mkdir={} msg={}",dir_path.data(),strerror(errno));
        }
    }
    if(access(dir_path.c_str(),F_OK) != 0)
    {
        SPDLOG_DEBUG("dir_path:{} not find",dir_path.c_str());
        return false;
    }
    else
    {
        SPDLOG_DEBUG("dir_path:{}",dir_path.c_str());
        SPDLOG_DEBUG("log_full_path:{}",log_full_path.str().c_str());
    }

    cout << "log_full_path:" << log_full_path.str() << endl;

    my_logger = spdlog::basic_logger_mt< spdlog::async_factory >(logger_name_, log_full_path.str());

    my_logger->set_pattern("%Y-%m-%d-%H:%M:%S.%e,%n,%v");
    
    spdlog::flush_every(std::chrono::seconds(1));

    // my_logger = spdlog::get(logger_name_);

    return true;
}

void logdata_output::destroy_logger()
{
    SPDLOG_DEBUG("dextroy_logger");
    spdlog::drop(logger_name_);
}

void logdata_output::write_log(char *buff, int len)
{
    stringstream p2data;
    if(MinHasChanged() || !log_file_created)
    {
        if(log_droped)
        {
            destroy_logger();
        }
        time_t raw_time;
        struct tm * tm_info;
        
        time(&raw_time);
        tm_info = localtime(&raw_time);
        
        ostringstream time_pid_stream;
        time_pid_stream.fill(0);
        // time_pid_stream << device_name_ << '-' << channel_ << '-' << 1900 + tm_info->tm_year
        //                 <<  setw(2) << setfill('0') << 1 + tm_info->tm_mon << setw(2) << setfill('0')
        //                 << tm_info->tm_mday << '-'<< setw(2) << setfill('0') << tm_info->tm_hour 
        //                 << setw(2) << setfill('0') << tm_info->tm_min  << ".csv";
        time_pid_stream << sensor_name_ << '-' << device_name_ << '-' << channel_ << '-'
                        << tm_info->tm_min  << ".csv";

        cout << time_pid_stream.str() << endl;

        ostringstream dir_path_stream;
        dir_path_stream.fill(0);
        dir_path_stream << base_dir_path_ << '/' << sensor_name_ << '/' << 1900 + tm_info->tm_year
                        << setw(2) << setfill('0') << 1 + tm_info->tm_mon << setw(2) << setfill('0')
                        << tm_info->tm_mday << '-'<< tm_info->tm_hour << '/';
                    
        log_file_name    = time_pid_stream.str();
        cout << "log_file_name:" << log_file_name << endl;
        log_droped       = create_log_file(dir_path_stream.str());
        log_file_created = true;
    }
    if(log_droped)
    {
        // std::vector< unsigned char > uvchar;
        std::vector< char > uvchar;
        for(size_t i = 0;i < len;++i)
        {
            uvchar.push_back(buff[i]);
        }
            
            // my_logger = spdlog::get(logger_name_);
        // my_logger -> info("{:Xspn}",spdlog::to_hex(uvchar));
        // cout << buff[0] << "," << buff[1] << endl;
        // ROS_INFO("x:%f,y:%f",buff[0],buff[1]);
        for(size_t i = 0;i < len;++i)
        {
            p2data << buff[i];
        }
        // p2data << to_string(buff[0]) << "," << to_string(buff[1]);
        cout << p2data.str() << endl;
        my_logger -> info("{}",p2data.str());
    }
}

double a = 0;

logdata_output can_log("/home/hx/work/log","p2","2",3);

void test_subcallback(const location_sensor_msgs::IMUAndGNSSInfoConstPtr& msg)
{
    // a = msg->pose.x;
    stringstream data;
    char datap2[100];
    double pose[2] = {};
    double pose1,pose2;
    pose[0] = msg->pose.x;
    pose[1] = msg->pose.y;
    pose1 = msg->pose.x;
    pose2 = msg->pose.y;
    sprintf(datap2,"%0.7lf,%0.7lf,",pose1,pose2);
    // cout << datap2 << endl;
    // ROS_INFO("X:%lf,Y:%lf",pose[0],pose1);
    can_log.write_log(&datap2[0],sizeof(datap2));
    ROS_INFO("pose.x:%0.9lf,pose.x:%0.9lf",msg->pose.x,msg->pose.y);
    // double tm_ = ros::Time::now().toSec();
    // ROS_INFO("sec:%d,msec:%0.9lf,dt:%0.9lf",msg->sec,msg->msec);
    // a = msg->msec;
}

// #define A 16777216

void aaa(string &str)
{
    string s = str;
    if(str == "p2")
    {
        ROS_INFO("is p2");
    }
    else
    {
        ROS_INFO("not p2");
    }
}

double sum_x = 0.0;
double ave_x = 0.0;
double max_x = 0.0;
double min_x = 0.0;
double sum_y = 0.0;
double ave_y = 0.0;
double max_y = 0.0;
double min_y = 0.0;
double sum_z = 0.0;
double ave_z = 0.0;
double max_z = 0.0;
double min_z = 0.0;
long long int cnt = 0;
bool frist = true;

//void power_control_callback(const location_sensor_msgs::IMUAndGNSSInfo::ConstPtr& msg)
void power_control_callback(const perception_sensor_msgs::UltrasonicInfo::ConstPtr& msg,int& ar)
{
/*
    if(frist)
    {
        min_x = msg->accelgyro.linear.x;
        min_y = msg->accelgyro.linear.y;
        min_z = msg->accelgyro.linear.z;
        frist = false;
    }
    ++cnt;
    // ROS_INFO_STREAM(msg->state_num);
    max_x = max_x>msg->accelgyro.linear.x? max_x:msg->accelgyro.linear.x;
    min_x = min_x<msg->accelgyro.linear.x? min_x:msg->accelgyro.linear.x;
    sum_x += msg->accelgyro.linear.x;
    ave_x = sum_x/cnt;
    max_y = max_y>msg->accelgyro.linear.y? max_y:msg->accelgyro.linear.y;
    min_y = min_y<msg->accelgyro.linear.y? min_y:msg->accelgyro.linear.y;
    sum_y += msg->accelgyro.linear.y;
    ave_y = sum_y/cnt;
    max_z = max_z>msg->accelgyro.linear.z? max_z:msg->accelgyro.linear.z;
    min_z = min_z<msg->accelgyro.linear.z? min_z:msg->accelgyro.linear.z;
    sum_z += msg->accelgyro.linear.z;
    ave_z = sum_z/cnt;
    ROS_INFO_STREAM("max_x:" << max_x << ",min_x:" << min_x << ",ave_x:" << ave_x << ",max_y:" << max_y << ",min_y:" << min_y << ",ave_y:" << ave_y << ",max_z:" << max_z << ",min_z:" << min_z << ",ave_z:" << ave_z);
*/
	// for(int i = 0;i < 32;++i)
	// {
	// 	if(msg->ult_obstacle[i].id == ar)
	// 	{
	// 		ROS_INFO_STREAM("DISTANCE:" << msg->ult_obstacle[i].distance);
	// 	}
	// }
    for(int i = 0;i < msg->ult_obstacle.size();++i)
    {
        ROS_INFO("id:%u,distance:%f",msg->ult_obstacle[i].id,msg->ult_obstacle[i].distance);
    }
}

string num2str(int i)
{
        char ss[10];
		sprintf(ss,"%02d",i);
        return ss;
}

void time_zone_change(string &gps_date,string &gps_time,string &date,string &time)
{
  stringstream ss_date;
  stringstream ss_time;
  int hour = 0;
  int min  = 0;
  int sec  = 0;
  int year = 0;
  int mout = 0;
  int day  = 0;
  char s_time[10];
  char s_date[10];
  int day_max[12] = {31,29,31,30,31,30,31,31,30,31,30,31};
  ss_time << gps_time.substr(0,2);
  ss_time >> hour;
  ss_time.clear();
  ss_time << gps_time.substr(2,2);
  ss_time >> min;
  ss_time.clear();
  ss_time << gps_time.substr(4,2);
  ss_time >> sec;
  ss_time.clear();
  ss_date << gps_date.substr(4,2);
  ss_date >> year;
  year += 2000;
  ss_date.clear();
  ss_date << gps_date.substr(2,2);
  ss_date >> mout;
  ss_date.clear();
  ss_date << gps_date.substr(0,2);
  ss_date >> day;
  ss_date.clear();
  if((0 == year%4 && 0 != year%100) || 0 == year%400)
  {
    day_max[1] = 28;
  }
  if(sec + TIME_ZONE_S_D >= 60)
  {
    if(min + 1 >= 60)
    {
      if(hour + 1 >= 24)
      {
        if(day + 1 > day_max[mout - 1])
        {
          if(mout + 1 > 12)
          {
            year += 1;
            mout = 1;
          }
          else
          {
            mout += 1;
          }
          day = 1;
        }
        else
        {
          day += 1;
        }
        hour = 0;
      }
      else
      {
        hour += 1;
      }
      min = 0;
    }
    else
    {
      min += 1;
    }
    sec = (sec + TIME_ZONE_S_D) - 60;
  }
  else
  {
    sec += TIME_ZONE_S_D;
  }
  if(min + TIME_ZONE_M_D >= 60)
  {
    if(hour + 1 >= 24)
    {
      if(day + 1 > day_max[mout - 1])
      {
        if(mout + 1 > 12)
        {
          year += 1;
          mout = 1;
        }
        else
        {
          mout += 1;
        }
        day = 1;
      }
      else
      {
        day += 1;
      }
      hour = 0;
    }
    else
    {
      hour += 1;
    }
    min = (min + TIME_ZONE_M_D) - 60;
  }
  else
  {
    min += TIME_ZONE_M_D;
  }
  if((hour + TIME_ZONE_H_D) >= 24)
  {
    if(day + 1 > day_max[mout - 1])
    {
      if(mout + 1 > 12)
      {
        year += 1;
        mout = 1;
      }
      else
      {
        mout += 1;
      }
      day = 1;
    }
    else
    {
      day += 1;
    }
    hour = (hour + TIME_ZONE_H_D) - 24;
  }
  else
  {
    hour += TIME_ZONE_H_D;
  }
  sprintf(s_time,"%02d:%02d:%02d",hour,min,sec);
  sprintf(s_date,"%02d-%02d-%02d",year,mout,day);
  time = s_time;
  date = s_date;
}



// void signal_cb(evutil_socket_t fd, short event, void *arg)
// {
//     struct event *signal = (struct event*)arg;
//     printf("signal_cb: got signal %d\n",event_get_signal(signal));
// }

typedef void(*lpFunc)(void *, char *);

// void GetCallBack(void * lpVoid,lpFunc callback)
// {
//     callback(lpVoid,"test");
// }


void fun1(int slp)
{
    while(ros::ok())
    {
        // printf("  hello, fun1 !  %d\n" ,std::this_thread::get_id());
        if (slp>0) {
            // printf(" ======= fun1 sleep %d  =========  %d\n",slp, std::this_thread::get_id());
            std::this_thread::sleep_for(std::chrono::milliseconds(slp));
        }
        sleep(2);
    }
    // printf("  hello, fun1 !  %d\n" ,std::this_thread::get_id());
    // if (slp>0) {
    //     printf(" ======= fun1 sleep %d  =========  %d\n",slp, std::this_thread::get_id());
    //     std::this_thread::sleep_for(std::chrono::milliseconds(slp));
    // }
}

struct gfun {
    int operator()(int n) {
        // printf("%d  hello, gfun !  %d\n" ,n, std::this_thread::get_id() );
        return 42;
    }
};

class A { 
public:
    static int Afun(int n = 0) {   //函数必须是 static 的才能直接使用线程池
        std::cout << n << "  hello, Afun !  " << std::this_thread::get_id() << std::endl;
        return n;
    }

    static std::string Bfun(int n, std::string str, char c) {
        std::cout << n << "  hello, Bfun !  "<< str.c_str() <<"  " << (int)c <<"  " << std::this_thread::get_id() << std::endl;
        return str;
    }
};

using namespace google;

#define PI 3.141592

double angle_to_radian(double degree, double min, double second)
{
	double flag = (degree < 0)? -1.0 : 1.0;			//判断正负
    if(degree<0)
    {
        degree = degree * (-1.0);
    }
	double angle = degree + min/60 + second/3600;
	double result =flag * (angle * PI)/180;
	return result;
	//cout<<result<<endl;
}

class que
{
    private:
        std::mutex mtx_test;
        static std::queue<string> q_test;
    public:
        bool q_sensor_empty()
        {
            bool empty = false;
            mtx_test.lock();
            empty = q_test.empty();
            mtx_test.unlock();
            return empty;
        };
        string q_sensor_front()
        {
            string sensor_type = "";
            if(!q_sensor_empty())
            {
                mtx_test.lock();
                sensor_type = q_test.front();
                q_test.pop();
                mtx_test.unlock();
            }
            return sensor_type;
        };
        uint16_t get_q_sensor_size()
        {
            return uint16_t(q_test.size());
        };
        void q_sensor_push(string &str)
        {
            mtx_test.lock();
            q_test.push(str);
            mtx_test.unlock();
            ROS_INFO_STREAM("q_sensor_type:" << q_test.size());
        };    
        void t1()
        {
            string str = "t1";
            while(ros::ok())
            {
                ROS_INFO("t1 push");
                q_sensor_push(str);
                usleep(1000 * 10);
            }
        };

        void t2()
        {
            string str = "t2";
            while(ros::ok())
            {
                ROS_INFO("t2 push");
                q_sensor_push(str);
                usleep(1000 * 100);
            }
        };

        void t3()
        {
            string str = "t3";
            while(ros::ok())
            {
                ROS_INFO("t3 push");
                q_sensor_push(str);
                usleep(1000 * 100);
            }
        };

        void t4()
        {
            string str = "t4";
            while(ros::ok())
            {
                ROS_INFO("t4 push");
                q_sensor_push(str);
                usleep(1000 * 200);
            }
        };

        void t5()
        {
            string str_sensor_type = "";
            ros::Rate rate_loop(100);
            list<string> list_sensor;
            while(ros::ok())
            {
                if(!q_sensor_empty())
                {
                    printf("mtx_test.lock() before\r\n");
                    mtx_test.lock();
                    printf("mtx_test.lock() end\r\n");
                    for(uint8_t j = 0;j < get_q_sensor_size();++j)
                    {
                        printf("1\r\n");
                        list_sensor.push_back(q_test.front());
                        printf("2\r\n");
                        q_test.pop();
                        printf("3\r\n");
                    }
                    printf("mtx_test.unlock() before\r\n");
                    mtx_test.unlock();
                    printf("mtx_test.unlock() end\r\n");
                }
                printf("q_sensor_empty end\r\n");
                if(!list_sensor.empty())
                {
                    list<string>::iterator iter = list_sensor.begin();
                    for(;iter != list_sensor.end();++iter)
                    {
                        str_sensor_type = *iter;
                        ROS_INFO_STREAM("str:" << str_sensor_type);
                    }
                    list_sensor.clear();
                }
                printf("list_sensor.empty end\r\n");
                rate_loop.sleep();
            }
        };
};


std::queue<string> que::q_test;


struct odometryConf
{
  float ID;
  float gps_seconds;
  float lon;
  float lat;
  float height;
  float velocity;
  float angle_x;
  float volume_x;
  float volume_y;
  float volume_z;
};

odometryConf obstacle_data;
void recvObstacleCallback(const nav_msgs::OdometryConstPtr &msg)
{
  obstacle_data.gps_seconds = msg->header.stamp.nsec;
  obstacle_data.ID          = msg->pose.pose.orientation.w;
  obstacle_data.lat         = msg->pose.pose.position.x;
  obstacle_data.lon         = msg->pose.pose.position.y;
  obstacle_data.height      = msg->pose.pose.position.z;
  obstacle_data.velocity    = msg->twist.twist.angular.x;
  //  obstacle_data.angle_x     = -(msg->pose.pose.orientation.x - 90); // Y轴方向角度
  obstacle_data.angle_x  = msg->pose.pose.orientation.x; // gh坐标
  obstacle_data.volume_x = msg->twist.twist.linear.x;    //长
  obstacle_data.volume_y = msg->twist.twist.linear.y;    //宽
  obstacle_data.volume_z = msg->twist.twist.linear.z;    //高
//   new_odometry_send_tip  = 1;
  cout << "id:" << obstacle_data.ID << endl;
  visualization_msgs::MarkerArray maker_temp;
  maker_temp.markers.clear();
  visualization_msgs::Marker points_rviz;
  points_rviz.header.frame_id   = "/odom";
  points_rviz.header.stamp = ros::Time::now();
  points_rviz.ns           = "/per/lidar_obstacle_fusion";
  points_rviz.action       = visualization_msgs::Marker::ADD;
  points_rviz.pose.orientation.w = 1.0;
  points_rviz.id           = 0;
  points_rviz.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  points_rviz.scale.z = 0.2;
  points_rviz.color.b = 0;
  points_rviz.color.g = 0;
  points_rviz.color.r = 1;
  points_rviz.color.a = 1;

  geometry_msgs::Pose pose;
  pose.position.x =  obstacle_data.lat;
  pose.position.y =  obstacle_data.lon;
  pose.position.z =0;
  ostringstream str;
  str << "id:" << obstacle_data.ID << ",v:" << obstacle_data.velocity;
  points_rviz.text=str.str();
  points_rviz.pose=pose;
  maker_temp.markers.push_back(points_rviz);
  obstacle_cam_lidar_pub.publish(maker_temp);
}

struct utc
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
};

void UTC2GPS(int year, int month, int day, int hour, int minute, double second, int *weekNo, double *secondOfweek)
{ 
/*****协调世界时转换为GPS的周秒表示*****///输入时间应为协调世界时，即当地时间-8，返回时间为GPS周和周秒
int DayofYear = 0;
int DayofMonth = 0;


for (int i = 1980; i < year; i++)  //从1980年到当前年的上一年经过的天数
{
    if ((i % 4 == 0 && i % 100 != 0) || i % 400 == 0)
        DayofYear += 366;
    else
        DayofYear += 365;
}
for (int i = 1; i < month; i++)   //从一月到当前月的上一月经历的天数
{
    if (i == 1 || i == 3 || i == 5 || i == 7 || i == 8 || i == 10 || i ==12)
        DayofMonth += 31;
    else if (i == 4 || i == 6 || i == 9 || i == 11)
    DayofMonth += 30;
    else
    {
        if ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0)
            DayofMonth += 29;
        else
            DayofMonth += 28;
    }
}
int Day;
Day = DayofMonth + day + DayofYear-6;
*weekNo = Day/7;
*secondOfweek = Day % 7 * 86400 + hour * 3600 + minute * 60 + second+18;//18表示跳秒

}

void GPS2TUC(int weekNo,double secondOfweek)
{
    
}

class Test1
{
public:
    Test1(int n)
    {
        num=n;
    }//普通构造函数
private:
    int num;
};

class Test2
{
public:
    explicit Test2(int n)
    {
        num=n;
    }//explicit(显式)构造函数
private:
    int num;
};

void OnTime(int sock, short event, void *arg)
{
    cout << "Game Over!" << endl;

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    // 重新添加定时事件（定时事件触发后默认自动删除）
    // event_add((struct event*)arg, &tv);
}

struct Event{
    int type;
    void* data;
    struct event  ev;
};

int arr[10][10] = {
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 1, 3, 0, 0, 0 },
    { 0, 0, 2, 0, 0, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } };


#pragma comment(lib,"ws2_32.lib")

#define pi 3.1415926

struct point
{
    int index;
    float x;
    float y;
    float safe_distance;
};

struct linepoint
{
    float x;
    float y;
};

struct linetype
{
    vector<linepoint> main_line;
    linepoint other_line;
};

void compute_Correlation(vector<linepoint> &points, double &Correlation)
{
    double x_sum = 0,y_sum = 0,xy_sum = 0;
    double x_mean = 0,y_mean = 0;
    double E_XY = 0;
    double x_thea = 0,y_thea = 0;
    double x_variance = 0,y_variance = 0;
    double Cov_XY = 0;
    for(vector<linepoint>::iterator ite_points = points.begin();ite_points != points.end();++ite_points)
    {
        x_sum += ite_points->x;
        y_sum += ite_points->y;
        xy_sum += ite_points->x*ite_points->y;
    }
    x_mean = x_sum / points.size();
    y_mean = y_sum / points.size();
    E_XY = xy_sum / points.size();
    Cov_XY = E_XY - (x_mean * y_mean);
    for(vector<linepoint>::iterator ite_points = points.begin();ite_points != points.end();++ite_points)
    {
        x_thea +=  pow(ite_points->x - x_mean,2);
        y_thea +=  pow(ite_points->y - y_mean,2);
    }
    x_variance = x_thea / points.size();
    y_variance = y_thea / points.size();
    Correlation = fabs(Cov_XY / sqrt(x_variance * y_variance));
}

// void find_line(vector<linepoint> &points,vector<vector<linepoint>> &v_line)
// {
//     double Correlation = 0;
//     double Correlation_ = 0;
//     vector<linetype> v_linetype_;
//     linetype linetype_;
//     linetype_.main_line = points;
//     v_linetype_.push_back(linetype_);
//     vector<vector<linetype>> v_line_;
//     vector<linepoint> vec_tmp;
//     if(points.size() >= 5)
//     {
//         compute_Correlation(points,Correlation_);bool array_recv[9] = {0,0,0,0,0,0,0,0,0};
//         for(int i = 0;i < points.size();++i)
//         {
//             v_line_.push_back(v_linetype_);
//             v_line_.at(i).main_line.erase(v_line_.at(i).main_line.begin() + i);
//         }
//         // for(vector<linepoint>::iterator ite_points = points.begin();ite_points != points.end();++ite_points)
//         // {
//         //     points_.erase(ite_points);
//         //     v_line_++ = points_;
//         //     // compute_Correlation(points_,Correlation);
//         //     // if(Correlation > Correlation_)
//         //     // {
//         //     //     v_line[0] = points_;
//         //     //     v_line[1].push_back(ite_points);
//         //     // }
//         // }
//     }
    
//     for(vector<vector<linepoint>>::iterator ite_v_line_ = v_line_.begin();ite_v_line_ != v_line_.end();++ite_v_line_)
//     {
//         vec_tmp = *ite_v_line_;
//         for(vector<linepoint>::iterator ite_line_ = vec_tmp.begin();ite_line_ != vec_tmp.end();++ite_line_)
//         {
//             ROS_INFO_STREAM("line_x:" << ite_line_->x << "line_y:" << ite_line_->y);
//         }
//         ROS_INFO("~~~~~~~~~~~~~~~~~~~~~~~~");
//     }
// }
double compute_Covariance(const vector<linepoint> &points)
{
    double x_sum = 0,y_sum = 0,xy_sum = 0;
    double x_mean = 0,y_mean = 0;
    double E_XY = 0;
    double Cov_XY = 0;
    int size = points.size();
    for(int i = 0;i < size;++i)
    {
        x_sum += points[i].x/1000.0;
        y_sum += points[i].y/1000.0;
        xy_sum += points[i].x/1000.0*points[i].y/1000.0;
    }
    x_mean = x_sum / size;
    y_mean = y_sum / size;
    E_XY = xy_sum / size;
    Cov_XY = E_XY - (x_mean * y_mean);
    if(Cov_XY > 1.4325)
    {
        return Cov_XY;
    }
    else
    {
        return y_mean;
    }
}

double compute_correlation(const vector<linepoint> &points)
{
    double correlation_ = 0;
    double x_sum = 0,y_sum = 0,xy_sum = 0;
    double x_mean = 0,y_mean = 0;
    double x_thea = 0,y_thea = 0;
    double x_variance = 0,y_variance = 0;
    double E_XY = 0;
    double Cov_XY = 0;
    int size = points.size();

    for(int i = 0;i < size;++i)
    {
        x_sum += points[i].x/1000.0;
        y_sum += points[i].y/1000.0;
        xy_sum += points[i].x/1000.0*points[i].y/1000.0;
    }
    x_mean = x_sum / size;
    y_mean = y_sum / size;
    E_XY = xy_sum / size;
    Cov_XY = E_XY - (x_mean * y_mean);
    ROS_INFO_STREAM("x_mean:" << x_mean << ",y_mean:" << y_mean << ",E_XY:" << E_XY << ",Cov_XY:" << Cov_XY);
    if(fabs(Cov_XY) > 0.05)
    {
        for(int i = 0;i < size;++i)
        {
            x_thea +=  pow(points[i].x/1000.0 - x_mean,2);
            y_thea +=  pow(points[i].y/1000.0 - y_mean,2);
        }
        x_variance = x_thea / size;
        y_variance = y_thea / size;
        correlation_ = fabs(Cov_XY / sqrt(x_variance * y_variance));
        ROS_INFO_STREAM("x_thea:" << x_thea << ",y_thea:" << y_thea << ",x_variance:" << x_variance << ",y_thea:" << y_variance);
    }
    else
    {
        correlation_ = 1;
    }

    return correlation_;
}

#define THRESHOLD_CORRELATION 0.9

void show_vector(vector<linepoint> &points)
{
    ROS_INFO("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    for(vector<linepoint>::iterator ite_points = points.begin();ite_points != points.end();++ite_points)
    {
        ROS_INFO("x:%f,y:%f",ite_points->x,ite_points->y);
    }
    ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
}

void compute_thea(vector<linepoint> &points)
{
    double x_sum = 0,y_sum = 0;
    double x_mean = 0,y_mean = 0;
    double x_thea = 0,y_thea = 0;
    double x_variance = 0,y_variance = 0;
    int size = points.size();
    for(int i = 0;i < size;++i)
    {
        x_sum += points[i].x;
        y_sum += points[i].y;
    }
    x_mean = x_sum/size;
    y_mean = y_sum/size;
    for(int i = 0;i < size;++i)
    {
        x_thea +=  pow(points[i].x - x_mean,2);
        y_thea +=  pow(points[i].y - y_mean,2);
    }
    x_variance = x_thea / size;
    y_variance = y_thea / size;
}

void classify_point(vector<linepoint> &points,vector<linepoint> &other_points,double &Correlation)
{
    vector<linepoint> points_ = points;
    vector<linepoint> main_points;
    // vector<linepoint> other_points;
    main_points.clear();
    double main_Correlation = compute_correlation(points_);
    double main_Correlation_temp = 0;
    int other_id = -1;
    // ROS_INFO_STREAM("correlation:" << main_Correlation);
    if(main_Correlation < THRESHOLD_CORRELATION)
    {
        ROS_INFO_STREAM("correlation:" << main_Correlation);
        show_vector(points);
        for(int i = 0;i < points.size();++i)
        {
            // show_vector(points);
            main_points = points;
            // show_vector(main_points);
            main_points.erase(main_points.begin() + i);
            // show_vector(main_points);
            main_Correlation_temp = compute_correlation(main_points);
            ROS_INFO_STREAM("%%%%%%%%%%%%%%%%%%%erase correlation:" << main_Correlation_temp);
            if(main_Correlation_temp > main_Correlation)
            {
                // ROS_INFO_STREAM("i:" << i << ",other_id:" << other_id);
                other_points.push_back(main_points[i]);
                // points.erase(main_points.begin() + i);
                other_id = i;
                main_Correlation = main_Correlation_temp;
            }
        }
        main_points.clear();
        for(int i = 0;i < points.size();++i)
        {
            if(i != other_id)
            {
                main_points.push_back(points[i]);
            }
            else
            {
                other_points.push_back(points[i]);
                ROS_INFO_STREAM("y:" << points[i].y);
            }
        }
        show_vector(main_points);
        main_Correlation = compute_correlation(main_points);
        ROS_INFO_STREAM("finally correlation:" << main_Correlation);
        points.clear();
        points = main_points;
        // for(vector<linepoint>::iterator ite_points = points.begin();ite_points != points.end();++ite_points)
        // {
        //     main_points.clear();
            
        // }
    }
    else
    {
        ROS_INFO_STREAM("111111correlation:" << main_Correlation);
    }
    Correlation = main_Correlation;
}

void classify_point_(vector<linepoint> &points,vector<linepoint> &other_points,double &Correlation)
{
    vector<linepoint> points_ = points;
    vector<linepoint> main_points;
    // vector<linepoint> other_points;
    main_points.clear();
    double main_Correlation = compute_Covariance(points_);
    double main_Correlation_temp = 0;
    int other_id = -1;
    // ROS_INFO_STREAM("correlation:" << main_Correlation);
    if(main_Correlation < THRESHOLD_CORRELATION)
    {
        ROS_INFO_STREAM("correlation:" << main_Correlation);
        show_vector(points);
        for(int i = 0;i < points.size();++i)
        {
            // show_vector(points);
            main_points = points;
            // show_vector(main_points);
            main_points.erase(main_points.begin() + i);
            // show_vector(main_points);
            main_Correlation_temp = compute_Covariance(main_points);
            ROS_INFO_STREAM("%%%%%%%%%%%%%%%%%%%erase correlation:" << main_Correlation_temp);
            if(main_Correlation_temp > main_Correlation)
            {
                // ROS_INFO_STREAM("i:" << i << ",other_id:" << other_id);
                other_points.push_back(main_points[i]);
                // points.erase(main_points.begin() + i);
                other_id = i;
                main_Correlation = main_Correlation_temp;
            }
        }
        main_points.clear();
        for(int i = 0;i < points.size();++i)
        {
            if(i != other_id)
            {
                main_points.push_back(points[i]);
            }
            else
            {
                other_points.push_back(points[i]);
                ROS_INFO_STREAM("y:" << points[i].y);
            }
        }
        show_vector(main_points);
        main_Correlation = compute_correlation(main_points);
        ROS_INFO_STREAM("finally correlation:" << main_Correlation);
        points.clear();
        points = main_points;
        // for(vector<linepoint>::iterator ite_points = points.begin();ite_points != points.end();++ite_points)
        // {
        //     main_points.clear();
            
        // }
    }
    else
    {
        ROS_INFO_STREAM("111111correlation:" << main_Correlation);
    }
    Correlation = main_Correlation;
}

void lineFit_(const vector<linepoint> &points, double &k, double &b, double &Correlation)
{
    vector<linepoint> main_points;
    main_points.clear();
    double main_Correlation = 0;

    double x_sum = 0,y_sum = 0,xy_sum = 0,xx_sum = 0;
    double x_mean = 0,y_mean = 0;
    double x_thea = 0,y_thea = 0;
    double x_variance = 0,y_variance = 0;
    int size = points.size();

    double E_XY = 0;
    double Cov_XY = 0;
    // for(vector<linepoint>::iterator ite_points = points.begin();ite_points != points.end();++i
    for(int i = 0;i < size;++i)
    {
        x_sum += points[i].x;
        xx_sum += pow(points[i].x,2);
        y_sum += points[i].y;
        xy_sum += points[i].x*points[i].y;
    }
    x_mean = x_sum / size;
    y_mean = y_sum / size;
    E_XY = xy_sum / size;
    Cov_XY = E_XY - (x_mean * y_mean);
    for(int i = 0;i < size;++i)
    {
        x_thea +=  pow(points[i].x - x_mean,2);
        y_thea +=  pow(points[i].y - y_mean,2);
    }
    x_variance = x_thea / size;
    y_variance = y_thea / size;
    // ROS_INFO("x_mean:%lf,y_mean:%lf,x_variance:%lf,y_variance:%lf",x_mean,y_mean,x_variance,y_variance);
    k = (xy_sum - (x_sum * y_sum) / (size)) / (xx_sum  - (pow(x_sum,2)) / (size));
    b = (y_sum - k * x_sum) / (size);
    Correlation = fabs(Cov_XY / sqrt(x_variance * y_variance));
    // ROS_INFO_STREAM("Correlation:" << Correlation);
}

bool lineFit(const vector<linepoint> &points, double &a, double &b, double &c)
{
     int size = points.size();
     if(size < 2)
     {
         a = 0;
         b = 0;
         c = 0;
         return false;
     }
     double x_mean = 0;
     double y_mean = 0;
     for(int i = 0; i < size; i++)
     {
         x_mean += points[i].x;
         y_mean += points[i].y;
     }
     x_mean /= size;
     y_mean /= size; //至此，计算出了 x y 的均值
 
     double Dxx = 0, Dxy = 0, Dyy = 0;
 
     for(int i = 0; i < size; i++)
     {
         Dxx += (points[i].x - x_mean) * (points[i].x - x_mean);
         Dxy += (points[i].x - x_mean) * (points[i].y - y_mean);
         Dyy += (points[i].y - y_mean) * (points[i].y - y_mean);
     }
     double lambda = ( (Dxx + Dyy) - sqrt( (Dxx - Dyy) * (Dxx - Dyy) + 4 * Dxy * Dxy) ) / 2.0;
     double den = sqrt( Dxy * Dxy + (lambda - Dxx) * (lambda - Dxx) );
     a = Dxy / den;
     b = (lambda - Dxx) / den;
     c = - a * x_mean - b * y_mean;
     return true;
}

struct thea_y
{
    int id_1;
    int id_2;
    double thea;
};

struct thea_y_info
{
    double thea;
    int count;
    vector<thea_y> thea_y_;
};

struct line_clustering
{
    double thea;
    vector<int> v_id;
};

int cmpn1(thea_y_info a,thea_y_info b)
{
    return a.count>b.count;
}

void Clustering(const vector<linepoint> &points)
{
    vector<thea_y> v_thea_y;
    vector<thea_y_info> v_thea_y_info;
    vector<thea_y_info>::iterator ite_v_thea_y_info;

    line_clustering line_clustering_;
    vector<line_clustering> v_line_clustering;
    line_clustering line_clustering_finally;
    vector<line_clustering> v_line_clustering_finally;
    bool b_line_clustering_finally = false;

    thea_y_info thea_y_info_;
    thea_y thea_y_;
    int sum_count = 0;
    int i = 0;
    // int max_count = 0;
    double type_thea_y = 0.0;
    int size = points.size();
    bool flag = false;
    for(int i = 0;i < size;++i)
    {
        for(int j = i+1;j < size;++j)
        {
            // thea_y_.thea = points[i].y - points[j].y;
            // thea_y_.thea = sqrt(pow(points[i].y - points[j].y,2) + pow(points[i].x - points[j].x,2));
            thea_y_.thea = (points[i].y - points[j].y) / (points[i].x - points[j].x);
            thea_y_.id_1 = i;
            thea_y_.id_2 = j;
            if(v_thea_y_info.empty())
            {
                thea_y_info_.thea_y_.clear();
                thea_y_info_.thea = thea_y_.thea;
                thea_y_info_.count = 1;
                thea_y_info_.thea_y_.push_back(thea_y_);
                v_thea_y_info.push_back(thea_y_info_);
                ROS_INFO_STREAM("v_thea_y_info.empty(),thea:" << thea_y_info_.thea << ",count:" << thea_y_info_.count << ",id1:" << thea_y_.id_1 << ",id2:" << thea_y_.id_2 << ",size:" << thea_y_info_.thea_y_.size());
            }
            else
            {
                for(ite_v_thea_y_info = v_thea_y_info.begin();ite_v_thea_y_info != v_thea_y_info.end();++ite_v_thea_y_info)
                {
                    if(thea_y_.thea == ite_v_thea_y_info->thea)
                    {
                        ite_v_thea_y_info->count += 1;
                        ite_v_thea_y_info->thea_y_.push_back(thea_y_);
                        flag = true;
                        ROS_INFO_STREAM("xiangtong,thea:" << ite_v_thea_y_info->thea << ",count:" << ite_v_thea_y_info->count << ",id1:" << thea_y_.id_1 << ",id2:" << thea_y_.id_2 << ",size:" << ite_v_thea_y_info->thea_y_.size());
                        break;
                    }
                    else
                    {
                        flag = false;
                    }
                }
                if(!flag)
                {
                    thea_y_info_.thea_y_.clear();
                    thea_y_info_.thea = thea_y_.thea;
                    thea_y_info_.count = 1;
                    thea_y_info_.thea_y_.push_back(thea_y_);
                    v_thea_y_info.push_back(thea_y_info_);
                    ROS_INFO_STREAM("ite_v_thea_y_info == v_thea_y_info.end(),thea:" << thea_y_info_.thea << ",count:" << thea_y_info_.count << ",id1:" << thea_y_.id_1 << ",id2:" << thea_y_.id_2 << ",size:" << thea_y_info_.thea_y_.size());
                }
            }
            v_thea_y.push_back(thea_y_);
        }
    }
    // for(vector<thea_y>::iterator ite_v_thea_y = v_thea_y.begin();ite_v_thea_y != v_thea_y.end();++ite_v_thea_y)
    // {
    //     ROS_INFO_STREAM("id_1:" << ite_v_thea_y->id_1 << ",id_2:" << ite_v_thea_y->id_2 << ",thea:" << ite_v_thea_y->thea);
    // }
    sort(v_thea_y_info.begin(),v_thea_y_info.end(),cmpn1);
    for(ite_v_thea_y_info = v_thea_y_info.begin();ite_v_thea_y_info != v_thea_y_info.end();++ite_v_thea_y_info)
    {
        // if(thea_y_info_.count < ite_v_thea_y_info->count)
        // {
        //     thea_y_info_.count = ite_v_thea_y_info->count;
        //     thea_y_info_.thea = ite_v_thea_y_info->thea;
        // }
        // max_count = max_count > ite_v_thea_y_info->count ? max_count:ite_v_thea_y_info->count;
        ROS_INFO_STREAM("thea:" << ite_v_thea_y_info->thea << ",count:" << ite_v_thea_y_info->count);
    }
    vector<vector<linepoint>> v_linepoint_clustering;
    vector<int> v_id;
    for(ite_v_thea_y_info = v_thea_y_info.begin();ite_v_thea_y_info != v_thea_y_info.end();++ite_v_thea_y_info)
    {
        if(ite_v_thea_y_info->thea_y_.size() > 2)
        {
            cout << "max_count:" << v_thea_y_info[i].count << ",type_thea_y:" << v_thea_y_info[i].thea <<  endl;
            cout << "size:" << v_thea_y_info[i].thea_y_.size() << endl;
            for(vector<thea_y>::iterator ite_thea_y_ = v_thea_y_info[i].thea_y_.begin();ite_thea_y_ != v_thea_y_info[i].thea_y_.end();++ite_thea_y_)
            {
                cout << "id1:" << ite_thea_y_->id_1 << ",id2:" << ite_thea_y_->id_2 << endl;
                line_clustering_.thea = ite_thea_y_->thea;
                line_clustering_.v_id.push_back(ite_thea_y_->id_1);
                line_clustering_.v_id.push_back(ite_thea_y_->id_2);
                // set<int> s(line_clustering_.v_id.begin(),line_clustering_.v_id.end());
                // line_clustering_.v_id.assign(s.begin(),s.end());
                v_id.push_back(ite_thea_y_->id_1);
                v_id.push_back(ite_thea_y_->id_2);
            }
            cout << "22type_thea_y:" << line_clustering_.thea <<  endl;
            cout << "22size:" << line_clustering_.v_id.size() << endl;
            v_line_clustering.push_back(line_clustering_);
            line_clustering_.v_id.clear();
        }
        ++i;
    }
    // while(sum_count < v_thea_y_info.size())
    // {
    //     sum_count += v_thea_y_info[i].count;
    //     if(v_thea_y_info[i].thea_y_.size() > 2)
    //     {
    //         cout << "max_count:" << v_thea_y_info[i].count << ",type_thea_y:" << v_thea_y_info[i].thea <<  endl;
    //         cout << "size:" << v_thea_y_info[i].thea_y_.size() << endl;
    //         for(vector<thea_y>::iterator ite_thea_y_ = v_thea_y_info[i].thea_y_.begin();ite_thea_y_ != v_thea_y_info[i].thea_y_.end();++ite_thea_y_)
    //         {
    //             cout << "id1:" << ite_thea_y_->id_1 << ",id2:" << ite_thea_y_->id_2 << endl;
    //             line_clustering_.thea = ite_thea_y_->thea;
    //             line_clustering_.v_id.push_back(ite_thea_y_->id_1);
    //             line_clustering_.v_id.push_back(ite_thea_y_->id_2);
    //             v_id.push_back(ite_thea_y_->id_1);
    //             v_id.push_back(ite_thea_y_->id_2);
    //         }
    //         v_line_clustering.push_back(line_clustering_);
    //         line_clustering_.v_id.clear();
    //     }
    //     ++i;
    // }
    cout << "line_clustering_.v_id.size:" << v_line_clustering.size() << endl;
        // set<int> s(v_id.begin(),v_id.end());
        // v_id.assign(s.begin(),s.end());
    for(vector<line_clustering>::iterator ite_v_line_clustering = v_line_clustering.begin();ite_v_line_clustering != v_line_clustering.end();++ite_v_line_clustering)
    {
        while(ite_v_line_clustering->v_id.size() > 2)
        {
            set<int> s(ite_v_line_clustering->v_id.begin(),ite_v_line_clustering->v_id.end());
            ite_v_line_clustering->v_id.assign(s.begin(),s.end());
            cout << "ite_v_line_clustering->v_id.size:" << ite_v_line_clustering->v_id.size() << endl;
            for(int i = 0;i < ite_v_line_clustering->v_id.size() - 1;++i)
            {
                double point_distance = (points[ite_v_line_clustering->v_id[i]].y - points[ite_v_line_clustering->v_id[i+1]].y) / (points[ite_v_line_clustering->v_id[i]].x - points[ite_v_line_clustering->v_id[i+1]].x);
                cout << "idx1:" << points[ite_v_line_clustering->v_id[i]].x << ",idx2:" << points[ite_v_line_clustering->v_id[i+1]].x << endl;
                if(point_distance != ite_v_line_clustering->thea)
                {
                    cout << "distance:" << point_distance;
                    line_clustering_finally.thea = ite_v_line_clustering->thea;
                    // v_line_clustering_finally.thea = ite_v_line_clustering->thea;
                    for(int j = 0;j < i+1;++j)
                    {
                        line_clustering_finally.v_id.push_back(ite_v_line_clustering->v_id[j]);
                        ite_v_line_clustering->v_id.erase(ite_v_line_clustering->v_id.begin());
                        // ite_v_line_clustering->v_id.erase(ite_v_line_clustering->v_id.begin(),ite_v_line_clustering->v_id.begin() + i);
                        // v_line_clustering_finally.v_id.push_back(ite_v_line_clustering->v_id[j]);
                    }
                    v_line_clustering_finally.push_back(line_clustering_finally);
                    ite_v_line_clustering = v_line_clustering.end()  - 1;
                    b_line_clustering_finally = false;
                    break;
                }
                else
                {
                    b_line_clustering_finally = true;
                }
                // cout << (points[ite_v_line_clustering->v_id[i]].y - points[ite_v_line_clustering->v_id[i+1]].y) / (points[ite_v_line_clustering->v_id[i]].x - points[ite_v_line_clustering->v_id[i+1]].x) << ",";
                // for(int j = i+1;j < v_id.size();++j)
                // {
                //     cout << (points[v_id[i]].y - points[v_id[j]].y) / (points[v_id[i]].x - points[v_id[j]].x) << ",";
                // }
            }
            if(b_line_clustering_finally)
            {
                line_clustering_finally.v_id.clear();
                line_clustering_finally.thea = ite_v_line_clustering->thea;
                line_clustering_finally.v_id = ite_v_line_clustering->v_id;
                v_line_clustering_finally.push_back(line_clustering_finally);
                break;
            }
            cout << endl;
        }
        cout << "v_line_clustering_finally.size:" << v_line_clustering_finally.size() << endl;
    }
    for(vector<line_clustering>::iterator ite_v_line_clustering_finally = v_line_clustering_finally.begin();ite_v_line_clustering_finally != v_line_clustering_finally.end();++ite_v_line_clustering_finally)
    {
        cout << "thea:" << ite_v_line_clustering_finally->thea << endl;
        for(vector<int>::iterator ite = ite_v_line_clustering_finally->v_id.begin();ite != ite_v_line_clustering_finally->v_id.end();++ite)
        {
            cout << "id:" << *ite << endl;
        }
    }
    // for(int x : v_id)
    //     cout << x << ",";
    // cout << endl;
    // if(v_thea_y_info[0].count == v_thea_y_info.size())
    // {
    //     cout << "max_count:" << v_thea_y_info[0].count << ",type_thea_y:" << v_thea_y_info[0].thea << endl;
    // }
    // else
    // {
    //     // if()
    // }
}

ros::Publisher markerPub;

struct ult_obstacle
{
    uint32 id;
    float distance;
    float thea_distance;
    bool status;
    bool operator ==(const uint32 &x)
    {
        return(this->id==x);
    }
};


bool sortThea_distance(const ult_obstacle &p1, const ult_obstacle &p2)
{
	return p1.thea_distance > p2.thea_distance;//降序排列  
}

bool sortId(const ult_obstacle &p1, const ult_obstacle &p2)
{
	return p1.id < p2.id;//升序排列  
}

void recvultrasonicCallback(const perception_sensor_msgs::UltrasonicInfoConstPtr &msg)
{
    vector<linepoint> v_linepoint;
    vector<linepoint> v_other_linepoint;
    linepoint linepoint_;
    visualization_msgs::Marker marker;
    visualization_msgs::Marker marker_;
    visualization_msgs::Marker line;
    visualization_msgs::Marker clear;
    visualization_msgs::Marker main_point;
    visualization_msgs::Marker other_point;
    visualization_msgs::MarkerArray markerarray;
    double k = 0.0,b = 0.0;
    vector<point_> dataset;
    ult_obstacle ult_obstacle_;
    uint32 ult_id_max = 32;
    uint32 ult_id_min = 32;
    float distance_max = numeric_limits<float>::min(),distance_min = numeric_limits<float>::max();
    float distance_mean = 0.0;
    double Correlation = 0;
    vector<ult_obstacle> v_ult_obstacle;
    if(msg->obstacle_num > 0)
    {
        main_point.header.frame_id="/odom";
        main_point.header.stamp = ros::Time::now();
        main_point.ns = "basic_shapes";
        main_point.action = visualization_msgs::Marker::ADD;
        main_point.pose.orientation.w = 1.0;
        main_point.id =201;
        main_point.type = visualization_msgs::Marker::POINTS;
        main_point.scale.x = 0.2;
        main_point.scale.y = 0.2;
        main_point.scale.z = 1.0;
        main_point.color.b = 0;
        main_point.color.g = 0;
        main_point.color.r = 1.0;
        main_point.color.a = 1;

        other_point.header.frame_id="/odom";
        other_point.header.stamp = ros::Time::now();
        other_point.ns = "basic_shapes";
        other_point.action = visualization_msgs::Marker::ADD;
        other_point.pose.orientation.w = 1.0;
        other_point.id =202;
        other_point.type = visualization_msgs::Marker::POINTS;
        other_point.scale.x = 0.2;
        other_point.scale.y = 0.2;
        other_point.scale.z = 1.0;
        other_point.color.b = 0;
        other_point.color.g = 1.0;
        other_point.color.r = 0;
        other_point.color.a = 1;

        clear.header.frame_id="/odom";
        clear.header.stamp = ros::Time::now();
        clear.ns = "basic_shapes";
        clear.action = visualization_msgs::Marker::DELETE;
        clear.id = 201;
        markerarray.markers.push_back(clear);
        clear.id = 202;
        markerarray.markers.push_back(clear);
        markerPub.publish(markerarray);

        marker.header.frame_id="/odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id =0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 1.0;
        // marker.color.b = 0;
        // marker.color.g = 0;
        marker.color.r = 1.0;
        marker.color.a = 1;

        geometry_msgs::Point point;

        for(int i = 0;i < msg->obstacle_num;++i)
        {
            if(msg->ult_obstacle[i].id >= 20)
            {
                ult_obstacle_.id = msg->ult_obstacle[i].id;
                ult_obstacle_.distance = msg->ult_obstacle[i].distance;
                distance_mean += ult_obstacle_.distance;
                ult_obstacle_.status = msg->ult_obstacle[i].status;
                v_ult_obstacle.push_back(ult_obstacle_);
            }
        }
        if(!v_ult_obstacle.empty() && v_ult_obstacle.size() > 5)
        {
            distance_mean = distance_mean /  v_ult_obstacle.size();
            for(vector<ult_obstacle>::iterator ite_v_ult_obstacle = v_ult_obstacle.begin();ite_v_ult_obstacle != v_ult_obstacle.end();++ite_v_ult_obstacle)
            {
                // ROS_INFO_STREAM("ID:" << ite_v_ult_obstacle->id << ",DISTANCE:" << ite_v_ult_obstacle->distance << ",distance_mean:" << distance_mean << ",distance_max:" << distance_max << ",dis_thea:" << fabs(ite_v_ult_obstacle->distance - distance_mean));
                ite_v_ult_obstacle->thea_distance = fabs(ite_v_ult_obstacle->distance - distance_mean);
                if(distance_max < fabs(ite_v_ult_obstacle->distance - distance_mean))
                {
                    distance_max = fabs(ite_v_ult_obstacle->distance - distance_mean);
                    ult_id_max = ite_v_ult_obstacle->id;
                }
                // if(distance_min > (ite_v_ult_obstacle->distance - distance_mean))
                // {
                //     ult_id_min = ite_v_ult_obstacle->id;
                // }
                // distance_max = distance_max > (ite_v_ult_obstacle->distance - distance_mean) ? distance_max:(ite_v_ult_obstacle->distance - distance_mean);
                // distance_min = distance_min < (ite_v_ult_obstacle->distance - distance_mean) ? distance_min:(ite_v_ult_obstacle->distance - distance_mean);
            }
            for(vector<ult_obstacle>::iterator ite_v_ult_obstacle = v_ult_obstacle.begin();ite_v_ult_obstacle != v_ult_obstacle.end();++ite_v_ult_obstacle)
            {
                // ROS_INFO_STREAM("id:" << ite_v_ult_obstacle->id << ",distance:" <<ite_v_ult_obstacle->distance << ",thea_distance:" << ite_v_ult_obstacle->thea_distance);
            }
            sort(v_ult_obstacle.begin(),v_ult_obstacle.end(),sortThea_distance);
            marker_.header.frame_id="/odom";
            marker_.header.stamp = ros::Time::now();
            marker_.ns = "basic_shapes";
            marker_.action = visualization_msgs::Marker::ADD;
            marker_.pose.orientation.w = 1.0;
            marker_.id =99;
            marker_.type = visualization_msgs::Marker::POINTS;
            marker_.scale.x = 0.2;
            marker_.scale.y = 0.2;
            marker_.scale.z = 1.0;
            // marker.color.b = 0;
            // marker.color.g = 0;
            marker_.color.g = 1.0;
            marker_.color.a = 1;
            linepoint_.x = -3380.4 + 613.6 * (v_ult_obstacle.begin()->id - 20);
            linepoint_.y = 1432.5 + v_ult_obstacle.begin()->distance * 1000;
            point.x = linepoint_.x / 1000;
            point.y = linepoint_.y / 1000;
            point.z =0;
            marker_.points.push_back(point);
            markerarray.markers.push_back(marker_);
            // v_ult_obstacle.erase(v_ult_obstacle.begin());
            // ROS_INFO_STREAM("ult_id_max:" << ult_id_max << ",ult_id_min:" << ult_id_min);
            // if(v_ult_obstacle.size() >= 6)
            // {
            //     vector<ult_obstacle>::iterator ite_v_ult_obstacle_=find(v_ult_obstacle.begin(),v_ult_obstacle.end(),ult_id_max);
            //     if(ite_v_ult_obstacle_ != v_ult_obstacle.end())
            //     {
            //         ROS_INFO_STREAM("max_ite_id:" << ite_v_ult_obstacle_->id << ",distance:" << ite_v_ult_obstacle_->distance);
            //         v_ult_obstacle.erase(ite_v_ult_obstacle_);
            //     }
            // }
            sort(v_ult_obstacle.begin(),v_ult_obstacle.end(),sortId);
            for(vector<ult_obstacle>::iterator ite_v_ult_obstacle = v_ult_obstacle.begin();ite_v_ult_obstacle != v_ult_obstacle.end();++ite_v_ult_obstacle)
            {
                linepoint_.x = -3380.4 + 613.6 * (ite_v_ult_obstacle->id - 20);
                linepoint_.y = 1432.5 + ite_v_ult_obstacle->distance * 1000;
                // point.x = linepoint_.x / 1000;
                // point.y = linepoint_.y / 1000;
                // point.z =0;
                //     // pose.orientation.w = 1.0;
                // point_ p(point.x,point.y,0);
                // dataset.push_back(p);
                // marker.points.push_back(point);
                v_linepoint.push_back(linepoint_);
                // markerarray.markers.push_back(marker);
                // ROS_INFO_STREAM("id:" << ite_v_ult_obstacle->id);
            }
            vector<vector<linepoint>> v_line;
            // find_line(v_linepoint,v_line);
            // for(int i = 0;i < msg->obstacle_num;++i)
            // {
            //     if(msg->ult_obstacle[i].id >= 20)
            //     {
            //         linepoint_.x = -3380.4 + 613.6 * (msg->ult_obstacle[i].id - 20);
            //         linepoint_.y = 1432.5 + msg->ult_obstacle[i].distance * 1000;
            //         point.x = linepoint_.x / 1000;
            //         point.y = linepoint_.y / 1000;
            //         point.z =0;
            //         // pose.orientation.w = 1.0;
            //         point_ p(point.x,point.y,0);
            //         dataset.push_back(p);
            //         marker.points.push_back(point);
            //         v_linepoint.push_back(linepoint_);
            //         // ROS_INFO_STREAM("id:" << msg->ult_obstacle[i].id << ",distance:" << msg->ult_obstacle[i].distance);
            //         // ROS_INFO_STREAM("x:" << linepoint_.x << ",y:" << linepoint_.y);
            //     }
            //     // else
            //     // {
            //     //     ROS_INFO_STREAM("id:" << msg->ult_obstacle[i].id << ",distance:" << msg->ult_obstacle[i].distance);
            //     // }
            //     // else if(msg->ult_obstacle[i].id >= 16)
            //     // {
            //     //     linepoint_.x = -7735 - (msg->ult_obstacle[i].distance * 1000);
            //     //     linepoint_.y = -1432.5 + 400 * (msg->ult_obstacle[i].id - 16);
            //     //     point.x = linepoint_.x / 1000;
            //     //     point.y = linepoint_.y / 1000;
            //     //     point.z =0;
            //     //     // pose.orientation.w = 1.0;
            //     //     marker.points.push_back(point);
            //     // }
            //     // else if(msg->ult_obstacle[i].id >= 4)
            //     // {
            //     //     linepoint_.x = 3380.4 - 613.6 * (msg->ult_obstacle[i].id - 4);
            //     //     linepoint_.y = -1432.5 - msg->ult_obstacle[i].distance * 1000;
            //     //     point.x = linepoint_.x / 1000;
            //     //     point.y = linepoint_.y / 1000;
            //     //     point.z =0;
            //     //     // pose.orientation.w = 1.0;
            //     //     marker.points.push_back(point);
            //     // }
            //     // else
            //     // {
            //     //     linepoint_.x = 7735 + (msg->ult_obstacle[i].distance * 1000);
            //     //     linepoint_.y = 1432.5 - 400 * (msg->ult_obstacle[i].id);
            //     //     point.x = linepoint_.x / 1000;
            //     //     point.y = linepoint_.y / 1000;
            //     //     point.z =0;
            //     //     // pose.orientation.w = 1.0;
            //     //     marker.points.push_back(point);
            //     // }
            //     markerarray.markers.push_back(marker);
            // }
            // lineFit_(v_linepoint,k,b,Correlation);
            ROS_INFO("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
            // double Covariance = compute_Covariance(v_linepoint);
            // ROS_INFO_STREAM("compute_Covariance:" << Covariance);
            // classify_point_(v_linepoint,v_other_linepoint,Covariance);
            double Correlation_temp = 2;
            while(ros::ok())
            {
                classify_point(v_linepoint,v_other_linepoint,Correlation);
                if(v_linepoint.size() <= 4)
                {
                    break;
                }
                if(Correlation_temp == Correlation)
                {
                    break;
                }
                else
                {
                    Correlation_temp = Correlation;
                }
            }
            // classify_point(v_linepoint,v_other_linepoint);
            lineFit_(v_linepoint,k,b,Correlation);
            ROS_INFO_STREAM("k:" << k << ",b:" << b);
            for(vector<linepoint>::iterator ite_v_ult_obstacle = v_linepoint.begin();ite_v_ult_obstacle != v_linepoint.end();++ite_v_ult_obstacle)
            {
                point.x = ite_v_ult_obstacle->x / 1000;
                point.y = ite_v_ult_obstacle->y / 1000;
                point.z =0;
                    // pose.orientation.w = 1.0;
                point_ p(point.x,point.y,0);
                main_point.points.push_back(point);
                markerarray.markers.push_back(main_point);
                // ROS_INFO_STREAM("id:" << ite_v_ult_obstacle->id);
            }
            for(vector<linepoint>::iterator ite_v_ult_obstacle = v_other_linepoint.begin();ite_v_ult_obstacle != v_other_linepoint.end();++ite_v_ult_obstacle)
            {
                point.x = ite_v_ult_obstacle->x / 1000;
                point.y = ite_v_ult_obstacle->y / 1000;
                point.z =0;
                    // pose.orientation.w = 1.0;
                point_ p(point.x,point.y,0);
                other_point.points.push_back(point);
                markerarray.markers.push_back(other_point);
                // ROS_INFO_STREAM("id:" << ite_v_ult_obstacle->id);
            }
            line.header.frame_id="/odom";
            line.header.stamp = ros::Time::now();
            line.ns = "basic_shapes";
            line.action = visualization_msgs::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.id =1;
            line.type = visualization_msgs::Marker::LINE_LIST;
            line.scale.x = 0.1;
            line.scale.y = 0.1;
            line.color.b = 1.0;
            line.color.a = 1;
            point.x = 3.3804;
            point.y = (point.x * 1000 * k + b) / 1000;
            point.z =0;
            // ROS_INFO_STREAM("x:" << point.x << ",y:" << point.y);
            line.points.push_back(point);
            // marker.points.push_back(point);
            point.x = -3.3804;
            point.y = (point.x * 1000 * k + b) / 1000;
            point.z =0;
            // ROS_INFO_STREAM("x:" << point.x << ",y:" << point.y);
            // marker.points.push_back(point);
            line.points.push_back(point);
            markerarray.markers.push_back(line);

            marker.header.frame_id="/odom";
            marker.header.stamp = ros::Time::now();
            marker.ns = "basic_shapes";
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.id =2;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            geometry_msgs::Pose pose;
            pose.position.x =  3.3804;
            pose.position.y =  1.4325;
            pose.position.z =0;
            ostringstream str;
            str<<"F";
            marker.text=str.str();
            marker.pose=pose;

            marker.scale.z = 0.2;
            marker.color.b = 1.0;
            marker.color.a = 1;
            markerarray.markers.push_back(marker);

            marker.header.frame_id="/odom";
            marker.header.stamp = ros::Time::now();
            marker.ns = "basic_shapes";
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.id =3;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            // geometry_msgs::Pose pose;
            pose.position.x =  -3.3804;
            pose.position.y =  1.4325;
            pose.position.z =0;
            ostringstream str1;
            str1<<"T";
            marker.text=str1.str();
            marker.pose=pose;

            marker.scale.z = 0.2;
            marker.color.b = 1.0;
            marker.color.a = 1;
            markerarray.markers.push_back(marker);

            marker.header.frame_id="/odom";
            marker.header.stamp = ros::Time::now();
            marker.ns = "basic_shapes";
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.id = 4;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            // geometry_msgs::Pose pose;
            pose.position.x =  0;
            pose.position.y =  0;
            pose.position.z =0;
            ostringstream str2;
            // str2<<"O";
            str2 << Correlation << "," << k;
            // str2 << Covariance;
            marker.text=str2.str();
            marker.pose=pose;

            marker.scale.z = 0.2;
            marker.color.b = 1.0;
            marker.color.a = 1;
            markerarray.markers.push_back(marker);

            markerPub.publish(markerarray);
            // lineFit_(v_linepoint,k,b);
            // ROS_INFO_STREAM("k:" << k << ",b:" << b << ",thea:" << (atan(k)/3.141592) * 180);
            // k_means(dataset,7);
        }
    }
    else
    {
        clear.header.frame_id="/odom";
        clear.header.stamp = ros::Time::now();
        clear.ns = "basic_shapes";
        clear.action = visualization_msgs::Marker::DELETEALL;
    }
}

bool array_recv[9] = {0,0,0,0,0,0,0,0,0};

// void recvcameraCallback(const perception_sensor_msgs::ObjectListConstPtr &msg)
// {
//     if(msg->sensor_index == 6)
//     {
//     visualization_msgs::Marker obst_mark;
//     visualization_msgs::Marker car_mark;
//     visualization_msgs::Marker string_mark;
//     visualization_msgs::MarkerArray markerarray;

//     static vector<int> array_id[9];

//     obst_mark.header.frame_id="/odom";
//     obst_mark.header.stamp = ros::Time::now();
//     obst_mark.ns = "basic_shapes";
//     obst_mark.action = visualization_msgs::Marker::ADD;
//     obst_mark.pose.orientation.w = 0.1;
//     obst_mark.id = msg->sensor_index;
//     obst_mark.type = visualization_msgs::Marker::LINE_STRIP;
//     obst_mark.scale.x = 0.1;
//     obst_mark.scale.y = 0.1;
//       // Points are green
//     switch(int(msg->sensor_index))
//     {
//         case 1:
//             obst_mark.color.r = 1;
//             obst_mark.color.g = 0;
//             obst_mark.color.b = 0;
//             if(msg->object_list.size() != 0)
//             {
//                 array_recv[1] = true;
//             }
//             break;
//         case 3:
//             obst_mark.color.r = 0;
//             obst_mark.color.g = 1;
//             obst_mark.color.b = 0;
//             if(msg->object_list.size() != 0)
//             {
//                 array_recv[3] = true;
//             }
//             break;
//         case 5:
//             obst_mark.color.r = 0;
//             obst_mark.color.g = 0;
//             obst_mark.color.b = 1;
//             if(msg->object_list.size() != 0)
//             {
//                 array_recv[5] = true;
//             }
//             break;
//         case 6:
//             obst_mark.color.r = 1;
//             obst_mark.color.g = 1;
//             obst_mark.color.b = 0;
//             if(msg->object_list.size() != 0)
//             {
//                 array_recv[6] = true;
//             }
//             break;
//         case 7:
//             obst_mark.color.r = 1;
//             obst_mark.color.g = 0;
//             obst_mark.color.b = 1;
//             if(msg->object_list.size() != 0)
//             {
//                 array_recv[7] = true;
//             }
//             break;
//         case 8:
//             obst_mark.color.r = 0;
//             obst_mark.color.g = 1;
//             obst_mark.color.b = 1;
//             if(msg->object_list.size() != 0)
//             {
//                 array_recv[8] = true;
//             }
//             break;
//         default:
//             break;
//     }
//     obst_mark.color.a = 1.0;
//     // if(int(msg->sensor_index) != 6)
//     // {
//     //     obst_mark.color.a = 1.0;
//     // }
//     // else
//     // {
//     //     obst_mark.color.a = 0;
//     // }f(msg->sensor_index == 6)
//     // {
    
//     if(msg->object_list.size() != 0)
//     {
//         for(int i = 0;i < msg->object_list.size();++i)
//         {
//             ROS_INFO_STREAM("INDEX:" << msg->sensor_index << ",SIZE:" << msg->object_list.size() << ",i:" << i);   
//             geometry_msgs::Point p;

//             obst_mark.points.clear();
            
//             double posex = 0.0;
//             double posey = 0.0;

//             p.x = msg->object_list.at(i).peek.at(0).x;
//             p.y = msg->object_list.at(i).peek.at(0).y;
//             p.z = 0;
//             ROS_INFO_STREAM("x:" << p.x << ",y:" << p.y);
//             posex += p.x;
//             posey += p.y;
//             obst_mark.points.push_back(p);
//             p.x = msg->object_list.at(i).peek.at(1).x;
//             p.y = msg->object_list.at(i).peek.at(1).y;
//             p.z = 0;
//             ROS_INFO_STREAM("x:" << p.x << ",y:" << p.y);
//             posex += p.x;
//             posey += p.y;
//             obst_mark.points.push_back(p);
//             p.x = msg->object_list.at(i).peek.at(2).x;
//             p.y = msg->object_list.at(i).peek.at(2).y;
//             p.z = 0;
//             ROS_INFO_STREAM("x:" << p.x << ",y:" << p.y);
//             posex += p.x;
//             posey += p.y;
//             obst_mark.points.push_back(p);
//             p.x = msg->object_list.at(i).peek.at(3).x;
//             p.y = msg->object_list.at(i).peek.at(3).y;
//             p.z = 0;
//             ROS_INFO_STREAM("x:" << p.x << ",y:" << p.y);
//             posex += p.x;
//             posey += p.y;
//             obst_mark.points.push_back(p);
//             p.x = msg->object_list.at(i).peek.at(0).x;
//             p.y = msg->object_list.at(i).peek.at(0).y;
//             p.z = 0;
//             ROS_INFO_STREAM("x:" << p.x << ",y:" << p.y);
//             obst_mark.points.push_back(p);
//             markerarray.markers.push_back(obst_mark);

//             string_mark.header.frame_id="/odom";
//             string_mark.header.stamp = ros::Time::now();
//             string_mark.ns = "basic_shapes";
//             string_mark.action = visualization_msgs::Marker::ADD;
//             string_mark.pose.orientation.w = 1.0;
//             string_mark.id =100 + int(msg->sensor_index);
//             string_mark.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//             geometry_msgs::Pose pose;
//             // pose.position.x =  (msg->object_list.at(i).peek.at(0).x + msg->object_list.at(i).peek.at(1).x
//             //                   +msg->object_list.at(i).peek.at(2).x + msg->object_list.at(i).peek.at(3).x)/4;
//             // pose.position.y =  (msg->object_list.at(i).peek.at(0).y + msg->object_list.at(i).peek.at(1).y
//             //                   +msg->object_list.at(i).peek.at(2).y + msg->object_list.at(i).peek.at(3).y)/4;
//             pose.position.x =  posex/4;
//             pose.position.y =  posey/4;
//             pose.position.z =0;
//             ostringstream str2;
//             str2 << msg->object_list.at(i).id;
//             string str_mark = to_string(msg->sensor_index) + "," + str2.str() + "," + to_string(msg->object_list.at(i).obj_class)
//                                 + "(" + to_string(pose.position.x) + "," + to_string(pose.position.y) + ")";
//             // obst_mark.text=str2.str();
//             string_mark.text=str_mark;
//             string_mark.pose=pose;
//             string_mark.scale.z = 0.5;
//             switch(int(msg->sensor_index))
//             {
//                 case 1:
//                     string_mark.color.r = 1;
//                     string_mark.color.g = 0;
//                     string_mark.color.b = 0;
//                     if(posey < 7.5)
//                     {
//                         ROS_INFO("camera index 1 distance_y < 7.5:%f",posey);
//                     }
//                     break;
//                 case 3:
//                     string_mark.color.r = 0;
//                     string_mark.color.g = 1;
//                     string_mark.color.b = 0;
//                     if(posey > -7.5)
//                     {
//                         ROS_INFO("camera index 3 distance_y > -7.5:%f",posey);
//                     }
//                     break;
//                 case 5:
//                     string_mark.color.r = 0;
//                     string_mark.color.g = 0;
//                     string_mark.color.b = 1;
//                     if(posex < 1.1)
//                     {
//                         ROS_INFO("camera index 5 distance_y < 1.1:%f",posex);
//                     }
//                     break;
//                 case 6:
//                     string_mark.color.r = 1;
//                     string_mark.color.g = 1;
//                     string_mark.color.b = 0;
//                     if(posex < 1.1)
//                     {
//                         ROS_INFO("camera index 6 distance_y < 1.1:%f",posex);
//                     }
//                     break;
//                 case 7:
//                     string_mark.color.r = 1;
//                     string_mark.color.g = 0;
//                     string_mark.color.b = 1;
//                     if(posex > -1.1)
//                     {
//                         ROS_INFO("camera index 7 distance_y > -1.1:%f",posex);
//                     }
//                     break;
//                 case 8:
//                     string_mark.color.r = 0;
//                     string_mark.color.g = 1;
//                     string_mark.color.b = 1;
//                     if(posex > -1.1)
//                     {
//                         ROS_INFO("camera index 8 distance_y > -1.1:%f",posex);
//                     }
//                     break;
//                 default:
//                     break;
//             }
//             // string_mark.color.b = 1.0;
//             string_mark.color.a = 1;
//             markerarray.markers.push_back(string_mark);
//             ROS_INFO_STREAM("markerarray.markers.SIZE:" << markerarray.markers.size());
//         }
//     }
//     else
//     {
//         obst_mark.header.frame_id="/odom";
//         obst_mark.header.stamp = ros::Time::now();
//         obst_mark.ns = "basic_shapes";
//         obst_mark.id = int(msg->sensor_index);
//         obst_mark.action = visualization_msgs::Marker::DELETE;
//         markerarray.markers.push_back(obst_mark);
//         obst_mark.id = 100 + int(msg->sensor_index);
//         markerarray.markers.push_back(obst_mark);
//     }
//     car_mark.header.frame_id="/odom";
//     car_mark.header.stamp = ros::Time::now();
//     car_mark.ns = "basic_shapes";
//     car_mark.action = visualization_msgs::Marker::ADD;
//     car_mark.pose.orientation.w = 0.1;
//     car_mark.id = 100;
//     car_mark.type = visualization_msgs::Marker::LINE_STRIP;
//     car_mark.scale.x = 0.1;
//     car_mark.scale.y = 0.1;
//     car_mark.color.r = 1;
//     car_mark.color.g = 0;
//     car_mark.color.b = 0;
//     car_mark.color.a = 1.0;
//     car_mark.points.clear();
//     geometry_msgs::Point po;
//     po.x = 1.4325;
//     po.y = 7.735;
//     po.z = 0;
//     car_mark.points.push_back(po);
//     po.x = 1.4325;
//     po.y = -7.735;
//     po.z = 0;
//     car_mark.points.push_back(po);
//     po.x = -1.4325;
//     po.y = -7.735;
//     po.z = 0;
//     car_mark.points.push_back(po);
//     po.x = -1.4325;
//     po.y = 7.735;
//     po.z = 0;
//     car_mark.points.push_back(po);
//     po.x = 1.4325;
//     po.y = 7.735;
//     po.z = 0;
//     car_mark.points.push_back(po);
//     markerarray.markers.push_back(car_mark);
//     markerPub.publish(markerarray);
//     }
// }

void recvcameraCallback(const perception_sensor_msgs::ObjectListConstPtr &msg)
{
    // if(msg->sensor_index == 5)
    // {
    // ROS_INFO("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    visualization_msgs::Marker clear_mark;
    visualization_msgs::Marker obst_mark;
    visualization_msgs::Marker car_mark;
    visualization_msgs::Marker string_mark;
    visualization_msgs::MarkerArray markerarray;

    static vector<int> array_id[9];

    clear_mark.header.frame_id="/odom";
    clear_mark.header.stamp = ros::Time::now();
    clear_mark.ns = "basic_shapes";
    clear_mark.action = visualization_msgs::Marker::DELETE;
    // for(vector<int>::iterator ite_array_id = array_id[msg->sensor_index].begin();ite_array_id != array_id[msg->sensor_index].end();++ite_array_id)
    // {
    //     clear_mark.id = *ite_array_id;
    //     markerarray.markers.push_back(clear_mark);
    // }
    for(int i = 0;i < 33;++i)
    {
        clear_mark.id = msg->sensor_index * 100 + i;
        markerarray.markers.push_back(clear_mark);
        clear_mark.id = 1000 + msg->sensor_index * 100 + i;
        markerarray.markers.push_back(clear_mark);
    }
    array_id[msg->sensor_index].clear();
    markerPub.publish(markerarray);

    if(msg->object_list.size() != 0)
    {
        obst_mark.header.frame_id="/odom";
        obst_mark.header.stamp = ros::Time::now();
        obst_mark.ns = "basic_shapes";
        obst_mark.action = visualization_msgs::Marker::ADD;
        obst_mark.pose.orientation.w = 0.1;
        // obst_mark.id = msg->sensor_index;
        obst_mark.type = visualization_msgs::Marker::LINE_STRIP;
        obst_mark.scale.x = 0.1;
        obst_mark.scale.y = 0.1;

        switch(int(msg->sensor_index))
        {
            case 1:
                obst_mark.color.r = 1;
                obst_mark.color.g = 0;
                obst_mark.color.b = 0;
                if(msg->object_list.size() != 0)
                {
                    array_recv[1] = true;
                }
                break;
            case 3:
                obst_mark.color.r = 0;
                obst_mark.color.g = 1;
                obst_mark.color.b = 0;
                if(msg->object_list.size() != 0)
                {
                    array_recv[3] = true;
                }
                break;
            case 5:
                obst_mark.color.r = 0;
                obst_mark.color.g = 0;
                obst_mark.color.b = 1;
                if(msg->object_list.size() != 0)
                {
                    array_recv[5] = true;
                }
                break;
            case 6:
                obst_mark.color.r = 1;
                obst_mark.color.g = 1;
                obst_mark.color.b = 0;
                if(msg->object_list.size() != 0)
                {
                    array_recv[6] = true;
                }
                break;
            case 7:
                obst_mark.color.r = 1;
                obst_mark.color.g = 0;
                obst_mark.color.b = 1;
                if(msg->object_list.size() != 0)
                {
                    array_recv[7] = true;
                }
                break;
            case 8:
                obst_mark.color.r = 0;
                obst_mark.color.g = 1;
                obst_mark.color.b = 1;
                if(msg->object_list.size() != 0)
                {
                    array_recv[8] = true;
                }
                break;
            default:
                break;
        }
        obst_mark.color.a = 1.0;

        for(int i = 0;i < msg->object_list.size();++i)
        {
            // ROS_INFO_STREAM("INDEX:" << msg->sensor_index << ",SIZE:" << msg->object_list.size() << ",i:" << i);   
            geometry_msgs::Point p;

            obst_mark.points.clear();
                
            double posex = 0.0;
            double posey = 0.0;

            obst_mark.id = msg->sensor_index * 100 + msg->object_list.at(i).id;
            vector<int>::iterator ite_find = find(array_id[int(msg->sensor_index)].begin(),array_id[int(msg->sensor_index)].end(),obst_mark.id);
            if(ite_find == array_id[int(msg->sensor_index)].end())
            {
                array_id[int(msg->sensor_index)].push_back(int(obst_mark.id));
                // ROS_INFO_STREAM("push_obs:" << obst_mark.id);
            }

            p.x = msg->object_list.at(i).peek.at(0).x;
            p.y = msg->object_list.at(i).peek.at(0).y;
            p.z = 0;
            // ROS_INFO_STREAM("x:" << p.x << ",y:" << p.y);
            posex += p.x;
            posey += p.y;
            obst_mark.points.push_back(p);
            p.x = msg->object_list.at(i).peek.at(1).x;
            p.y = msg->object_list.at(i).peek.at(1).y;
            p.z = 0;
            // ROS_INFO_STREAM("x:" << p.x << ",y:" << p.y);
            posex += p.x;
            posey += p.y;
            obst_mark.points.push_back(p);
            p.x = msg->object_list.at(i).peek.at(2).x;
            p.y = msg->object_list.at(i).peek.at(2).y;
            p.z = 0;
            // ROS_INFO_STREAM("x:" << p.x << ",y:" << p.y);
            posex += p.x;
            posey += p.y;
            obst_mark.points.push_back(p);
            p.x = msg->object_list.at(i).peek.at(3).x;
            p.y = msg->object_list.at(i).peek.at(3).y;
            p.z = 0;
            // ROS_INFO_STREAM("x:" << p.x << ",y:" << p.y);
            posex += p.x;
            posey += p.y;
            obst_mark.points.push_back(p);
            p.x = msg->object_list.at(i).peek.at(0).x;
            p.y = msg->object_list.at(i).peek.at(0).y;
            p.z = 0;
            // ROS_INFO_STREAM("x:" << p.x << ",y:" << p.y);
            obst_mark.points.push_back(p);
            markerarray.markers.push_back(obst_mark);

            string_mark.header.frame_id="/odom";
            string_mark.header.stamp = ros::Time::now();
            string_mark.ns = "basic_shapes";
            string_mark.action = visualization_msgs::Marker::ADD;
            string_mark.pose.orientation.w = 1.0;
            string_mark.id =1000 + int(msg->sensor_index)*100 + msg->object_list.at(i).id;
            vector<int>::iterator ite_find_str = find(array_id[int(msg->sensor_index)].begin(),array_id[int(msg->sensor_index)].end(),string_mark.id);
            if(ite_find_str == array_id[int(msg->sensor_index)].end())
            {
                // ROS_INFO_STREAM("push_string:" << string_mark.id);
                array_id[int(msg->sensor_index)].push_back(int(string_mark.id));
            }
            string_mark.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            geometry_msgs::Pose pose;
            pose.position.x =  posex/4;
            pose.position.y =  posey/4;
            pose.position.z =0;
            ostringstream str2;
            str2 << msg->object_list.at(i).id;
            string str_mark = to_string(msg->sensor_index) + "," + str2.str() + "," + to_string(msg->object_list.at(i).obj_class)
                                + "(" + to_string(pose.position.x) + "," + to_string(pose.position.y) + ")";
            // obst_mark.text=str2.str();
            string_mark.text=str_mark;
            string_mark.pose=pose;
            string_mark.scale.z = 0.5;
            switch(int(msg->sensor_index))
            {
                case 1:
                    string_mark.color.r = 1;
                    string_mark.color.g = 0;
                    string_mark.color.b = 0;
                    if(posey < 7.5)
                    {
                        ROS_INFO("camera index 1 distance_y < 7.5:%f",posey);
                    }
                    break;
                case 3:
                    string_mark.color.r = 0;
                    string_mark.color.g = 1;
                    string_mark.color.b = 0;
                    if(posey > -7.5)
                    {
                        ROS_INFO("camera index 3 distance_y > -7.5:%f",posey);
                    }
                    break;
                case 5:
                    string_mark.color.r = 0;
                    string_mark.color.g = 0;
                    string_mark.color.b = 1;
                    if(posex < 1.1)
                    {
                        ROS_INFO("camera index 5 distance_x < 1.1:%f",posex);
                    }
                    break;
                case 6:
                    string_mark.color.r = 1;
                    string_mark.color.g = 1;
                    string_mark.color.b = 0;
                    if(posex < 1.1)
                    {
                        ROS_INFO("camera index 6 distance_x < 1.1:%f",posex);
                    }
                    break;
                case 7:
                    string_mark.color.r = 1;
                    string_mark.color.g = 0;
                    string_mark.color.b = 1;
                    if(posex > -1.1)
                    {
                        ROS_INFO("camera index 7 distance_x > -1.1:%f",posex);
                    }
                    break;
                case 8:
                    string_mark.color.r = 0;
                    string_mark.color.g = 1;
                    string_mark.color.b = 1;
                    if(posex > -1.1)
                    break;
                default:
                    break;
            }
            // string_mark.color.b = 1.0;
            string_mark.color.a = 1;
            markerarray.markers.push_back(string_mark);
            // ROS_INFO_STREAM("markerarray.markers.SIZE:" << markerarray.markers.size());
        }
    }
    // else
    // {
    //     if(array_id[int(msg->sensor_index)].size() != 0)
    //     {
    //         obst_mark.header.frame_id="/odom";
    //         obst_mark.header.stamp = ros::Time::now();
    //         obst_mark.ns = "basic_shapes";
    //         obst_mark.action = visualization_msgs::Marker::DELETE;
    //         for(vector<int>::iterator ite_array_id = array_id[int(msg->sensor_index)].begin();ite_array_id != array_id[int(msg->sensor_index)].end();++ite_array_id)
    //         {
    //             obst_mark.id = *ite_array_id;
    //             markerarray.markers.push_back(obst_mark);
    //             ROS_INFO_STREAM("clear_id:" << obst_mark.id);
    //         }
    //         array_id[int(msg->sensor_index)].clear();
    //     }
    // }
    car_mark.header.frame_id="/odom";
    car_mark.header.stamp = ros::Time::now();
    car_mark.ns = "basic_shapes";
    car_mark.action = visualization_msgs::Marker::ADD;
    car_mark.pose.orientation.w = 0.1;
    car_mark.id = 0;
    car_mark.type = visualization_msgs::Marker::LINE_STRIP;
    car_mark.scale.x = 0.1;
    car_mark.scale.y = 0.1;
    car_mark.color.r = 1;
    car_mark.color.g = 0;
    car_mark.color.b = 0;
    car_mark.color.a = 1.0;
    car_mark.points.clear();
    geometry_msgs::Point po;
    po.x = 1.4325;
    po.y = 7.735;
    po.z = 0;
    car_mark.points.push_back(po);
    po.x = 1.4325;
    po.y = -7.735;
    po.z = 0;
    car_mark.points.push_back(po);
    po.x = -1.4325;
    po.y = -7.735;
    po.z = 0;
    car_mark.points.push_back(po);
    po.x = -1.4325;
    po.y = 7.735;
    po.z = 0;
    car_mark.points.push_back(po);
    po.x = 1.4325;
    po.y = 7.735;
    po.z = 0;
    car_mark.points.push_back(po);
    markerarray.markers.push_back(car_mark);
    markerPub.publish(markerarray);
    // }
}

void clear_mark()
{
    while(ros::ok())
    {
        visualization_msgs::Marker obst_mark;
        visualization_msgs::MarkerArray markerarray;
        for(int i = 1;i <= 9;++i)
        {
            if(!array_recv[i])
            {
                obst_mark.header.frame_id="/odom";
                obst_mark.header.stamp = ros::Time::now();
                obst_mark.ns = "basic_shapes";
                obst_mark.id = i;
                obst_mark.action = visualization_msgs::Marker::DELETE;
                markerarray.markers.push_back(obst_mark);
                ROS_INFO("clear id:%d",i);
            }
        }
        markerPub.publish(markerarray);
        usleep(1000 * 300);
    }
}

int main(int argc,char* argv[])
{
    ros::init(argc,argv,"test_sub");
    ros::NodeHandle nh;

    markerPub = nh.advertise<visualization_msgs::MarkerArray>("TEXT_VIEW_FACING", 10);
    ros::Subscriber trasonic_sub        = nh.subscribe("/drivers/can_wr/sonser_info", 10, recvultrasonicCallback);
    // ros::Subscriber camera_sub        = nh.subscribe("/drivers/perception/camera_obstacle_info", 10, recvcameraCallback);

    // int kk = 2;
    // KMEANS<double> kms(kk);
	// kms.loadDataSet(filename);
	// kms.randCent();
	// kms.kmeans();

    // vector<point_> dataset = openFile("dataset3.txt");
	// k_means(dataset,7);

    while(ros::ok())
    {
        ros::spin();
    }

    double line_a = 0;
    double line_b = 0;
    double line_c = 0;
    double line_k = 0,line_bb = 0;
    double Cov_XY = 0,x_variance = 0,y_variance = 0;

    double Correlation = 0;

    vector<linepoint> v_linepoint;
    linepoint linepoint_;
    for(int i = 0;i < 12;++i)
    {
        linepoint_.x = i;
        if(i == 1 || i == 3 || i == 7 || i == 11)
        {
            linepoint_.y = 20;
        }
        else
        {
            linepoint_.y = 15;
        }
        // if(i < 6)
        // {
        //     linepoint_.y = 20 - i;
        // }
        // else if(i < 14)
        // {
        //     linepoint_.y = 20 - 2*i;
        // }
        // else
        // {
        //     linepoint_.y = 10 - 4*i;
        // }
        cout << "x:" << linepoint_.x << ",y:" << linepoint_.y << endl;
        v_linepoint.push_back(linepoint_);
    }
    Clustering(v_linepoint);
    lineFit_(v_linepoint,line_k,line_bb,Correlation);
    ROS_INFO("k:%lf,b:%lf",line_k,line_bb);
    lineFit(v_linepoint,line_a,line_b,line_c);
    cout << ",a:" << line_a << ",b:" << line_b << ",c:" << line_c << ",k:" << -(line_a/line_b) << ",b:" << -(line_c/line_b) << endl;

    cout << "angle:" << (atan(660/660) * 180)/3.1415926 << endl;
    while(ros::ok())
    {
        sleep(1);
    }

    float length_car = 15470;
    float weight = 2865;
    float lf = 5508.5;
    float lr = 5508.5;
    float af = 30;
    float ar = 00;
    float veh_v = 1000.0;
    float turn_v = 0;
    float ox = 0,oy = 0;
    float R = 0,R1 = 0,R2 = 0,R3 = 0,R4 = 0;
    float out_R = 0;
    float max_out = 0;
    point point_;
    vector<point> v_ultrantic_left;
    vector<point> v_ultrantic_right;
    vector<point> v_ultrantic_front;
    vector<point> v_ultrantic_tail;
    vector<point> v_ultrantic_regional;
    for(int i = 0;i < 4;++i)
    {
        point_.index = i;
        point_.x     = length_car/2;
        point_.y     = 542.4 - 361.6 * i;
        v_ultrantic_front.push_back(point_);
    }
    for(int i = 4;i < 16;++i)
    {
        point_.index = i;
        point_.x     = 3380.4 - (614.6) * (i - 4);
        point_.y     = -weight/2;
        v_ultrantic_left.push_back(point_);
    }
    for(int i = 16;i < 20;++i)
    {
        point_.index = i;
        point_.x     = -length_car/2;
        point_.y     = -542.4 + 361.6 * (i - 16);
        v_ultrantic_front.push_back(point_);
    }
    for(int i = 20;i < 32;++i)
    {
        point_.index = i;
        point_.x     = -3380.4 + (614.6) * (i - 20);
        point_.y     = weight/2;
        v_ultrantic_right.push_back(point_);
    }
    for(vector<point>::iterator ite_v_ultrantic = v_ultrantic_left.begin();ite_v_ultrantic != v_ultrantic_left.end();++ite_v_ultrantic)
    {
        ROS_INFO_STREAM("ult_index:" << ite_v_ultrantic->index << ",ult_x:" << ite_v_ultrantic->x << ",ult_y:" << ite_v_ultrantic->y);
    }
    for(vector<point>::iterator ite_v_ultrantic = v_ultrantic_right.begin();ite_v_ultrantic != v_ultrantic_right.end();++ite_v_ultrantic)
    {
        ROS_INFO_STREAM("ult_index:" << ite_v_ultrantic->index << ",ult_x:" << ite_v_ultrantic->x << ",ult_y:" << ite_v_ultrantic->y);
    }
    // if(af == 0 && ar == 0)
    // {
    //     turn_v = 0;
    // }
    // else
    // {
    //     if(af*ar >= 0)
    //     {
    //         if(af == ar)
    //         {substr
    //             turn_v = 0;
    //         }
    //         else
    //         {
    //             turn_v = (180/pi)*veh_v/(5.5085*sqrt((4+(tan(abs(af)*pi/180) + tan(abs(ar)*pi/180))*(tan(abs(af)*pi/180) + tan(abs(ar)*pi/180)))/((tan(abs(ar)*pi/180) - tan(abs(af)*pi/180))*(tan(abs(ar)*pi/180) - tan(abs(af)*pi/180)))));
    //         }
    //     }
    //     else
    //     {
    //         turn_v = (180/pi)*veh_v/(5.5085*sqrt((4+(tan(abs(af)*pi/180) - tan(abs(ar)*pi/180))*(tan(abs(af)*pi/180) - tan(abs(ar)*pi/180)))/((tan(abs(ar)*pi/180) + tan(abs(af)*pi/180))*(tan(abs(ar)*pi/180) + tan(abs(af)*pi/180)))));
    //     }
    // }
    af = (af*pi)/180.0;
    ar = (ar*pi)/180.0;
    ROS_INFO_STREAM("af:" << af);
    
    // float F = 1/tan((lf*tan(ar) + lr*tan(af))/(lf+lr));
    float F = atan((lf*tan(ar) + lr*tan(af))/(lf+lr));
    turn_v = ((veh_v * cos(F)) / (lf + lr)) * (tan(af) - tan(ar));
    R = (cos(ar) * lr) / sin(F - ar);
    ox = -(R*sin(F));
    oy = R*cos(F);
    ROS_INFO_STREAM("oy:" << oy << ",ox:" << ox << ",R:" << R);
    R1 = sqrt(pow((length_car/2 - ox),2) + pow((-weight/2 - oy),2));
    R2 = sqrt(pow((length_car/2 - ox),2) + pow((weight/2 - oy),2));
    R3 = sqrt(pow((-length_car/2 - ox),2) + pow((-weight/2 - oy),2));
    R4 = sqrt(pow((-length_car/2 - ox),2) + pow((weight/2 - oy),2));
    ROS_INFO_STREAM("R1:" << R1 << ",R2:" << R2 << ",R3:" << R3 << ",R4:" << R4);
    if(veh_v > 0)
    {
        float temp_y = 0;
        // if(ox > (-length_car/2))
        // {
            ROS_INFO_STREAM("o3x:" << -length_car/2 << ",o3y:" << -sqrt(pow(R3,2) - pow((-length_car/2 - ox),2)) + oy);
            for(vector<point>::iterator ite_v_ultrantic = v_ultrantic_left.begin();ite_v_ultrantic != v_ultrantic_left.end();++ite_v_ultrantic)
            {
                // temp_y = -sqrt(pow(R1,2) - pow((7735 - ite_v_ultrantic->x),2)) - (2865/2);
                temp_y = min((-sqrt(pow(R3,2) - pow((ite_v_ultrantic->x - ox),2)) + oy),(sqrt(pow(R3,2) - pow((ite_v_ultrantic->x - ox),2)) + oy));
                // ROS_INFO("temp_y:%f",temp_y);
                if(ite_v_ultrantic->y > temp_y)
                {
                    point_ = *ite_v_ultrantic;
                    // ROS_INFO_STREAM("ult_index:" << point_.index << ",ult_x:" << point_.x << ",ult_y:" << point_.y);
                    point_.safe_distance = fabs(ite_v_ultrantic->y - temp_y);
                    v_ultrantic_regional.push_back(point_);
                    // ROS_INFO_STREAM("ult_index:" << ite_v_ultrantic->index << ",ult_x:" << ite_v_ultrantic->x << ",ult_y:" << ite_v_ultrantic->y);
                }
            }
            for(vector<point>::iterator ite_v_ultrantic = v_ultrantic_right.begin();ite_v_ultrantic != v_ultrantic_right.end();++ite_v_ultrantic)
            {
                // temp_y = -sqrt(pow(R1,2) - pow((7735 - ite_v_ultrantic->x),2)) - (2865/2);
                temp_y = min((-sqrt(pow(R4,2) - pow((ite_v_ultrantic->x - ox),2)) + oy),(sqrt(pow(R4,2) - pow((ite_v_ultrantic->x - ox),2)) + oy));
                // ROS_INFO("temp_y:%f",temp_y);
                if(ite_v_ultrantic->y < temp_y)
                {
                    point_ = *ite_v_ultrantic;
                    // ROS_INFO_STREAM("ult_index:" << point_.index << ",ult_x:" << point_.x << ",ult_y:" << point_.y);
                    point_.safe_distance = fabs(ite_v_ultrantic->y - temp_y);
                    v_ultrantic_regional.push_back(point_);
                    // ROS_INFO_STREAM("ult_index:" << ite_v_ultrantic->index << ",ult_x:" << ite_v_ultrantic->x << ",ult_y:" << ite_v_ultrantic->y);
                }
            }
        // }
    }
    else if(veh_v < 0)
    {
        float temp_y = 0;
        // if(ox > (-length_car/2))
        // {
            ROS_INFO_STREAM("o3x:" << -length_car/2 << ",o3y:" << -sqrt(pow(R3,2) - pow((-length_car/2 - ox),2)) + oy);
            for(vector<point>::iterator ite_v_ultrantic = v_ultrantic_left.begin();ite_v_ultrantic != v_ultrantic_left.end();++ite_v_ultrantic)
            {
                // temp_y = -sqrt(pow(R1,2) - pow((7735 - ite_v_ultrantic->x),2)) - (2865/2);
                temp_y = min((-sqrt(pow(R1,2) - pow((ite_v_ultrantic->x - ox),2)) + oy),(sqrt(pow(R3,2) - pow((ite_v_ultrantic->x - ox),2)) + oy));
                // ROS_INFO("temp_y:%f",temp_y);
                if(ite_v_ultrantic->y > temp_y)
                {
                    point_ = *ite_v_ultrantic;
                    point_.safe_distance = fabs(ite_v_ultrantic->y - temp_y);
                    v_ultrantic_regional.push_back(point_);
                    // ROS_INFO_STREAM("ult_index:" << ite_v_ultrantic->index << ",ult_x:" << ite_v_ultrantic->x << ",ult_y:" << ite_v_ultrantic->y);
                }
            }
            for(vector<point>::iterator ite_v_ultrantic = v_ultrantic_right.begin();ite_v_ultrantic != v_ultrantic_right.end();++ite_v_ultrantic)
            {
                // temp_y = -sqrt(pow(R1,2) - pow((7735 - ite_v_ultrantic->x),2)) - (2865/2);
                temp_y = min((-sqrt(pow(R2,2) - pow((ite_v_ultrantic->x - ox),2)) + oy),(sqrt(pow(R4,2) - pow((ite_v_ultrantic->x - ox),2)) + oy));
                // ROS_INFO("temp_y:%f",temp_y);
                if(ite_v_ultrantic->y < temp_y)
                {
                    point_ = *ite_v_ultrantic;
                    point_.safe_distance = fabs(ite_v_ultrantic->y - temp_y);
                    v_ultrantic_regional.push_back(point_);
                    // ROS_INFO_STREAM("ult_index:" << ite_v_ultrantic->index << ",ult_x:" << ite_v_ultrantic->x << ",ult_y:" << ite_v_ultrantic->y);
                }
            }
    }
    // ROS_INFO_STREAM("length:" << v_ultrantic_regional.size());
    if(v_ultrantic_regional.size())
    {
        for(vector<point>::iterator ite_v_ultrantic = v_ultrantic_regional.begin();ite_v_ultrantic != v_ultrantic_regional.end();++ite_v_ultrantic)
        {
            ROS_INFO_STREAM("ult_index:" << ite_v_ultrantic->index << ",ult_x:" << ite_v_ultrantic->x << ",ult_y:" << ite_v_ultrantic->y << ",saf_dis:" << ite_v_ultrantic->safe_distance);
        }
    }
    // out_R = sqrt(pow(length_car/2 - ,2) + pow(R + (weight / 2),2));
    // max_out = out_R - R;
    // ROS_INFO_STREAM("R:" << R << ",out_R:" << out_R << ",max_out:" << max_out);
    ROS_INFO_STREAM("FA:" << (F*180/pi) << ",FR:" << F << ",turn_va:" << (turn_v * 180 /pi) << ",turn_vr:" << turn_v);
    while(ros::ok())
    {
        sleep(1);
    }
    return 0;

    // ros::Publisher agv_info_pub = nh.advertise< control_msgs::AGVStatus >("/drivers/com2agv/agv_status", 10, true);
    // ros::Publisher ult_info_pub = nh.advertise< perception_sensor_msgs::UltrasonicInfo >("/drivers/can_wr/sonser_info", 10, true);

    // control_msgs::AGVStatus agv_msg;
    // perception_sensor_msgs::UltrasonicInfo ult_msg;
    // common_msgs::UltrasonicPoint ult_point_msg;
    // agv_msg.header.stamp = ros::Time::now();
    // agv_msg.VEHMode = 2;
    // agv_msg.Dir_PRND = 1;
    // agv_msg.ActualAgl_R = atoi(argv[2]);
    // agv_msg.ActualAgl_F = atoi(argv[3]);

    // ult_msg.header.stamp = ros::Time::now();
    // ult_point_msg.id = atoi(argv[1]);
    // ult_point_msg.distance = 0.5;
    // ult_point_msg.status = true;
    // ult_msg.ult_obstacle.push_back(ult_point_msg);
    
    // while(ros::ok())
    // {
    //     agv_info_pub.publish(agv_msg);
    //     ult_info_pub.publish(ult_msg);
    //     sleep(1);
    // }

    // cout << 210%100/10 << endl;

    int row = MAP_MAX_X, col = MAP_MAX_Y;
    printf("hello world!\n");
    LNode **map = Translate_array(arr,row, col); //这里将数组的地图转为节点map的地图
    output(map,10,10);
    LinkList open_List = InitList();     //定义并初始化一个开放列表
    LinkList close_List = InitList();    //一个封闭列表
    LNode* startLNode = find_start_LNode(map, row, col);
    LNode* endLNode = find_end_LNode(map, row, col);

    LNode* curLNode = startLNode;        //当前节点=开始节点
    curLNode->G = 0;        //计算节点的三个值
    count_LNode_H(curLNode, endLNode);
    count_LNode_F(curLNode);
    push_OpenList_Node(open_List, curLNode);        //先将开始节点插入开放列表
    while (curLNode->data != 3)
    {
        printf("~~~~~~~~~~~~~~~~\n");
        //LNode *e = NULL;
        curLNode = pop_OpenList_minNode(open_List);
        printfnode(curLNode);
        insert_Into_CloseList(curLNode, close_List);
        //2、查看起点周围的点是否在开放列表里，不在加入，在检测经过该点F值是否最小等；
        check_around_curNode(curLNode, endLNode, open_List, map);
    }
    while (endLNode->path_next)
    {
        printf("x:%d---y:%d\n", endLNode->path_next->x,endLNode->path_next->y);
        endLNode->path_next = endLNode->path_next->path_next;
    }
    return 0;

    // event_init();

    // struct event evTimer;
    // evtimer_set(&evTimer, OnTime, &evTimer);

    // struct timeval tv;
    // tv.tv_sec = 1;
    // tv.tv_usec = 0;

    // event_add(&evTimer, &tv);

    // event_dispatch();
    // ROS_INFO("loop over");

    ostringstream system_out;
    string system_out_str = "";
    // int str_cmd = system("ps -ef | grep adcuServer");
    // system_out << system("pgrep can_wr_n");
    // system_out >> system_out_str;
    // cout << "result:" << system_out.str() << endl;
    // cout << WIFEXITED(str_cmd) << endl;
    // cout << WIFSIGNALED(str_cmd) << endl;

    ostringstream date1;
    date1 << to_string(2019) << setw(2) << setfill('0') << "-"  << to_string(2) << setw(2) << setfill('0');
    cout << date1.str() << endl;

    while(ros::ok())
    {
        ROS_INFO("!");
        FILE* fp = NULL;
        char buf[1024] = {0};
        string strResult = "";
        vector<string> v_test;
        // fp = popen("pgrep adcuServer", "r");
        fp = popen("df /home", "r");
        // system_out << popen("df /home", "r");
        if(NULL == fp)
        {
            perror("popen");
            exit(1);
        }
        // fgets(buf, 1024, fp);
        while(fgets(buf, 1024, fp))	
        {		
            strResult += buf;	
        }
        ROS_INFO("$");
        SplitString(strResult,v_test," ");
        if(2 <= (v_test.size()+1)/6)
        {
            ROS_INFO("@");
            cout << "break" << endl;
            break;
        }
        ROS_INFO("@");
        cout << "loop" << endl;
        sleep(1);
        // cout << (v_test.size()+1)/6 << endl;
    // for(vector<string>::iterator ite_v = v_test.begin();ite_v != v_test.end();++ite_v)
    // {
    //     cout << *ite_v << endl;
    // }
    }
    ROS_INFO("@");
    cout << "out the loop" << endl;
    while(ros::ok())
    {
        sleep(1);    
    }
    // system_out_str = buf;
    // cout << "system_out_str:" << system_out_str << endl;
    // while(fgets(buf, 1024, fp) != NULL)
    // {
    //     fprintf(stdout, "%s", buf); 
    //     printf("result:%s",buf);
    // }

    // pclose(fp);

    // adcuDeviceType deviceType;
    // deviceType = adcuCAN;
    // int channel;
    // int devid = 1;
    // channel                 = CHANNEL_P2;
    // devid                   = adcuDevOpen(deviceType, ( adcuDeviceChannel )channel);
    // cout << "devid:" << devid << endl;
    // if (adcuDevStatus(devid) == ADCU_DEV_STATUS_ABNORMAL)
    // {
    //   ROS_INFO("Can %d error", channel);
    //   // data_Output.Closefile(logname,logname);
    // //   break;
    // }

    double ang = -90;
    double rad = angle_to_radian(ang,0,0);

    uint16_t abc;

    MatrixXd sensor(1,3);
    MatrixXd line(3,3);
    MatrixXd angle(3,3);
    MatrixXd change(3,3);
    MatrixXd car(1,3);
    sensor << 0.1,0.1,1;
    line << 1,0,0,
            0,1,0,
            -0.5,1.5,1;
    angle << sin(rad),cos(rad),0,
             cos(rad),-sin(rad),0,
             0,0,1;
    change = line * angle;
    cout << angle << endl;
    cout << endl;
    cout << change << endl;
    car = sensor * change;
    // car = sensor * line * angle;
    cout << car << endl;
    abc = (0xFF & 0x3F) * 5;
    cout << "aaa:" << abc << endl;
    ROS_INFO("aaa:%d",abc);
    int u_id = atoi(argv[2]);
    cout << u_id << endl;
    float dist = 0.0;
    uint8_t can_buf[1] = {0xFF};
    dist = (can_buf[0] & 0x3F) * 0.05;
    
    timeval p;

	gettimeofday(&p, NULL);
	printf("currentTime:%d\n", (int)p.tv_sec);

	p.tv_sec = p.tv_sec + 60 * 5;    //系统时间增加五分钟
	printf("settimeofday(&p, NULL):%d",settimeofday(&p, NULL));

	gettimeofday(&p, NULL);
	printf("currentTime:%d\n", (int)p.tv_sec);


    ROS_INFO("dist:%f",dist);

    string time_ = "";
    string date_ = "";
    // string cmd_ = "echo 123456 | sudo date -s ";
    string cmd_ = "sudo date -s ";
    string str_1 = "123456.00";
    string str_2 = "050819";
    time_ = str_1.substr(0,2) + ":" + str_1.substr(2,2) + ":" + str_1.substr(4,2);
    date_ = str_2.substr(4,2) + "-" + str_2.substr(2,2) + "-" + str_2.substr(0,2);
    time_ = cmd_ + time_;
    date_ = cmd_ + date_;
    cout << str_1.substr(0,2) << "," << str_1.substr(2,2) << "," << str_1.substr(4,2) << endl;
    cout << str_2.substr(4,2) << "," << str_2.substr(2,2) << "," << str_2.substr(0,2) << endl;
    // getCmdResult("echo 123456");
    ROS_INFO("A");
    cout << time_ << endl;
    // system(time_.c_str());
    // getCmdResult(time_);
    ROS_INFO("B");
    // getCmdResult("echo 123456");
    ROS_INFO("C");
    cout << date_ << endl;
    // system(date_.c_str());
    // getCmdResult(date_);
    ROS_INFO("D");
    // getCmdResult("echo 123456");
    ROS_INFO("E");
    // m(0,0) = 1;
    // m(1,0) = 1;
    // m(0,1) = 0;
    // m(1,1) = 0;
    // m2(0,0) = 1;
    // m2(1,0) = 3;
    // m2(0,1) = 2;
    // m2(1,1) = 4;
    // m3 = m * m2;
    // cout << m << endl;
    // cout << m2 << endl;
    // cout << m3 << endl;
    int a = 0xFFFF107E;
    // ROS_INFO_STREAM("A:" << sin(angle_to_radian(ang,0,0)));
//    ros::Time begin = ros::Time::now();
    // uint8_t data[8] = {0x00,0x04,0x8F,0xFF,0xDA,0x02,0x71,0x90};
    // uint8_t data_x[4] = {0xFF,0xFF,0xFF,0xDA};
    // uint8_t data_y[4] = {0xDA,0xFF,0xFF,0xFF};
    // uint8_t data_z[4] = {0xFF,0xFF,0xFF,0xDA};
    // int y = 0;
    // int x = 0;
    // int z = 0;
    // if(((data[0]) & 0xF0) >> 7)
    // {
    //     x = (0xFFF << 20) | (data[0]) << 12 | (data[1]) << 4 | ((data[2]) >> 4 & 0x0F);
    // }
    // else
    // {
    //     x = (data[0]) << 12 | (data[1]) << 4 |nav_msgs::Odometry ((data[2]) >> 4 & 0x0F);
    // }
    // if(((data[2]) & 0x0F) >> 3)
    // {
    //     y = (0xFFF << 20 | ((data[2]) & 0x0F) << 16 | (data[3]) << 8 | (data[4]));
    // }
    // else
    // {
    //     y = ((data[2]) & 0x0F) << 16 | (data[3]) << 8 | (data[4]);
    // }
    // if(((data[5]) & 0xF0) >> 7)
    // {
    //     z = (0xFFF << 20) | (data[5]) << 12 | (data[6]) << 4 | ((data[7]) >> 4 & 0x0F);
    // }
    // else
    // {
    //     z = (data[5]) << 12 | (data[6]) << 4 | ((data[7]) >> 4 & 0x0F);
    // }

    // ROS_INFO_STREAM("X:" << x <<",y:" << y << ",z:" << z << "," << sizeof(a));

    // ros::Publisher node_state_pub = nh.advertise<power_control_msgs::PowerControlCmd>("/power_control/power_control_cmd", 1000, true);
    // ros::Subscriber node_state_sub = nh.subscribe<perception_sensor_msgs::UltrasonicInfo>("/drivers/can_wr/sonser_info", 1000, boost::bind(&power_control_callback,_1,u_id));
        // ros::Publisher markerPub = nh.advertise<visualization_msgs::Marker>("TEXT_VIEW_FACING", 10);
        ros::Subscriber obstacle_sub        = nh.subscribe("/prescan/obstacle_location", 10, recvObstacleCallback);
        obstacle_cam_lidar_pub = nh.advertise< visualization_msgs::MarkerArray >("/perception/rviz_obstacle_cam_lidar_msg", 10, true);

        visualization_msgs::Marker marker;
        marker.header.frame_id="/odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id =0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        marker.scale.z = 0.2;
        marker.color.b = 0;
        marker.color.g = 0;
        marker.color.r = 255;
        marker.color.a = 1;

        ros::Rate r(1);
        int k = 0;
    ROS_INFO("!");
    // time_t itime = 767441115;
    int week = 2066;
    double dsec = 99835.111;
    time_t itime = week * 604800 + dsec + 432000 - 18;
    // time_t itime = week * 604800 + dsec;
    double te = 1.333;
    string str_te = to_string(te);
    vector<string> v_te;
    SplitString(str_te,v_te,".");
    cout << "v_te[2]:" << v_te[1] << endl;
    struct tm *timeinfo;
    string date_str = "";
    string sdate = "";
    string stime = "";
    string cmd = "sudo date -s ";
    vector<string> v;
    // time(&itime);
    // timeinfo = localtime(&itime);
    // date_str = asctime(timeinfo);
    date_str = ctime(&itime);
    cout << date_str << endl;
    SplitString(date_str,v," ");
    
    // v[4] = to_string(atoi((v[4].substr(0,((v[4].size()) - 1))).c_str()) + 10);
    v[4] = to_string(atoi((v[4].substr(2,((v[4].size()) - 1))).c_str()) + 10);
    sdate = cmd + v[4] + "-" + monthstr2num(v[1]) + "-" + daystr2num(v[2]);
    stime = cmd + v[3];
    // system(sdate.c_str());
    // system(stime.c_str());
    cout << "sdate:" << sdate << endl;
    cout << "stime:" << stime << endl;
    int pweek = 0;
    double psecweek = 0;
    UTC2GPS(2019,8,12,3,43,37,&pweek,&psecweek);
    cout << "week:" << pweek << ",sec:" << psecweek << endl;
    ROS_INFO("!");
    // time_t itime = 0;
    // cout << "ctime(&itime):" << ctime(&itime) << endl;
    

    // que que_;
    
    // thread t_1_send(std::bind(&que::t1, &que_));
    // t_1_send.detach();
    // thread t_2_send(std::bind(&que::t2, &que_));
    // t_2_send.detach();
    // thread t_3_send(std::bind(&que::t3, &que_));
    // t_3_send.detach();
    // thread t_4_send(std::bind(&que::t4, &que_));
    // t_4_send.detach();
    // thread t_5_send(std::bind(&que::t5, &que_));
    // t_5_send.detach();

    // nav_msgs::GridCells cells;
    // cells.header.frame_id="map";
    // cells.cell_height=0.3;
    // cells.cell_width=0.3;
    // cells.cells.resize(1);
    // // cells.cells[0].x=1;
    // // cells.cells[0].y=1;
    // // cells.cells[0].z=0;
    
    // float x = 0.0;
    // ros::Publisher pub = nh.advertise<nav_msgs::GridCells>("/cells", 1);

    while(ros::ok())
    {
        ROS_INFO("~");
        usleep(1000 * 100);
        // geometry_msgs::Pose pose;
        // pose.position.x =  (float)(k++)/10;
        // pose.position.y =  0;
        // pose.position.z =0;
        // ostringstream str;
        // str << "k:" << k;
        // marker.text=str.str();
        // marker.pose=pose;
        // markerPub.publish(marker);
        // cout<<"k="<<k<<endl;
        // r.sleep();
        
        // cells.cells[0].x=x;
        // cells.cells[0].y=1;
        // cells.cells[0].z=0;
        // pub.publish(cells);
        // x = x + 0.1;
        // sleep(2);
        // ros::spin();
        // power_control_msgs::PowerControlCmd PowerControlCmd_msg;
        // PowerControlCmd_msg.length = 20;
        // cout << "~~~~~~~~~~~~~~~~~~~~~~" << endl;
        // for(int i = 0;i < 20; ++i)
        // {
        //     // if(15 == i)
        //     // {
        //     //     PowerControlCmd_msg.data.push_back(1);
        //     // }
        //     // else
        //     // {
        //     //     PowerControlCmd_msg.data.push_back(0);
        //     // }
        //     PowerControlCmd_msg.data.push_back(atoi(argv[i+1]));
        //     // cout << atoi(argv[i+1]) << endl;
        // }
        
        // node_state_pub.publish(PowerControlCmd_msg);
        // sleep(1);
    }

/*
    threadpool pool(8);

    std::future<void> ff = pool.commit(fun1,0);
    ff.get();

    while(ros::ok())
    {
        ROS_INFO("main sleep 1s");
        ROS_INFO_STREAM(pool.thrCount() << "," << pool.idlCount());
        sleep(1);
        // std::this_thread::sleep_for(std::chrono::seconds(3));
    }
*/


    // ThreadPool pool(4);
    // std::vector< std::future<int> > results;

    // for(int i = 0; i < 8; ++i) {perception_sensor_msgs::UltrasonicInfo::ConstPtr
    //     results.emplace_back(
    //       pool.enqueue([i] {
    //         std::cout << "hello " << i << std::endl;
    //         std::this_thread::sleep_for(std::chrono::seconds(1));
    //         std::cout << "world " << i << std::endl;
    //         return i*i;
    //     })
    //   );
    // }


    // try {
    //     std::t_pool executor{ 50 };
    //     A a;
    //     std::future<void> ff = executor.commit(fun1,0);
        // std::future<int> fg = executor.commit(gfun{},0);
        // // std::future<int> gg = executor.commit(a.Afun, 9999); //IDE提示错误,但可以编译运行
        // std::future<std::string> gh = executor.commit(A::Bfun, 9998,"mult args", 123);
        // std::future<std::string> fh = executor.commit([]()->std::string { std::cout << "hello, fh !  " << std::this_thread::get_id() << std::endl; return "hello,fh ret !"; });

        // std::cout << " =======  sleep ========= " << std::this_thread::get_id() << std::endl;
        // std::this_thread::sleep_for(std::chrono::microseconds(900));

        // for (int i = 0; i < 50; i++) {
        //     executor.commit(fun1,i*100 );
        // }
        // std::cout << " =======  commit all ========= " << std::this_thread::get_id()<< " idlsize="<<executor.idlCount() << std::endl;

        // std::cout << " =======  sleep ========= " << std::this_thread::get_id() << std::endl;
        // std::this_thread::sleep_for(std::chrono::seconds(3));

        // ff.get(); //调用.get()获取返回值会等待线程执行完,获取返回值
        // std::cout << fg.get() << "  " << fh.get().c_str()<< "  " << std::this_thread::get_id() << std::endl;

        // std::cout << " =======  sleep ========= " << std::this_thread::get_id() << std::endl;
        // std::this_thread::sleep_for(std::chrono::seconds(3));

        // std::cout << " =======  fun1,55 ========= " << std::this_thread::get_id() << std::endl;
        // executor.commit(fun1,55).get();    //调用.get()获取返回值会等待线程执行完

        // std::cout << "end... " << std::this_thread::get_id() << std::endl;


        // std::threadpool pool(4);
        // std::vector< std::future<int> > results;

        // for (int i = 0; i < 8; ++i) {
        //     results.emplace_back(
        //         pool.commit([i] {
        //             std::cout << "hello " << i << std::endl;
        //             std::this_thread::sleep_for(std::chrono::seconds(1));
        //             std::cout << "world " << i << std::endl;
        //             return i*i;
        //         })
        //     );
        // }
        // std::cout << " =======  commit all2 ========= " << std::this_thread::get_id() << std::endl;

        // for (auto && result : results)
        //     std::cout << result.get() << ' ';
        // std::cout << std::endl;
//         return 0;
//     }
// catch (std::exception& e) {
//     std::cout << "some unhappy happened...  " << std::this_thread::get_id() << e.what() << std::endl;}



    // for(auto && result: results)    //通过future.get()获取返回值
    //     std::cout << result.get() << ' ';
    // std::cout << std::endl;

    // while(ros::ok())
    // {
    //     // ros::spin();
    //     // can_log.write_log(&data[0],1);
    //     // LOG(INFO) << "glog 1s!!!";
    //     sleep(1);
    // }
    // ros::spin();
    return 0;
}
