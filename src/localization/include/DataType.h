#ifndef DATATYPE_H
#define DATATYPE_H

#include <ros/ros.h>
#include <string>
#include <time.h>

using namespace std;

struct UTC{
    uint32_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    double msec;
    UTC(){
        memset(this,0,sizeof(UTC));
    }
    //获取本地时间
    UTC UTCtoLocal(){
        time_t time_now;
        struct tm* local_time;          //从 1900.1.1 00:00 开始表示起
        struct timeval tv;
        memset(&tv,0,sizeof(tv));
        tzset(); //设置时间环境变量-时区
        time(&time_now);        //获得UTC秒数,从 1970.1.1 00:00 开始算起
        local_time = localtime(&time_now);  //将UTC时间转换为本地时间到秒级
        gettimeofday(&tv,NULL);
        year = local_time->tm_year + 1900;
        month = local_time->tm_mon + 1;
        day = local_time->tm_mday;
        hour = local_time->tm_hour;
        min = local_time->tm_min;
        sec = local_time->tm_sec;
        msec = tv.tv_usec/1000;
        return *this;
    }
};

struct  LLH_POS
{
	double  lan,lon,h;
	LLH_POS(){
		memset(this,0.0,sizeof(LLH_POS));
	}
};

struct  ECEF_POS
{
	double  x,y,z;
	ECEF_POS(){
		memset(this,0.0,sizeof(ECEF_POS));
	}
};

struct Linear      //机体坐标系下角速度
{
    double vx,vy,vz;
    Linear(){
        memset(this,0.0,sizeof(Linear));
    }
};

struct Angular      //机体坐标系下角速度
{
    double wx,wy,wz;
    Angular(){
        memset(this,0.0,sizeof(Angular));
    }
};

struct VEL
{
	Linear venu;  
    Angular wxyz;
	VEL(){
		memset(this,0.0,sizeof(VEL));
	}
};

//line_heading车道线的方向角
struct Atitude
{
	float yaw,pitch,roll,line_heading;
	Atitude(){
		memset(this,0.0,sizeof(Atitude));
	}
};

struct Accel
{
    float ax,ay,az;
    Accel(){
        memset(this,0.0,sizeof(Accel));
    }
};

//场端lidar停位点的港区坐标（港区全局坐标系,单位m,xyz东北天)
struct Flidar_tager_position
{
    double tpx,tpy,tpz;
    Flidar_tager_position(){
        memset(this,0.0,sizeof(Flidar_tager_position));
    }
};

//场端lidar:AGV距离停位点横向、纵向距离(相对坐标，单位 m，负值时为越过停止位的距离) 
struct AGV_distance
{
    double agv_x,agv_y;
    AGV_distance(){
        memset(this,0.0,sizeof(AGV_distance));
    }
};

//场端lidar:横向（垂直车道线）速度 ,纵向（平行车道线）速度
struct Flidar_vel
{
    double Vx,Vy;
    Flidar_vel(){
        memset(this,0.0,sizeof(Flidar_vel));
    }
};


struct PoseResult
{
    Atitude att;      
    VEL     vel;   
    LLH_POS  pos;
    Accel    accel;  
    Flidar_tager_position fl_tager_pos;
    AGV_distance agv_dist; 
    Flidar_vel fl_vel;
    // PoseResult(){
    //     memset(this,0.0,sizeof(PoseResult));
    // }
};

#endif