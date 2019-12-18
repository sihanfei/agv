#include "MatchForTime.h"
#include "CoordinateSystem.h"
#include "FusionCenter.h"
#include <eigen3/Eigen/Core>


#define Ta 0.01     //组合导航采样时间间隔100Hz为0.01s

MatchForTime::MatchForTime(FusionCenter *pFsCenter)
{
    g_pfscenter = pFsCenter;
}

MatchForTime::~MatchForTime(void)
{

}


// **************
// 功能:线性运动下,对高频传感器的84坐标系下速度,位置向低频配准
// 输入:UTM坐标系下位置,速度,中央子午线经度
// 输出:配准后的经纬高,东北天下的速度
// 无返回
// ***************
void MatchForTime::MatchForLinear(vector<double*>& pose_match_utm, vector<double*>& vel_match_utm, 
                                  PoseResult &high_freq_match,double &lon0)
{
    UTC hf_time;    //高频传感器时间
    UTC lf_time;    //低频传感器时间
    setTime(hf_time,lf_time);
    double  high_freq_LLH_match[3] = {0.0,0.0,0.0};
    //double  high_freq_VENU_match[3] = {0.0,0.0,0.0};
    if(hf_time.hour == lf_time.hour)
    {
        if(hf_time.min == lf_time.min)
        {
            if(hf_time.sec >= lf_time.sec)
            {
                if(hf_time.msec > lf_time.msec)
                {
                    float delta_a[3];  //加速度变化率xyz
                    float accel_[3];      //加速度
                    double high_freq_utm_pose_match[3];   //匹配后的坐标，84系统UTM坐标
                    double high_freq_utm_vel_match[3];  //匹配后的速度，84系统UTM坐标
                    float delta_t = 0.0;    //配准时间间隔
                    delta_t = (lf_time.msec - (hf_time.msec - Ta*1000))/1000;
                    for(int i=0; i < 3; i++)
                    {
                        delta_a[i] = 2*(pose_match_utm[2][i] - 2*pose_match_utm[1][i] + pose_match_utm[0][i] 
                        - 
                        vel_match_utm[1][i]*Ta + vel_match_utm[0][i]*Ta)/pow(Ta,3);
                        accel_[i] = 2*(pose_match_utm[1][i] - pose_match_utm[0][i] - vel_match_utm[0][i]*Ta 
                        - 
                        1/6*delta_a[i]*pow(Ta,3))/pow(Ta,2);
                        high_freq_utm_pose_match[i] = pose_match_utm[1][i] + vel_match_utm[1][i]*delta_t 
                        + 
                        1/2*accel_[i]*pow(delta_t,2) + 1/6*delta_a[i]*pow(delta_t,3);
                        high_freq_utm_vel_match[i] = vel_match_utm[1][i] + accel_[i]*delta_t + 1/2*delta_a[i]*pow(delta_t,2);
                    }
                    transForUTMtoLLH(high_freq_utm_pose_match,high_freq_LLH_match,lon0);
                    //memcpy(high_freq_VENU_match,high_freq_utm_vel_match,sizeof(double)*3);
                    high_freq_match.pos.lan = high_freq_LLH_match[0];
                    high_freq_match.pos.lon = high_freq_LLH_match[1];
                    high_freq_match.pos.h = high_freq_LLH_match[2];
                    high_freq_match.vel.venu.vx = high_freq_utm_vel_match[0];
                    high_freq_match.vel.venu.vy = high_freq_utm_vel_match[1];
                    high_freq_match.vel.venu.vz = high_freq_utm_vel_match[2];
                    // high_freq_match.vel.venu.vx = high_freq_VENU_match[0];
                    // high_freq_match.vel.venu.vy = high_freq_VENU_match[1];
                    // high_freq_match.vel.venu.vz = high_freq_VENU_match[2];
                    //根据传感器类型后续再加
                    if(g_pfscenter->high_freq_type == g_pfscenter->integrated_nav)
                    {
                        high_freq_match.att.yaw = g_pfscenter->g_record_fuse.yaw;
                        high_freq_match.att.pitch = g_pfscenter->g_record_fuse.pitch;
                        high_freq_match.att.roll = g_pfscenter->g_record_fuse.roll;
                        high_freq_match.vel.wxyz.wx = g_pfscenter->g_record_fuse.velocity.angular.x;     //机体坐标系角速度
                        high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
                        high_freq_match.vel.wxyz.wz = g_pfscenter->g_record_fuse.velocity.angular.z;
                        high_freq_match.accel.ax = g_pfscenter->g_record_fuse.accel.linear.x;   //加速度计采的机体加速度
                        high_freq_match.accel.ay = g_pfscenter->g_record_fuse.accel.linear.y;
                        high_freq_match.accel.az = g_pfscenter->g_record_fuse.accel.linear.z;
                    }

                }
                else if (hf_time.msec == lf_time.msec)
                {
                    //根据传感器类型后续再加
                    if(g_pfscenter->high_freq_type == g_pfscenter->integrated_nav){
                        high_freq_match.pos.lan = g_pfscenter->g_record_fuse.pose_llh.x;
                        high_freq_match.pos.lon = g_pfscenter->g_record_fuse.pose_llh.y;
                        high_freq_match.pos.h = g_pfscenter->g_record_fuse.pose_llh.z;
                        high_freq_match.vel.venu.vx = g_pfscenter->g_record_fuse.velocity.linear.x;
                        high_freq_match.vel.venu.vy = g_pfscenter->g_record_fuse.velocity.linear.y;
                        high_freq_match.vel.venu.vz = g_pfscenter->g_record_fuse.velocity.linear.z;
                        high_freq_match.vel.wxyz.wx = g_pfscenter->g_record_fuse.velocity.angular.x;     //机体坐标系角速度
                        high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
                        high_freq_match.vel.wxyz.wz = g_pfscenter->g_record_fuse.velocity.angular.z;
                        high_freq_match.att.yaw = g_pfscenter->g_record_fuse.yaw;
                        high_freq_match.att.pitch = g_pfscenter->g_record_fuse.pitch;
                        high_freq_match.att.roll = g_pfscenter->g_record_fuse.roll;
                        high_freq_match.accel.ax = g_pfscenter->g_record_fuse.accel.linear.x;   //加速度计采的机体加速度
                        high_freq_match.accel.ay = g_pfscenter->g_record_fuse.accel.linear.y;
                        high_freq_match.accel.az = g_pfscenter->g_record_fuse.accel.linear.z;
                    }
                }
                else
                {
                    float accel_[3];      //加速度
                    double high_freq_utm_pose_match[3];   //匹配后的坐标，84系统
                    double high_freq_utm_vel_match[3];  //匹配后的速度，84系统
                    float delta_t = 0.0;    //配准时间间隔
                    delta_t = lf_time.sec + lf_time.msec/1000 - (hf_time.sec + hf_time.msec/1000);
                    for(int i = 0; i < 3; i++)
                    {
                        accel_[i] = (pose_match_utm[2][i] - 2*pose_match_utm[1][i] +pose_match_utm[0][i])/pow(delta_t,2);
                        high_freq_utm_pose_match[i] = pose_match_utm[2][i] + vel_match_utm[2][i]*delta_t + 1/2*accel_[i]*pow(delta_t,2);
                        high_freq_utm_vel_match[i] = vel_match_utm[2][i] + accel_[i]*delta_t;
                    }
                    transForUTMtoLLH(high_freq_utm_pose_match,high_freq_LLH_match,lon0);
                    high_freq_match.pos.lan = high_freq_LLH_match[0];
                    high_freq_match.pos.lon = high_freq_LLH_match[1];
                    high_freq_match.pos.h = high_freq_LLH_match[2];
                    high_freq_match.vel.venu.vx = high_freq_utm_vel_match[0];
                    high_freq_match.vel.venu.vy = high_freq_utm_vel_match[1];
                    high_freq_match.vel.venu.vz = high_freq_utm_vel_match[2];
                    //根据传感器类型后续再加
                    if(g_pfscenter->high_freq_type == g_pfscenter->integrated_nav)
                    {
                        high_freq_match.att.yaw = g_pfscenter->g_record_fuse.yaw;
                        high_freq_match.att.pitch = g_pfscenter->g_record_fuse.pitch;
                        high_freq_match.att.roll = g_pfscenter->g_record_fuse.roll;
                        high_freq_match.vel.wxyz.wx = g_pfscenter->g_record_fuse.velocity.angular.x;
                        high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
                        high_freq_match.vel.wxyz.wz = g_pfscenter->g_record_fuse.velocity.angular.z;
                        high_freq_match.accel.ax = g_pfscenter->g_record_fuse.accel.linear.x;   //加速度计采的机体加速度
                        high_freq_match.accel.ay = g_pfscenter->g_record_fuse.accel.linear.y;
                        high_freq_match.accel.az = g_pfscenter->g_record_fuse.accel.linear.z;
                    }
                }
            }
            else
            {
                float accel_[3];      //加速度
                double high_freq_utm_pose_match[3];   //匹配后的坐标，84系统
                double high_freq_utm_vel_match[3];  //匹配后的速度，84系统
                float delta_t = 0.0;    //配准时间间隔
                delta_t = lf_time.sec + lf_time.msec/1000 - (hf_time.sec + hf_time.msec/1000);
                for(int i = 0; i < 3; i++)
                {
                    accel_[i] = (pose_match_utm[2][i] - 2*pose_match_utm[1][i] +pose_match_utm[0][i])/pow(delta_t,2);
                    high_freq_utm_pose_match[i] = pose_match_utm[2][i] + vel_match_utm[2][i]*delta_t + 1/2*accel_[i]*pow(delta_t,2);
                    high_freq_utm_vel_match[i] = vel_match_utm[2][i] + accel_[i]*delta_t;
                }
                transForUTMtoLLH(high_freq_utm_pose_match,high_freq_LLH_match,lon0);
                high_freq_match.pos.lan = high_freq_LLH_match[0];
                high_freq_match.pos.lon = high_freq_LLH_match[1];
                high_freq_match.pos.h = high_freq_LLH_match[2];
                high_freq_match.vel.venu.vx = high_freq_utm_vel_match[0];
                high_freq_match.vel.venu.vy = high_freq_utm_vel_match[1];
                high_freq_match.vel.venu.vz = high_freq_utm_vel_match[2];
                //根据传感器类型后续再加
                if(g_pfscenter->high_freq_type == g_pfscenter->integrated_nav)
                {
                    high_freq_match.att.yaw = g_pfscenter->g_record_fuse.yaw;
                    high_freq_match.att.pitch = g_pfscenter->g_record_fuse.pitch;
                    high_freq_match.att.roll = g_pfscenter->g_record_fuse.roll;
                    high_freq_match.vel.wxyz.wx = g_pfscenter->g_record_fuse.velocity.angular.x;
                    high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
                    high_freq_match.vel.wxyz.wz = g_pfscenter->g_record_fuse.velocity.angular.z;
                    high_freq_match.accel.ax = g_pfscenter->g_record_fuse.accel.linear.x;   //加速度计采的机体加速度
                    high_freq_match.accel.ay = g_pfscenter->g_record_fuse.accel.linear.y;
                    high_freq_match.accel.az = g_pfscenter->g_record_fuse.accel.linear.z;
                }
            } 
        }
        else
        {
            float accel_[3];      //加速度
            double high_freq_utm_pose_match[3];   //匹配后的坐标，84系统
            double high_freq_utm_vel_match[3];  //匹配后的速度，84系统
            float delta_t = 0.0;    //配准时间间隔
            delta_t = hf_time.min*60 + hf_time.sec + hf_time.msec/1000 - 
                      (lf_time.min*60 + lf_time.sec + lf_time.msec/1000);
            if(delta_t > 0)
            {
                delta_t = (lf_time.msec - (1000 + hf_time.msec - Ta))/1000;
                for(int i = 0; i < 3; i++)
                {
                    accel_[i] = (pose_match_utm[2][i] - 2*pose_match_utm[1][i] +pose_match_utm[0][i])/pow(delta_t,2);
                    high_freq_utm_pose_match[i] = pose_match_utm[1][i] + vel_match_utm[1][i]*delta_t + 1/2*accel_[i]*pow(delta_t,2);
                    high_freq_utm_vel_match[i] = vel_match_utm[1][i] + accel_[i]*delta_t;
                }
            }
            else
            {
                delta_t = (1000 + lf_time.msec - hf_time.msec)/1000;
                for(int i = 0; i < 3; i++)
                {
                    accel_[i] = (pose_match_utm[2][i] - 2*pose_match_utm[1][i] +pose_match_utm[0][i])/pow(delta_t,2);
                    high_freq_utm_pose_match[i] = pose_match_utm[2][i] + vel_match_utm[2][i]*delta_t + 1/2*accel_[i]*pow(delta_t,2);
                    high_freq_utm_vel_match[i] = vel_match_utm[2][i] + accel_[i]*delta_t;
                }
            }
            transForUTMtoLLH(high_freq_utm_pose_match,high_freq_LLH_match,lon0);
            high_freq_match.pos.lan = high_freq_LLH_match[0];
            high_freq_match.pos.lon = high_freq_LLH_match[1];
            high_freq_match.pos.h = high_freq_LLH_match[2];
            high_freq_match.vel.venu.vx = high_freq_utm_vel_match[0];
            high_freq_match.vel.venu.vy = high_freq_utm_vel_match[1];
            high_freq_match.vel.venu.vz = high_freq_utm_vel_match[2];
            //根据传感器类型后续再加
            if(g_pfscenter->high_freq_type == g_pfscenter->integrated_nav)
            {
                high_freq_match.att.yaw = g_pfscenter->g_record_fuse.yaw;
                high_freq_match.att.pitch = g_pfscenter->g_record_fuse.pitch;
                high_freq_match.att.roll = g_pfscenter->g_record_fuse.roll;
                high_freq_match.vel.wxyz.wx = g_pfscenter->g_record_fuse.velocity.angular.x;
                high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
                high_freq_match.vel.wxyz.wz = g_pfscenter->g_record_fuse.velocity.angular.z;
                high_freq_match.accel.ax = g_pfscenter->g_record_fuse.accel.linear.x;   //加速度计采的机体加速度
                high_freq_match.accel.ay = g_pfscenter->g_record_fuse.accel.linear.y;
                high_freq_match.accel.az = g_pfscenter->g_record_fuse.accel.linear.z;
            }
        }
    }
    else
    {
        float accel_[3];      //加速度
        double high_freq_utm_pose_match[3];   //匹配后的坐标，84系统
        double high_freq_utm_vel_match[3];  //匹配后的速度，84系统
        float delta_t = 0.0;    //配准时间间隔
        delta_t = hf_time.hour*3600 + hf_time.min*60 + hf_time.sec + hf_time.msec/1000 - 
                  (lf_time.hour*3600 + lf_time.min*60 + lf_time.sec + lf_time.msec/1000);
        if(delta_t > 0)
        {
            delta_t = (lf_time.msec - (1000 + hf_time.msec - Ta))/1000;
            for(int i = 0; i < 3; i++)
            {
                accel_[i] = (pose_match_utm[2][i] - 2*pose_match_utm[1][i] +pose_match_utm[0][i])/pow(delta_t,2);
                high_freq_utm_pose_match[i] = pose_match_utm[1][i] + vel_match_utm[1][i]*delta_t + 1/2*accel_[i]*pow(delta_t,2);
                high_freq_utm_vel_match[i] = vel_match_utm[1][i] + accel_[i]*delta_t;
            }
        }
        else
        {
            delta_t = (1000 + lf_time.msec - hf_time.msec)/1000;
            for(int i = 0; i < 3; i++)
            {
                accel_[i] = (pose_match_utm[2][i] - 2*pose_match_utm[1][i] +pose_match_utm[0][i])/pow(delta_t,2);
                high_freq_utm_pose_match[i] = pose_match_utm[2][i] + vel_match_utm[2][i]*delta_t + 1/2*accel_[i]*pow(delta_t,2);
                high_freq_utm_vel_match[i] = vel_match_utm[2][i] + accel_[i]*delta_t;
            }
        }
        transForUTMtoLLH(high_freq_utm_pose_match,high_freq_LLH_match,lon0);
        high_freq_match.pos.lan = high_freq_LLH_match[0];
        high_freq_match.pos.lon = high_freq_LLH_match[1];
        high_freq_match.pos.h = high_freq_LLH_match[2];
        high_freq_match.vel.venu.vx = high_freq_utm_vel_match[0];
        high_freq_match.vel.venu.vy = high_freq_utm_vel_match[1];
        high_freq_match.vel.venu.vz = high_freq_utm_vel_match[2];
        //根据传感器类型后续再加
        if(g_pfscenter->high_freq_type == g_pfscenter->integrated_nav)
        {
            high_freq_match.att.yaw = g_pfscenter->g_record_fuse.yaw;
            high_freq_match.att.pitch = g_pfscenter->g_record_fuse.pitch;
            high_freq_match.att.roll = g_pfscenter->g_record_fuse.roll;
            high_freq_match.vel.wxyz.wx = g_pfscenter->g_record_fuse.velocity.angular.x;
            high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
            high_freq_match.vel.wxyz.wz = g_pfscenter->g_record_fuse.velocity.angular.z;
            high_freq_match.accel.ax = g_pfscenter->g_record_fuse.accel.linear.x;   //加速度计采的机体加速度
            high_freq_match.accel.ay = g_pfscenter->g_record_fuse.accel.linear.y;
            high_freq_match.accel.az = g_pfscenter->g_record_fuse.accel.linear.z;
        }  
    }
}


void MatchForTime::MatchForStop(PoseResult &high_freq_match)
{
    high_freq_match.pos.lan = g_pfscenter->g_record_fuse.pose_llh.x;
    high_freq_match.pos.lon = g_pfscenter->g_record_fuse.pose_llh.y;
    high_freq_match.pos.h = g_pfscenter->g_record_fuse.pose_llh.z;
    high_freq_match.att.yaw = g_pfscenter->g_record_fuse.yaw;
    high_freq_match.att.pitch = g_pfscenter->g_record_fuse.pitch;
    high_freq_match.att.roll = g_pfscenter->g_record_fuse.roll;
    high_freq_match.vel.venu.vx= g_pfscenter->g_record_fuse.velocity.linear.x;
    high_freq_match.vel.venu.vy= g_pfscenter->g_record_fuse.velocity.linear.y;
    high_freq_match.vel.venu.vz= g_pfscenter->g_record_fuse.velocity.linear.z;
    high_freq_match.vel.wxyz.wx = g_pfscenter->g_record_fuse.velocity.angular.x;
    high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
    high_freq_match.vel.wxyz.wz = g_pfscenter->g_record_fuse.velocity.angular.z;
    high_freq_match.accel.ax = g_pfscenter->g_record_fuse.accel.linear.x;   //加速度计采的机体加速度
    high_freq_match.accel.ay = g_pfscenter->g_record_fuse.accel.linear.y;
    high_freq_match.accel.az = g_pfscenter->g_record_fuse.accel.linear.z;
}


// **************
// 功能:匀速圆周运动下,对高频传感器的84坐标系下速度,位置向低频配准
// 输入:84坐标系下位置,速度
// 输出:配准后的经纬高,东北天下的速度
// 无返回
// ***************
void MatchForTime::MatchForCircular(vector<double*>& pose_match_utm, vector<double*> vel_match_utm, 
                                     PoseResult &high_freq_match,double &lon0)
{
    UTC hf_time;    //高频传感器时间
    UTC lf_time;    //低频传感器时间
    setTime(hf_time,lf_time);
    double high_freq_LLH_match[3] = {0.0};
    //double high_freq_VENU_match[3] = {0.0};
    if(hf_time.hour == lf_time.hour)
    {
        if(hf_time.min == lf_time.min)
        {
            if(hf_time.sec >= lf_time.sec)
            {
                if(hf_time.msec > lf_time.msec)
                {
                    Vector3d B;
                    B = Vector3d::Zero();
                    Matrix3d A;
                    A = Matrix3d::Zero();
                    Vector3d X;
                    X = Vector3d::Zero();
                    double circle[4];
                    CalculateCircular(pose_match_utm,A,B,X,circle);
                    float delta_t = 0.0;    //配准时间间隔
                    delta_t = (lf_time.msec - (hf_time.msec - Ta))/1000;
                    double delta_sita = circle[3]*delta_t; 
                    double high_freq_utm_pose_match[3];   //匹配后的坐标，84系统
                    high_freq_utm_pose_match[0] = circle[0] + circle[2]*cos(acos((pose_match_utm[1][0] - circle[0])/circle[2]) + delta_sita);
                    high_freq_utm_pose_match[1] = circle[1] + circle[2]*sin(acos((pose_match_utm[1][0] - circle[0])/circle[2]) + delta_sita);
                    high_freq_utm_pose_match[2] = pose_match_utm[1][2];
                    //根据传感器类型后续再加
                    if(g_pfscenter->high_freq_type == g_pfscenter->integrated_nav)
                    {
                        high_freq_match.vel.venu.vx = g_pfscenter->g_record_fuse.velocity.linear.x;
                        high_freq_match.vel.venu.vy = g_pfscenter->g_record_fuse.velocity.linear.y;
                        high_freq_match.vel.venu.vz = g_pfscenter->g_record_fuse.velocity.linear.z;
                        high_freq_match.vel.wxyz.wx = g_pfscenter->g_record_fuse.velocity.angular.x;
                        high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
                        high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
                        high_freq_match.att.yaw = g_pfscenter->g_record_fuse.yaw;
                        high_freq_match.att.pitch = g_pfscenter->g_record_fuse.pitch;
                        high_freq_match.att.roll = g_pfscenter->g_record_fuse.roll;
                        high_freq_match.accel.ax = g_pfscenter->g_record_fuse.accel.linear.x;   //加速度计采的机体加速度
                        high_freq_match.accel.ay = g_pfscenter->g_record_fuse.accel.linear.y;
                        high_freq_match.accel.az = g_pfscenter->g_record_fuse.accel.linear.z;
                    } 
                    transForUTMtoLLH(high_freq_utm_pose_match,high_freq_LLH_match,lon0);
                    high_freq_match.pos.lan = high_freq_LLH_match[0];
                    high_freq_match.pos.lon = high_freq_LLH_match[1];
                    high_freq_match.pos.h = high_freq_LLH_match[2];
                }
                else if (hf_time.msec == lf_time.msec)
                {
                    //根据传感器类型后续再加
                    if(g_pfscenter->high_freq_type == g_pfscenter->integrated_nav)
                    {
                        high_freq_match.pos.lan = g_pfscenter->g_record_fuse.pose_llh.x;
                        high_freq_match.pos.lon = g_pfscenter->g_record_fuse.pose_llh.y;
                        high_freq_match.pos.h = g_pfscenter->g_record_fuse.pose_llh.z;
                        high_freq_match.vel.venu.vx = g_pfscenter->g_record_fuse.velocity.linear.x;
                        high_freq_match.vel.venu.vy = g_pfscenter->g_record_fuse.velocity.linear.y;
                        high_freq_match.vel.venu.vz = g_pfscenter->g_record_fuse.velocity.linear.z;
                        high_freq_match.vel.wxyz.wx = g_pfscenter->g_record_fuse.velocity.angular.x;     //机体坐标系角速度
                        high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
                        high_freq_match.vel.wxyz.wz = g_pfscenter->g_record_fuse.velocity.angular.z;
                        high_freq_match.att.yaw = g_pfscenter->g_record_fuse.yaw;
                        high_freq_match.att.pitch = g_pfscenter->g_record_fuse.pitch;
                        high_freq_match.att.roll = g_pfscenter->g_record_fuse.roll;
                        high_freq_match.accel.ax = g_pfscenter->g_record_fuse.accel.linear.x;   //加速度计采的机体加速度
                        high_freq_match.accel.ay = g_pfscenter->g_record_fuse.accel.linear.y;
                        high_freq_match.accel.az = g_pfscenter->g_record_fuse.accel.linear.z;
                    }
                    
                }
                else
                {
                    Vector3d B;
                    B = Vector3d::Zero();
                    Matrix3d A;
                    A = Matrix3d::Zero();
                    Vector3d X;
                    X = Vector3d::Zero();
                    double circle[4];
                    CalculateCircular(pose_match_utm,A,B,X,circle);
                    float delta_t = 0.0;    //配准时间间隔
                    delta_t = lf_time.sec + lf_time.msec/1000 - (hf_time.sec + hf_time.msec/1000);
                    double delta_sita = circle[3]*delta_t; 
                    double high_freq_utm_pose_match[3];   //匹配后的坐标，84系统
                    high_freq_utm_pose_match[0] = circle[0] + circle[2]*cos(acos((pose_match_utm[2][0] - circle[0])/circle[2]) + delta_sita);
                    high_freq_utm_pose_match[1] = circle[1] + circle[2]*sin(acos((pose_match_utm[2][0] - circle[0])/circle[2]) + delta_sita);
                    high_freq_utm_pose_match[2] = pose_match_utm[2][2];
                    //根据传感器类型后续再加
                    if(g_pfscenter->high_freq_type == g_pfscenter->integrated_nav)
                    {
                        high_freq_match.vel.venu.vx = g_pfscenter->g_record_fuse.velocity.linear.x;
                        high_freq_match.vel.venu.vy = g_pfscenter->g_record_fuse.velocity.linear.y;
                        high_freq_match.vel.venu.vz = g_pfscenter->g_record_fuse.velocity.linear.z;
                        high_freq_match.vel.wxyz.wx = g_pfscenter->g_record_fuse.velocity.angular.x;
                        high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
                        high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
                        high_freq_match.att.yaw = g_pfscenter->g_record_fuse.yaw;
                        high_freq_match.att.pitch = g_pfscenter->g_record_fuse.pitch;
                        high_freq_match.att.roll = g_pfscenter->g_record_fuse.roll;
                        high_freq_match.accel.ax = g_pfscenter->g_record_fuse.accel.linear.x;   //加速度计采的机体加速度
                        high_freq_match.accel.ay = g_pfscenter->g_record_fuse.accel.linear.y;
                        high_freq_match.accel.az = g_pfscenter->g_record_fuse.accel.linear.z;
                    }
                    transForUTMtoLLH(high_freq_utm_pose_match,high_freq_LLH_match,lon0);
                    high_freq_match.pos.lan = high_freq_LLH_match[0];
                    high_freq_match.pos.lon = high_freq_LLH_match[1];
                    high_freq_match.pos.h = high_freq_LLH_match[2];
                }
            }
            else
            {
                Vector3d B;
                B = Vector3d::Zero();
                Matrix3d A;
                A = Matrix3d::Zero();
                Vector3d X;
                X = Vector3d::Zero();
                double circle[4];
                CalculateCircular(pose_match_utm,A,B,X,circle);
                double high_freq_utm_pose_match[3];   //匹配后的坐标，84系统
                float delta_t = 0.0;    //配准时间间隔
                delta_t = lf_time.sec + lf_time.msec/1000 - (hf_time.sec + hf_time.msec/1000);
                double delta_sita = circle[3]*delta_t;
                high_freq_utm_pose_match[0] = circle[0] + circle[2]*cos(acos((pose_match_utm[2][0] - circle[0])/circle[2]) + delta_sita);
                high_freq_utm_pose_match[1] = circle[1] + circle[2]*sin(acos((pose_match_utm[2][0] - circle[0])/circle[2]) + delta_sita);
                high_freq_utm_pose_match[2] = pose_match_utm[2][2];
                //根据传感器类型后续再加
                if(g_pfscenter->high_freq_type == g_pfscenter->integrated_nav)
                {
                    high_freq_match.vel.venu.vx = g_pfscenter->g_record_fuse.velocity.linear.x;
                    high_freq_match.vel.venu.vy = g_pfscenter->g_record_fuse.velocity.linear.y;
                    high_freq_match.vel.venu.vz = g_pfscenter->g_record_fuse.velocity.linear.z;
                    high_freq_match.vel.wxyz.wx = g_pfscenter->g_record_fuse.velocity.angular.x;
                    high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
                    high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
                    high_freq_match.att.yaw = g_pfscenter->g_record_fuse.yaw;
                    high_freq_match.att.pitch = g_pfscenter->g_record_fuse.pitch;
                    high_freq_match.att.roll = g_pfscenter->g_record_fuse.roll;
                    high_freq_match.accel.ax = g_pfscenter->g_record_fuse.accel.linear.x;   //加速度计采的机体加速度
                    high_freq_match.accel.ay = g_pfscenter->g_record_fuse.accel.linear.y;
                    high_freq_match.accel.az = g_pfscenter->g_record_fuse.accel.linear.z;
                }
                transForUTMtoLLH(high_freq_utm_pose_match,high_freq_LLH_match,lon0);
                high_freq_match.pos.lan = high_freq_LLH_match[0];
                high_freq_match.pos.lon = high_freq_LLH_match[1];
                high_freq_match.pos.h = high_freq_LLH_match[2];
            } 
        }
        else
        {
            Vector3d B;
            B = Vector3d::Zero();
            Matrix3d A;
            A = Matrix3d::Zero();
            Vector3d X;
            X = Vector3d::Zero();
            double circle[4];
            CalculateCircular(pose_match_utm,A,B,X,circle);
            double high_freq_utm_pose_match[3];   //匹配后的坐标，84系统
            float delta_t = 0.0;    //配准时间间隔
            delta_t = hf_time.min*60 + hf_time.sec + hf_time.msec/1000 - 
                      (lf_time.min*60 + lf_time.sec + hf_time.msec/1000);
            if(delta_t > 0){
                delta_t = (lf_time.msec - (1000 + hf_time.msec - Ta))/1000;
                double delta_sita = circle[3]*delta_t;
                high_freq_utm_pose_match[0] = circle[0] + circle[2]*cos(acos((pose_match_utm[1][0] - circle[0])/circle[2]) + delta_sita);
                high_freq_utm_pose_match[1] = circle[1] + circle[2]*sin(acos((pose_match_utm[1][0] - circle[0])/circle[2]) + delta_sita);
                high_freq_utm_pose_match[2] = pose_match_utm[1][2];
                //根据传感器类型后续再加
                if(g_pfscenter->high_freq_type == g_pfscenter->integrated_nav)
                {
                    high_freq_match.vel.venu.vx = g_pfscenter->g_record_fuse.velocity.linear.x;
                    high_freq_match.vel.venu.vy = g_pfscenter->g_record_fuse.velocity.linear.y;
                    high_freq_match.vel.venu.vz = g_pfscenter->g_record_fuse.velocity.linear.z;
                    high_freq_match.vel.wxyz.wx = g_pfscenter->g_record_fuse.velocity.angular.x;
                    high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
                    high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
                    high_freq_match.att.yaw = g_pfscenter->g_record_fuse.yaw;
                    high_freq_match.att.pitch = g_pfscenter->g_record_fuse.pitch;
                    high_freq_match.att.roll = g_pfscenter->g_record_fuse.roll;
                    high_freq_match.accel.ax = g_pfscenter->g_record_fuse.accel.linear.x;   //加速度计采的机体加速度
                    high_freq_match.accel.ay = g_pfscenter->g_record_fuse.accel.linear.y;
                    high_freq_match.accel.az = g_pfscenter->g_record_fuse.accel.linear.z;
                }
                transForUTMtoLLH(high_freq_utm_pose_match,high_freq_LLH_match,lon0);
                high_freq_match.pos.lan = high_freq_LLH_match[0];
                high_freq_match.pos.lon = high_freq_LLH_match[1];
                high_freq_match.pos.h = high_freq_LLH_match[2];
            }
            else
            {
                delta_t = (1000 + lf_time.msec - hf_time.msec)/1000;
                double delta_sita = circle[3]*delta_t;
                high_freq_utm_pose_match[0] = circle[0] + circle[2]*cos(acos((pose_match_utm[2][0] - circle[0])/circle[2]) + delta_sita);
                high_freq_utm_pose_match[1] = circle[1] + circle[2]*sin(acos((pose_match_utm[2][0] - circle[0])/circle[2]) + delta_sita);
                high_freq_utm_pose_match[2] = pose_match_utm[2][2];
                //根据传感器类型后续再加
                if(g_pfscenter->high_freq_type == g_pfscenter->integrated_nav)
                {
                    high_freq_match.vel.venu.vx = g_pfscenter->g_record_fuse.velocity.linear.x;
                    high_freq_match.vel.venu.vy = g_pfscenter->g_record_fuse.velocity.linear.y;
                    high_freq_match.vel.venu.vz = g_pfscenter->g_record_fuse.velocity.linear.z;
                    high_freq_match.vel.wxyz.wx = g_pfscenter->g_record_fuse.velocity.angular.x;
                    high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
                    high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
                    high_freq_match.att.yaw = g_pfscenter->g_record_fuse.yaw;
                    high_freq_match.att.pitch = g_pfscenter->g_record_fuse.pitch;
                    high_freq_match.att.roll = g_pfscenter->g_record_fuse.roll;
                    high_freq_match.accel.ax = g_pfscenter->g_record_fuse.accel.linear.x;   //加速度计采的机体加速度
                    high_freq_match.accel.ay = g_pfscenter->g_record_fuse.accel.linear.y;
                    high_freq_match.accel.az = g_pfscenter->g_record_fuse.accel.linear.z;
                }
                transForUTMtoLLH(high_freq_utm_pose_match,high_freq_LLH_match,lon0);
                high_freq_match.pos.lan = high_freq_LLH_match[0];
                high_freq_match.pos.lon = high_freq_LLH_match[1];
                high_freq_match.pos.h = high_freq_LLH_match[2];
            }   
        } 
    }
    else
    {
        Vector3d B;
        B = Vector3d::Zero();
        Matrix3d A;
        A = Matrix3d::Zero();
        Vector3d X;
        X = Vector3d::Zero();
        double circle[4];
        CalculateCircular(pose_match_utm,A,B,X,circle);
        double high_freq_utm_pose_match[3];   //匹配后的坐标，84系统
        float delta_t = 0.0;    //配准时间间隔
        delta_t = hf_time.hour*3600 + hf_time.min*60 + hf_time.sec + hf_time.msec/1000 - 
                  (lf_time.hour*3600 + lf_time.min*60 + lf_time.sec + lf_time.msec/1000);
        if(delta_t > 0)
        {
            delta_t = (lf_time.msec - (1000 + hf_time.msec - Ta))/1000;
            double delta_sita = circle[3]*delta_t;
            high_freq_utm_pose_match[0] = circle[0] + circle[2]*cos(acos((pose_match_utm[1][0] - circle[0])/circle[2]) + delta_sita);
            high_freq_utm_pose_match[1] = circle[1] + circle[2]*sin(acos((pose_match_utm[1][0] - circle[0])/circle[2]) + delta_sita);
            high_freq_utm_pose_match[2] = pose_match_utm[1][2];
            //根据传感器类型后续再加
            if(g_pfscenter->high_freq_type == g_pfscenter->integrated_nav)
            {
                high_freq_match.vel.venu.vx = g_pfscenter->g_record_fuse.velocity.linear.x;
                high_freq_match.vel.venu.vy = g_pfscenter->g_record_fuse.velocity.linear.y;
                high_freq_match.vel.venu.vz = g_pfscenter->g_record_fuse.velocity.linear.z;
                high_freq_match.vel.wxyz.wx = g_pfscenter->g_record_fuse.velocity.angular.x;
                high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
                high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
                high_freq_match.att.yaw = g_pfscenter->g_record_fuse.yaw;
                high_freq_match.att.pitch = g_pfscenter->g_record_fuse.pitch;
                high_freq_match.att.roll = g_pfscenter->g_record_fuse.roll;
                high_freq_match.accel.ax = g_pfscenter->g_record_fuse.accel.linear.x;   //加速度计采的机体加速度
                high_freq_match.accel.ay = g_pfscenter->g_record_fuse.accel.linear.y;
                high_freq_match.accel.az = g_pfscenter->g_record_fuse.accel.linear.z;
            }
            transWGS84toLLH(high_freq_utm_pose_match,high_freq_LLH_match);
            high_freq_match.pos.lan = high_freq_LLH_match[0];
            high_freq_match.pos.lon = high_freq_LLH_match[1];
            high_freq_match.pos.h = high_freq_LLH_match[2];
        }
        else
        {
            delta_t = (1000 + lf_time.msec - hf_time.msec)/1000;
            double delta_sita = circle[3]*delta_t;
            high_freq_utm_pose_match[0] = circle[0] + circle[2]*cos(acos((pose_match_utm[2][0] - circle[0])/circle[2]) + delta_sita);
            high_freq_utm_pose_match[1] = circle[1] + circle[2]*sin(acos((pose_match_utm[2][0] - circle[0])/circle[2]) + delta_sita);
            high_freq_utm_pose_match[2] = pose_match_utm[2][2];
            //根据传感器类型后续再加
            if(g_pfscenter->high_freq_type == g_pfscenter->integrated_nav)
            {
                high_freq_match.vel.venu.vx = g_pfscenter->g_record_fuse.velocity.linear.x;
                high_freq_match.vel.venu.vy = g_pfscenter->g_record_fuse.velocity.linear.y;
                high_freq_match.vel.venu.vz = g_pfscenter->g_record_fuse.velocity.linear.z;
                high_freq_match.vel.wxyz.wx = g_pfscenter->g_record_fuse.velocity.angular.x;
                high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
                high_freq_match.vel.wxyz.wy = g_pfscenter->g_record_fuse.velocity.angular.y;
                high_freq_match.att.yaw = g_pfscenter->g_record_fuse.yaw;
                high_freq_match.att.pitch = g_pfscenter->g_record_fuse.pitch;
                high_freq_match.att.roll = g_pfscenter->g_record_fuse.roll;
                high_freq_match.accel.ax = g_pfscenter->g_record_fuse.accel.linear.x;   //加速度计采的机体加速度
                high_freq_match.accel.ay = g_pfscenter->g_record_fuse.accel.linear.y;
                high_freq_match.accel.az = g_pfscenter->g_record_fuse.accel.linear.z;
            }
            transForUTMtoLLH(high_freq_utm_pose_match,high_freq_LLH_match,lon0);
            high_freq_match.pos.lan = high_freq_LLH_match[0];
            high_freq_match.pos.lon = high_freq_LLH_match[1];
            high_freq_match.pos.h = high_freq_LLH_match[2];
        }
        
    }
    
}


void MatchForTime::CalculateCircular(vector<double*>&pose_match_utm,Matrix3d &A,Vector3d &B,Vector3d &X,double (&circle)[4])
{
    for(int i=0 ; i<3; i++)
    {
        B(i) = pow(pose_match_utm[i][0],2) + pow(pose_match_utm[i][1],2);
        A(i,0) = 2*pose_match_utm[i][0];
        A(i,1) = 2*pose_match_utm[i][1];
        A(i,2) = 1;
    }
    if (A.determinant() != 1e-6)
    {
        X = A.colPivHouseholderQr().solve(B);
        double X0 = -1/2*X(0);    //  圆心
        double Y0 = -1/2*X(1);
        double R0 = sqrt(1/4*(pow(X(0),2) + pow(X(1),2)) - X(2));     //半径
        double Sita = acos((pose_match_utm[2][0] - X0)/R0) - acos((pose_match_utm[1][0] - X0)/R0);
        double w_i = Sita/Ta;   //角速度
        circle[0] = X0;
        circle[1] = Y0;
        circle[2] = R0;
        circle[3] = w_i;
    }
    else
    {
        cout<<"矩阵奇异"<<endl;
    }
    
    
}


void MatchForTime::setTime(UTC& hf_tm, UTC& lf_tm)
{
    //时间分支后续加入lidar等后再接着加入
    if(g_pfscenter->high_freq_type == g_pfscenter->integrated_nav)
    {
        hf_tm.hour = g_pfscenter->g_record_fuse.hour;
        hf_tm.min = g_pfscenter->g_record_fuse.min;
        hf_tm.sec = g_pfscenter->g_record_fuse.sec;
        hf_tm.msec = g_pfscenter->g_record_fuse.msec;
    }
    if(g_pfscenter->low_freq_type == g_pfscenter->uwb){
        lf_tm.hour = g_pfscenter->g_UWB_info.hour;
        lf_tm.min = g_pfscenter->g_UWB_info.min;
        lf_tm.sec = g_pfscenter->g_UWB_info.sec;
        lf_tm.msec = g_pfscenter->g_UWB_info.msec;
    }
    else if (g_pfscenter->low_freq_type == g_pfscenter->fixed_lidar )
    {
        lf_tm.hour = g_pfscenter->g_flidar_info.hour;
        lf_tm.min = g_pfscenter->g_flidar_info.min;
        lf_tm.sec = g_pfscenter->g_flidar_info.sec;
        lf_tm.msec = g_pfscenter->g_flidar_info.msec;
    }
    else if (g_pfscenter->low_freq_type == g_pfscenter->lidar)
    {
        lf_tm.hour = g_pfscenter->g_lidar_info.hour;
        lf_tm.min = g_pfscenter->g_lidar_info.min;
        lf_tm.sec = g_pfscenter->g_lidar_info.sec;
        lf_tm.msec = g_pfscenter->g_lidar_info.msec;
    }
    
}
