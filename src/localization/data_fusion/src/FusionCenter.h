#pragma once
#include <ros/ros.h>
#include <string>
#include <location_sensor_msgs/IMUAndGNSSInfo.h>
#include <location_msgs/FusionDataInfo.h>
#include <location_sensor_msgs/UWBInfo.h>
#include <location_sensor_msgs/FixedLidarInfo.h>
#include <location_sensor_msgs/LidarInfo.h>
#include <location_sensor_msgs/DRInfo.h>
#include <hmi_msgs/ADStatus.h>
#include "DataType.h"
#include "MatchForTime.h"
#include "FusionDR.h"
#include "FusionUWB.h"
#include "FusionFixedLidar.h"
#include "FusionLidar.h"
#include "FusionZUPTUWB.h"
#include "FusionZUPTFixedLidar.h"
#include "FusionZUPTLidar.h"
#include "CalibrateForIMU.h"
#include <vector>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

class MatchForTime;
class FusionDR;
class FusionUWB;
class FusionFixedLidar;
class FusionLidar;
class FusionZUPTUWB;
class FusionZUPTFixedLidar;
class FusionZUPTLidar;
class CalibrateForIMU;


class FusionCenter
{
  public:
    FusionCenter(ros::NodeHandle &fusion_nh);
    ~FusionCenter(void);

  private:
    MatchForTime *g_pmatch_for_time; //用于时间配准
    FusionDR *g_pfsdr;                  //用于组合导航与航位推算融合
    FusionUWB *g_pfsuwb;               //用于组合导航与UWB融合
    FusionFixedLidar *g_pfsflidar;    //用于组合导航与场端lidar融合
    FusionLidar *g_pfslidar;          //用于组合导航与lidar融合
    FusionZUPTUWB *g_pfszuptuwb;      //用于组合导航与带有zupt的UWB融合
    FusionZUPTFixedLidar *g_pfszuptflidar;      //用于组合导航与带有zupt的场端lidar融合
    FusionZUPTLidar *g_pfszuptlidar;            //用于组合导航与带有zupt的lidar融合
    CalibrateForIMU *g_pcalibimu;           //用于无传感器时IMU的误差补偿

  private:
  int g_IMUcalib_count;       //用于记录g_record_fuse中第几个IMU数据要做无传感器数据补偿
  bool DR_update_flag;
  bool IMU_GNSS_update_flag;
  bool UWB_update_flag;
  bool lidar_update_flag;
  bool fixed_lidar_update_flag;
  int sys_status_cnt; //记录标定状态为组合导航模式的数量
  int delta_cnt;    //记录订阅IMU_GNSS数据一次更新了几组
  int point_to; //用于记录指向buffer里第几个IMU数据用于运算

  public:
    bool g_drkint_flag;        //航位推算卡尔曼滤波初始标志
    bool g_ukinit_flag;       //UWB卡尔曼滤波初始标志
    bool g_flkinit_flag;      //场端lidar卡尔曼滤波初始标志
    bool g_lkinit_flag;       //lidar卡尔曼滤波初始标志
    bool g_uzkinit_flag;      //UWB带有ZUPT卡尔曼滤波初始标志
    bool g_flzkinit_flag;     //场端lidar带有ZUPT卡尔曼滤波初始标志
    bool g_lzkinit_flag;      //lidar带有ZUPT卡尔曼滤波初始标志
    enum CalibrationType
    {
      c_init = 1,
      c_yes = 2,
      c_no = 3
    } g_calib_flag;     //是否进行惯导与其他传感器融合定位校准
    enum SensorType
    {
        integrated_nav = 1,
        uwb = 2,
        lidar = 3,
        fixed_lidar = 4,
        zupt_stop = 5,
        zupt_linear = 6,
        dr = 7
    } high_freq_type,
      low_freq_type,
      zupt_type; //传感器类型

    VectorXd g_delta_IMUcalib;   //用于校正IMU姿态速度与位置的值,yaw,pitch,roll，ve,vn,vu,lan,lon,h
    MatrixXd g_IMUcalib_I;      //用于校正IMU的状态方程的估计均方误差
    UTC g_IMUlast_tm;           //用于校正IMU的上一时间配准时刻

  public:
    location_msgs::FusionDataInfo g_fusion_data; //融合数据
    location_msgs::FusionDataInfo g_fusion_data_calib; //无传感器的融合数据
    location_msgs::FusionDataInfo g_IMU_GNSS_info;
    location_sensor_msgs::DRInfo g_DR_info;
    location_sensor_msgs::UWBInfo g_UWB_info;
    location_sensor_msgs::FixedLidarInfo g_flidar_info;
    location_sensor_msgs::LidarInfo g_lidar_info;
    hmi_msgs::ADStatus g_adstatus_info;
    vector <location_msgs::FusionDataInfo> g_vprecord_fuse; //用于记录高频传感器相邻100组数据
    location_msgs::FusionDataInfo g_record_fuse;            //用于记录用于与各传感器融合及匹配时的IMU数据
    int g_record_match_count;   //用于记录高频传感器相邻100个数据的位置与速度
    PoseResult g_high_freq_match; //时间匹配后的位置速度姿态

  private:
    void callbackForIntegrationNavigation(const location_sensor_msgs::IMUAndGNSSInfoConstPtr &inte_nav_info); //订阅组合导航的回调函数
    void callbackForDR(const location_sensor_msgs::DRInfoConstPtr &dr_info);  //订阅航位推算的回调函数
    void callbackForUWB(const location_sensor_msgs::UWBInfoConstPtr &uwb_info); //订阅UWB的回调函数
    void callbackForFixedLidar(const location_sensor_msgs::FixedLidarInfoConstPtr &flidar_info);  //订阅场端Lidar的回调函数
    void callbackForLidar(const location_sensor_msgs::LidarInfoConstPtr &lidar_info);             //订阅Lidar的回调函数
    void callbackForADStatus(const hmi_msgs::ADStatusConstPtr &adstatus_info);          //订阅ADStatus的回调函数
    void processDRFuse(vector <double*> &pose_match_utm,vector <double*> &vel_match_utm);     //航位推算与组合导航融合
    void processUWBFuse(vector <double*> &pose_match_utm,vector <double*> &vel_match_utm);    //UWB与组合导航做融合
    void processFixedLidarFuse(vector <double*> &pose_match_utm,vector <double*> &vel_match_utm);   //场端Lidar与组合导航做融合
    void processLidarFuse(vector <double*> &pose_match_utm,vector <double*> &vel_match_utm);        //车端Lidar与组合导航做融合
    void processIMUCalibrate(); //没有其他传感器辅助时的IMU位置外推
    void recordIMUGNSSData();   //记录组合导航数据
    void findIMUDataForMatch(int &tm_delay,vector<double *> &pose_match_utm,vector<double *> &vel_match_utm,UTC &low_fre_utc, double &lon0); //找到用于当前时间匹配的IMU数据

};