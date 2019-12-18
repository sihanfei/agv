#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string>
#include <sstream>
#include <cmath>
#include <cstdlib> //string转化为double
#include <iomanip> //保留有效小数
#include "FusionCenter.h"
#include "CoordinateSystem.h"

#include <dynamic_reconfigure/server.h>
#include "data_fusion/IMU_global_Config.h"
#include "data_fusion/IMU_Lidar_Config.h"
#include "data_fusion/IMU_UWB_Config.h"
#include "data_fusion/IMU_ZUPT_Lidar_Config.h"
#include "data_fusion/IMU_ZUPT_UWB_Config.h"
#include "GlobalVari.h"

#define buffer_size (100)    //存储IMU数据的buf长度

// #include "data_fusion/IMU_Fixed_Lidar_Config.h"
// #include "data_fusion/IMU_ZUPT_FLidar_Config.h"

//---test
int g_imu_count_cfg = 3000;
float g_lidar_pose_confidience_cfg = 0.0; //!!!0.95调参量

Cfg_UWB g_cfg_uwb;
Cfg_ZUPTUWB g_cfg_zuptuwb;
Cfg_Lidar g_cfg_lidar;
Cfg_ZUPTLidar g_cfg_zuptlidar;
void IMU_GLOBALcallback(IMU_config_package::IMU_global_Config &config, uint32_t level)
{
    g_imu_count_cfg = config.IMU_count;
    g_lidar_pose_confidience_cfg = config.Lidar_pose_confidence;
    //   ROS_INFO("IMU_GLOBAL: %d %f", config.IMU_count, config.Lidar_pose_confidence);
}

void IMU_UWBcallback(IMU_config_package::IMU_UWB_Config &config, uint32_t level)
{
    //P0
    g_cfg_uwb.P0.Fai_yaw = config.UWB_P0_Fai_yaw;
    g_cfg_uwb.P0.Fai_pitch = config.UWB_P1_Fai_pitch;
    g_cfg_uwb.P0.Fai_roll = config.UWB_P2_Fai_roll;
    g_cfg_uwb.P0.delta_ve = config.UWB_P3_delta_ve;
    g_cfg_uwb.P0.delta_vn = config.UWB_P4_delta_vn;
    g_cfg_uwb.P0.delta_vu = config.UWB_P5_delta_vu;
    g_cfg_uwb.P0.delta_lan = config.UWB_P6_delta_lan;
    g_cfg_uwb.P0.delta_lon = config.UWB_P7_delta_lon;
    g_cfg_uwb.P0.delta_h = config.UWB_P8_delta_h;
    g_cfg_uwb.P0.delta_Uve = config.UWB_P9_delta_Uve;
    g_cfg_uwb.P0.delta_Uvn = config.UWB_P10_delta_Uvn;
    g_cfg_uwb.P0.delta_Ulan = config.UWB_P11_delta_Ulan;
    g_cfg_uwb.P0.delta_Ulon = config.UWB_P12_delta_Ulon;

    //q0
    g_cfg_uwb.q0.Fai_yaw = config.UWB_Q0_Fai_yaw;
    g_cfg_uwb.q0.Fai_pitch = config.UWB_Q1_Fai_pitch;
    g_cfg_uwb.q0.Fai_roll = config.UWB_Q2_Fai_roll;
    g_cfg_uwb.q0.delta_ve = config.UWB_Q3_delta_ve;
    g_cfg_uwb.q0.delta_vn = config.UWB_Q4_delta_vn;
    g_cfg_uwb.q0.delta_vu = config.UWB_Q5_delta_vu;
    g_cfg_uwb.q0.delta_lan = config.UWB_Q6_delta_lan;
    g_cfg_uwb.q0.delta_lon = config.UWB_Q7_delta_lon;
    g_cfg_uwb.q0.delta_Uve = config.UWB_Q9_delta_Uve;
    g_cfg_uwb.q0.delta_Uvn = config.UWB_Q10_delta_Uvn;
    g_cfg_uwb.q0.delta_Ulan = config.UWB_Q11_delta_Ulan;
    g_cfg_uwb.q0.delta_Ulon = config.UWB_Q12_delta_Ulon;

    //X0
    g_cfg_uwb.X0.Fai_yaw = config.UWB_X00_Fai_yaw;
    g_cfg_uwb.X0.Fai_pitch = config.UWB_X01_Fai_pitch;
    g_cfg_uwb.X0.Fai_roll = config.UWB_X02_Fai_roll;
    g_cfg_uwb.X0.delta_ve = config.UWB_X03_delta_ve;
    g_cfg_uwb.X0.delta_vn = config.UWB_X04_delta_vn;
    g_cfg_uwb.X0.delta_vu = config.UWB_X05_delta_vu;
    g_cfg_uwb.X0.delta_lan = config.UWB_X06_delta_lan;
    g_cfg_uwb.X0.delta_lon = config.UWB_X07_delta_lon;
    g_cfg_uwb.X0.delta_Uve = config.UWB_X09_delta_Uve;
    g_cfg_uwb.X0.delta_Uvn = config.UWB_X010_delta_Uvn;
    g_cfg_uwb.X0.delta_Ulan = config.UWB_X011_delta_Ulan;
    g_cfg_uwb.X0.delta_Ulon = config.UWB_X012_delta_Ulon;

    //R:
    g_cfg_uwb.R.delta_yaw = config.UWB_R0_delta_yaw;
    g_cfg_uwb.R.delta_Cve = config.UWB_R1_delta_Cve;
    g_cfg_uwb.R.delta_Cvn = config.UWB_R2_delta_Cvn;
    g_cfg_uwb.R.delta_Clan = config.UWB_R3_delta_Clan;
    g_cfg_uwb.R.delta_Clon = config.UWB_R4_delta_Clon;

    //Tao
    g_cfg_uwb.Tao.delta_Uve = config.UWB_Tao0_delta_Uve;
    g_cfg_uwb.Tao.delta_Uvn = config.UWB_Tao1_delta_Uvn;
    g_cfg_uwb.Tao.delta_Ulan = config.UWB_Tao2_delta_Ulan;
    g_cfg_uwb.Tao.delta_Ulon = config.UWB_Tao3_delta_Ulon;

    //ROS_INFO("IMU_UWB: %f %f", g_cfg_uwb.P0.Fai_yaw, g_cfg_uwb.P0.Fai_pitch);
}

void IMU_ZUPT_UWBcallback(IMU_config_package::IMU_ZUPT_UWB_Config &config, uint32_t level)
{
    //P0:
    g_cfg_zuptuwb.P0.Fai_yaw = config.ZUPT_UWB_P0_Fai_yaw;
    g_cfg_zuptuwb.P0.Fai_pitch = config.ZUPT_UWB_P1_Fai_pitch;
    g_cfg_zuptuwb.P0.Fai_roll = config.ZUPT_UWB_P2_Fai_roll;
    g_cfg_zuptuwb.P0.delta_ve = config.ZUPT_UWB_P3_delta_ve;
    g_cfg_zuptuwb.P0.delta_vn = config.ZUPT_UWB_P4_delta_vn;
    g_cfg_zuptuwb.P0.delta_vu = config.ZUPT_UWB_P5_delta_vu;
    g_cfg_zuptuwb.P0.delta_lan = config.ZUPT_UWB_P6_delta_lan;
    g_cfg_zuptuwb.P0.delta_lon = config.ZUPT_UWB_P7_delta_lon;
    g_cfg_zuptuwb.P0.delta_h = config.ZUPT_UWB_P8_delta_h;
    g_cfg_zuptuwb.P0.delta_Uve = config.ZUPT_UWB_P9_delta_Uve;
    g_cfg_zuptuwb.P0.delta_Uvn = config.ZUPT_UWB_P10_delta_Uvn;
    g_cfg_zuptuwb.P0.delta_Ulan = config.ZUPT_UWB_P11_delta_Ulan;
    g_cfg_zuptuwb.P0.delta_Ulon = config.ZUPT_UWB_P12_delta_Ulon;

    //q0:
    g_cfg_zuptuwb.q0.Fai_yaw = config.ZUPT_UWB_Q0_Fai_yaw;
    g_cfg_zuptuwb.q0.Fai_pitch = config.ZUPT_UWB_Q1_Fai_pitch;
    g_cfg_zuptuwb.q0.Fai_roll = config.ZUPT_UWB_Q2_Fai_roll;
    g_cfg_zuptuwb.q0.delta_ve = config.ZUPT_UWB_Q3_delta_ve;
    g_cfg_zuptuwb.q0.delta_vn = config.ZUPT_UWB_Q4_delta_vn;
    g_cfg_zuptuwb.q0.delta_vu = config.ZUPT_UWB_Q5_delta_vu;
    g_cfg_zuptuwb.q0.delta_lan = config.ZUPT_UWB_Q6_delta_lan;
    g_cfg_zuptuwb.q0.delta_lon = config.ZUPT_UWB_Q7_delta_lon;
    g_cfg_zuptuwb.q0.delta_h = config.ZUPT_UWB_Q8_delta_h;
    g_cfg_zuptuwb.q0.delta_Uve = config.ZUPT_UWB_Q9_delta_Uve;
    g_cfg_zuptuwb.q0.delta_Uvn = config.ZUPT_UWB_Q10_delta_Uvn;
    g_cfg_zuptuwb.q0.delta_Ulan = config.ZUPT_UWB_Q11_delta_Ulan;
    g_cfg_zuptuwb.q0.delta_Ulon = config.ZUPT_UWB_Q12_delta_Ulon;

    //X0:
    g_cfg_zuptuwb.X0.Fai_yaw = config.ZUPT_UWB_X00_Fai_yaw;
    g_cfg_zuptuwb.X0.Fai_pitch = config.ZUPT_UWB_X01_Fai_pitch;
    g_cfg_zuptuwb.X0.Fai_roll = config.ZUPT_UWB_X02_Fai_roll;
    g_cfg_zuptuwb.X0.delta_ve = config.ZUPT_UWB_X03_delta_ve;
    g_cfg_zuptuwb.X0.delta_vn = config.ZUPT_UWB_X04_delta_vn;
    g_cfg_zuptuwb.X0.delta_vu = config.ZUPT_UWB_X06_delta_lan;
    g_cfg_zuptuwb.X0.delta_lan = config.ZUPT_UWB_X06_delta_lan;
    g_cfg_zuptuwb.X0.delta_lon = config.ZUPT_UWB_X07_delta_lon;
    g_cfg_zuptuwb.X0.delta_h = config.ZUPT_UWB_X08_delta_h;
    g_cfg_zuptuwb.X0.delta_Uve = config.ZUPT_UWB_X09_delta_Uve;
    g_cfg_zuptuwb.X0.delta_Uvn = config.ZUPT_UWB_X010_delta_Uvn;
    g_cfg_zuptuwb.X0.delta_Ulan = config.ZUPT_UWB_X011_delta_Ulan;
    g_cfg_zuptuwb.X0.delta_Ulon = config.ZUPT_UWB_X012_delta_Ulon;

    //zupt_stop_R:
    g_cfg_zuptuwb.ZSTOP_R.delta_yaw = config.ZUPTSTOP_UWB_R0_delta_yaw;
    g_cfg_zuptuwb.ZSTOP_R.delta_ve = config.ZUPTSTOP_UWB_R1_delta_ve;
    g_cfg_zuptuwb.ZSTOP_R.delta_vn = config.ZUPTSTOP_UWB_R2_delta_vn;
    g_cfg_zuptuwb.ZSTOP_R.delta_vu = config.ZUPTSTOP_UWB_R3_delta_vu;
    g_cfg_zuptuwb.ZSTOP_R.delta_Uve = config.ZUPTSTOP_UWB_R4_delta_Uve;
    g_cfg_zuptuwb.ZSTOP_R.delta_Uvn = config.ZUPTSTOP_UWB_R5_delta_Uvn;
    g_cfg_zuptuwb.ZSTOP_R.delta_Clan = config.ZUPTSTOP_UWB_R6_delta_Clan;
    g_cfg_zuptuwb.ZSTOP_R.delta_Clon = config.ZUPTSTOP_UWB_R7_elta_Clon;
    //zupt_lin_R:
    g_cfg_zuptuwb.ZLIN_R.delta_yaw = config.ZUPTLIN_UWB_R0_delta_yaw;
    g_cfg_zuptuwb.ZLIN_R.delta_Cve = config.ZUPTLIN_UWB_R1_delta_Cve;
    g_cfg_zuptuwb.ZLIN_R.delta_Cvn = config.ZUPTLIN_UWB_R2_delta_Cvn;
    g_cfg_zuptuwb.ZLIN_R.delta_Clan = config.ZUPTLIN_UWB_R3_delta_Clan;
    g_cfg_zuptuwb.ZLIN_R.delta_Clon = config.ZUPTLIN_UWB_R4_delta_Clon;
    g_cfg_zuptuwb.ZLIN_R.Bvx = config.ZUPTLIN_UWB_R5_Bvx;
    g_cfg_zuptuwb.ZLIN_R.Bvz = config.ZUPTLIN_UWB_R6_Bvz;

    //Tao:
    g_cfg_zuptuwb.Tao.delta_Uve = config.ZUPT_UWB_Tao0_delta_Uve;
    g_cfg_zuptuwb.Tao.delta_Uvn = config.ZUPT_UWB_Tao1_delta_Uv;
    g_cfg_zuptuwb.Tao.delta_Ulan = config.ZUPT_UWB_Tao2_delta_Ulan;
    g_cfg_zuptuwb.Tao.delta_Ulon = config.ZUPT_UWB_Tao3_delta_Ulon;

    // ROS_INFO("IMU_LIDAR: %f %f", config.Lidar_P_0, config.Lidar_Q_0);
}

void IMU_LIDARcallback(IMU_config_package::IMU_Lidar_Config &config, uint32_t level)
{
    //P0:
    g_cfg_lidar.P0.Fai_yaw = config.Lidar_P0_Fai_yaw;
    g_cfg_lidar.P0.Fai_pitch = config.Lidar_P1_Fai_pitch;
    g_cfg_lidar.P0.Fai_roll = config.Lidar_P2_Fai_roll;
    g_cfg_lidar.P0.delta_ve = config.Lidar_P3_delta_ve;
    g_cfg_lidar.P0.delta_vn = config.Lidar_P4_delta_vn;
    g_cfg_lidar.P0.delta_vu = config.Lidar_P5_delta_vu;
    g_cfg_lidar.P0.delta_lan = config.Lidar_P6_delta_lan;
    g_cfg_lidar.P0.delta_lon = config.Lidar_P7_delta_lon;
    g_cfg_lidar.P0.delta_h = config.Lidar_P8_delta_h;
    g_cfg_lidar.P0.delta_Lve = config.Lidar_P9_delta_Lve;
    g_cfg_lidar.P0.delta_Lvn = config.Lidar_P10_delta_Lvn;
    g_cfg_lidar.P0.delta_Lvu = config.Lidar_P11_delta_Lvu;
    g_cfg_lidar.P0.delta_Llan = config.Lidar_P12_delta_Llan;
    g_cfg_lidar.P0.delta_Llon = config.Lidar_P13_delta_Llon;
    g_cfg_lidar.P0.delta_Lh = config.Lidar_P14_delta_Lh;

    //q0:
    g_cfg_lidar.q0.Fai_yaw = config.Lidar_Q0_Fai_yaw;
    g_cfg_lidar.q0.Fai_pitch = config.Lidar_Q1_Fai_pitch;
    g_cfg_lidar.q0.Fai_roll = config.Lidar_Q2_Fai_roll;
    g_cfg_lidar.q0.delta_ve = config.Lidar_Q3_delta_ve;
    g_cfg_lidar.q0.delta_vn = config.Lidar_Q4_delta_vn;
    g_cfg_lidar.q0.delta_vu = config.Lidar_Q5_delta_vu;
    g_cfg_lidar.q0.delta_lan = config.Lidar_Q6_delta_lan;
    g_cfg_lidar.q0.delta_lon = config.Lidar_Q7_delta_lon;
    g_cfg_lidar.q0.delta_h = config.Lidar_Q8_delta_h;
    g_cfg_lidar.q0.delta_Lve = config.Lidar_Q9_delta_Lve;
    g_cfg_lidar.q0.delta_Lvn = config.Lidar_Q10_delta_Lvn;
    g_cfg_lidar.q0.delta_Lvu = config.Lidar_Q11_delta_Lvu;
    g_cfg_lidar.q0.delta_Llan = config.Lidar_Q12_delta_Llan;
    g_cfg_lidar.q0.delta_Llon = config.Lidar_Q13_delta_Llon;
    g_cfg_lidar.q0.delta_Lh = config.Lidar_Q14_delta_Lh;

    //X0:
    g_cfg_lidar.X0.Fai_yaw = config.Lidar_X00_Fai_yaw;
    g_cfg_lidar.X0.Fai_pitch = config.Lidar_X01_Fai_pitch;
    g_cfg_lidar.X0.Fai_roll = config.Lidar_X02_Fai_roll;
    g_cfg_lidar.X0.delta_ve = config.Lidar_X03_delta_ve;
    g_cfg_lidar.X0.delta_vn = config.Lidar_X04_delta_vn;
    g_cfg_lidar.X0.delta_vu = config.Lidar_X05_delta_vu;
    g_cfg_lidar.X0.delta_lan = config.Lidar_X06_delta_lan;
    g_cfg_lidar.X0.delta_lon = config.Lidar_X07_delta_lon;
    g_cfg_lidar.X0.delta_h = config.Lidar_X08_delta_h;
    g_cfg_lidar.X0.delta_Lve = config.Lidar_X09_delta_Lve;
    g_cfg_lidar.X0.delta_Lvn = config.Lidar_X010_delta_Lvn;
    g_cfg_lidar.X0.delta_Lvu = config.Lidar_X011_delta_Lvu;
    g_cfg_lidar.X0.delta_Llan = config.Lidar_X012_delta_Llan;
    g_cfg_lidar.X0.delta_Llon = config.Lidar_X013_delta_Llon;
    g_cfg_lidar.X0.delta_Lh = config.Lidar_X014_delta_Lh;

    //R:
    g_cfg_lidar.R.delta_yaw = config.Lidar_R0_delta_yaw;
    g_cfg_lidar.R.delta_pitch = config.Lidar_R1_delta_pitch;
    g_cfg_lidar.R.delta_roll = config.Lidar_R2_delta_roll;
    g_cfg_lidar.R.delta_Cve = config.Lidar_R3_delta_Cve;
    g_cfg_lidar.R.delta_Cvn = config.Lidar_R4_delta_Cvn;
    g_cfg_lidar.R.delta_Cvu = config.Lidar_R5_delta_Cvu;
    g_cfg_lidar.R.delta_Clan = config.Lidar_R6_delta_Clan;
    g_cfg_lidar.R.delta_Clon = config.Lidar_R7_delta_Clon;
    g_cfg_lidar.R.delta_Ch = config.Lidar_R8_delta_Ch;

    //Tao:
    g_cfg_lidar.Tao.delta_Lve = config.Lidar_Tao0_delta_Lve;
    g_cfg_lidar.Tao.delta_Lvn = config.Lidar_Tao1_delta_Lvn;
    g_cfg_lidar.Tao.delta_Lvu = config.Lidar_Tao2_delta_Lvu;
    g_cfg_lidar.Tao.delta_Llan = config.Lidar_Tao3_delta_Llan;
    g_cfg_lidar.Tao.delta_Llon = config.Lidar_Tao4_delta_Llon;
    g_cfg_lidar.Tao.delta_Lh = config.Lidar_Tao5_delta_Lh;
    //ROS_INFO("IMU_LIDAR: %f %f", g_cfg_lidar.P0.Fai_yaw, g_cfg_lidar.P0.Fai_pitch);
}

void IMU_ZUPT_LIDARcallback(IMU_config_package::IMU_ZUPT_Lidar_Config &config, uint32_t level)
{
    //P0:
    g_cfg_zuptlidar.P0.Fai_yaw = config.ZUPT_Lidar_P0_Fai_yaw;
    g_cfg_zuptlidar.P0.Fai_pitch = config.ZUPT_Lidar_P1_Fai_pitch;
    g_cfg_zuptlidar.P0.Fai_roll = config.ZUPT_Lidar_P2_Fai_roll;
    g_cfg_zuptlidar.P0.delta_ve = config.ZUPT_Lidar_P3_delta_ve;
    g_cfg_zuptlidar.P0.delta_vn = config.ZUPT_Lidar_P4_delta_vn;
    g_cfg_zuptlidar.P0.delta_vu = config.ZUPT_Lidar_P5_delta_vu;
    g_cfg_zuptlidar.P0.delta_lan = config.ZUPT_Lidar_P6_delta_lan;
    g_cfg_zuptlidar.P0.delta_lon = config.ZUPT_Lidar_P7_delta_lon;
    g_cfg_zuptlidar.P0.delta_h = config.ZUPT_Lidar_P8_delta_h;
    g_cfg_zuptlidar.P0.delta_Lve = config.ZUPT_Lidar_P9_delta_Lve;
    g_cfg_zuptlidar.P0.delta_Lvn = config.ZUPT_Lidar_Q10_delta_Lvn;
    g_cfg_zuptlidar.P0.delta_Lvu = config.ZUPT_Lidar_Q11_delta_Lvu;
    g_cfg_zuptlidar.P0.delta_Llan = config.ZUPT_Lidar_Q12_delta_Llan;
    g_cfg_zuptlidar.P0.delta_Llon = config.ZUPT_Lidar_Q13_delta_Llon;
    g_cfg_zuptlidar.P0.delta_Lh = config.ZUPT_Lidar_Q14_delta_Lh;

    //q0:
    g_cfg_zuptlidar.q0.Fai_yaw = config.ZUPT_Lidar_Q0_Fai_yaw;
    g_cfg_zuptlidar.q0.Fai_pitch = config.ZUPT_Lidar_Q1_Fai_pitch;
    g_cfg_zuptlidar.q0.Fai_roll = config.ZUPT_Lidar_Q2_Fai_roll;
    g_cfg_zuptlidar.q0.delta_ve = config.ZUPT_Lidar_Q3_delta_ve;
    g_cfg_zuptlidar.q0.delta_vn = config.ZUPT_Lidar_Q4_delta_vn;
    g_cfg_zuptlidar.q0.delta_vu = config.ZUPT_Lidar_Q5_delta_vu;
    g_cfg_zuptlidar.q0.delta_lan = config.ZUPT_Lidar_Q6_delta_lan;
    g_cfg_zuptlidar.q0.delta_lon = config.ZUPT_Lidar_Q7_delta_lon;
    g_cfg_zuptlidar.q0.delta_h = config.ZUPT_Lidar_Q8_delta_h;
    g_cfg_zuptlidar.q0.delta_Lve = config.ZUPT_Lidar_Q9_delta_Lve;
    g_cfg_zuptlidar.q0.delta_Lvn = config.ZUPT_Lidar_Q10_delta_Lvn;
    g_cfg_zuptlidar.q0.delta_Lvu = config.ZUPT_Lidar_Q12_delta_Llan;
    g_cfg_zuptlidar.q0.delta_Llan = config.ZUPT_Lidar_Q12_delta_Llan;
    g_cfg_zuptlidar.q0.delta_Llon = config.ZUPT_Lidar_Q13_delta_Llon;
    g_cfg_zuptlidar.q0.delta_Lh = config.ZUPT_Lidar_Q14_delta_Lh;

    //X0:
    g_cfg_zuptlidar.X0.Fai_yaw = config.ZUPT_Lidar_X00_Fai_yaw;
    g_cfg_zuptlidar.X0.Fai_pitch = config.ZUPT_Lidar_X01_Fai_pitch;
    g_cfg_zuptlidar.X0.Fai_roll = config.ZUPT_Lidar_X02_Fai_roll;
    g_cfg_zuptlidar.X0.delta_ve = config.ZUPT_Lidar_X03_delta_ve;
    g_cfg_zuptlidar.X0.delta_vn = config.ZUPT_Lidar_X04_delta_vn;
    g_cfg_zuptlidar.X0.delta_vu = config.ZUPT_Lidar_X05_delta_vu;
    g_cfg_zuptlidar.X0.delta_lan = config.ZUPT_Lidar_X06_delta_lan;
    g_cfg_zuptlidar.X0.delta_lon = config.ZUPT_Lidar_X07_delta_lon;
    g_cfg_zuptlidar.X0.delta_h = config.ZUPT_Lidar_X08_delta_h;
    g_cfg_zuptlidar.X0.delta_Lve = config.ZUPT_Lidar_X09_delta_Lve;
    g_cfg_zuptlidar.X0.delta_Lvn = config.ZUPT_Lidar_X010_delta_Lvn;
    g_cfg_zuptlidar.X0.delta_Lvu = config.ZUPT_Lidar_X011_delta_Lvu;
    g_cfg_zuptlidar.X0.delta_Llan = config.ZUPT_Lidar_X012_delta_Llan;
    g_cfg_zuptlidar.X0.delta_Llon = config.ZUPT_Lidar_X013_delta_Llon;
    g_cfg_zuptlidar.X0.delta_Lh = config.ZUPT_Lidar_X014_delta_Lh;

    //ZUPTSTOP_R:
    g_cfg_zuptlidar.ZSTOP_R.delta_yaw = config.ZUPTSTOP_Lidar_R0_delta_yaw;
    g_cfg_zuptlidar.ZSTOP_R.delta_pitch = config.ZUPTSTOP_Lidar_R1_delta_pitch;
    g_cfg_zuptlidar.ZSTOP_R.delta_roll = config.ZUPTSTOP_Lidar_R2_delta_roll;
    g_cfg_zuptlidar.ZSTOP_R.delta_ve = config.ZUPTSTOP_Lidar_R3_delta_ve;
    g_cfg_zuptlidar.ZSTOP_R.delta_vn = config.ZUPTSTOP_Lidar_R4_delta_vn;
    g_cfg_zuptlidar.ZSTOP_R.delta_vu = config.ZUPTSTOP_Lidar_R5_delta_vu;
    g_cfg_zuptlidar.ZSTOP_R.delta_Lve = config.ZUPTSTOP_Lidar_R6_delta_Lve;
    g_cfg_zuptlidar.ZSTOP_R.delta_Lvn = config.ZUPTSTOP_Lidar_R7_delta_Lvn;
    g_cfg_zuptlidar.ZSTOP_R.delta_Lvu = config.ZUPTSTOP_Lidar_R8_delta_Lvu;
    g_cfg_zuptlidar.ZSTOP_R.delta_Clan = config.ZUPTSTOP_Lidar_R9_delta_Clan;
    g_cfg_zuptlidar.ZSTOP_R.delta_Clon = config.ZUPTSTOP_Lidar_R10_delta_Clon;
    g_cfg_zuptlidar.ZSTOP_R.delta_Ch = config.ZUPTSTOP_Lidar_R11_delta_Ch;
    //ZUPTLIN_R:
    g_cfg_zuptlidar.ZLIN_R.delta_yaw = config.ZUPTLIN_Lidar_R0_delta_yaw;
    g_cfg_zuptlidar.ZLIN_R.delta_pitch = config.ZUPTLIN_Lidar_R1_delta_pitch;
    g_cfg_zuptlidar.ZLIN_R.delta_roll = config.ZUPTLIN_Lidar_R2_delta_roll;
    g_cfg_zuptlidar.ZLIN_R.delta_Cve = config.ZUPTLIN_Lidar_R3_delta_Cve;
    g_cfg_zuptlidar.ZLIN_R.delta_Cvn = config.ZUPTLIN_Lidar_R4_delta_Cvn;
    g_cfg_zuptlidar.ZLIN_R.delta_Cvu = config.ZUPTLIN_Lidar_R5_delta_Cvu;
    g_cfg_zuptlidar.ZLIN_R.delta_Clan = config.ZUPTLIN_Lidar_R6_delta_Clan;
    g_cfg_zuptlidar.ZLIN_R.delta_Clon = config.ZUPTLIN_Lidar_R7_delta_Clon;
    g_cfg_zuptlidar.ZLIN_R.delta_Ch = config.ZUPTLIN_Lidar_R8_delta_Ch;
    g_cfg_zuptlidar.ZLIN_R.Bvx = config.ZUPTLIN_Lidar_R9_Bvx;
    g_cfg_zuptlidar.ZLIN_R.Bvz = config.ZUPTLIN_Lidar_R10_Bvz;

    //Tao:
    g_cfg_zuptlidar.Tao.delta_Lve = config.ZUPT_Lidar_Tao0_delta_Lve;
    g_cfg_zuptlidar.Tao.delta_Lvn = config.ZUPT_Lidar_Tao1_delta_Lvn;
    g_cfg_zuptlidar.Tao.delta_Lvu = config.ZUPT_Lidar_Tao2_delta_Lvu;
    g_cfg_zuptlidar.Tao.delta_Llan = config.ZUPT_Lidar_Tao3_delta_Llan;
    g_cfg_zuptlidar.Tao.delta_Llon = config.ZUPT_Lidar_Tao4_delta_Llon;
    g_cfg_zuptlidar.Tao.delta_Lh = config.ZUPT_Lidar_Tao5_delta_Lh;
    // ROS_INFO("IMU_LIDAR: %f %f", config.Lidar_P_0, config.Lidar_Q_0);
}

// void IMU_FLIDARcallback(IMU_config_package::IMU_Fixed_Lidar_Config &config, uint32_t level)
// {
//   // ROS_INFO("IMU_LIDAR: %f %f", config.Lidar_P_0, config.Lidar_Q_0);
// }

// IMU_ZUPT_FLidar_Config
// void IMU_ZUPT_FLIDARcallback(IMU_config_package::IMU_ZUPT_FLidar_Config &config, uint32_t level)
// {
//   // ROS_INFO("IMU_LIDAR: %f %f", config.Lidar_P_0, config.Lidar_Q_0);
// }

FusionCenter::FusionCenter(ros::NodeHandle &fusion_nh)
{
    //各传感器类初始化
    g_pmatch_for_time = new MatchForTime(this);
    g_pfsdr = new FusionDR(this);
    g_pfsuwb = new FusionUWB(this);
    g_pfsflidar = new FusionFixedLidar(this);
    g_pfslidar = new FusionLidar(this);
    g_pfszuptuwb = new FusionZUPTUWB(this);
    g_pfszuptflidar = new FusionZUPTFixedLidar(this);
    g_pfszuptlidar = new FusionZUPTLidar(this);
    g_pcalibimu = new CalibrateForIMU(this);

    //各传感器融合标志初始化
    g_drkint_flag = false;
    g_ukinit_flag = false;
    g_flkinit_flag = false;
    g_lkinit_flag = false;
    g_uzkinit_flag = false;
    g_flzkinit_flag = false;
    g_lzkinit_flag = false;
    g_calib_flag = c_init;

    //各传感器回调函数触发标志初始化
    IMU_GNSS_update_flag = false;
    DR_update_flag = false;
    UWB_update_flag = false;
    lidar_update_flag = false;
    fixed_lidar_update_flag = false;
    //记录组合导航buffer初始化
    memset(&g_vprecord_fuse, 0, sizeof(g_vprecord_fuse));
    g_record_match_count = 0; //用于记录高频传感器相邻20个数据的位置与速度

    //只有IMU数据进行外推的状态值初始化
    g_delta_IMUcalib.resize(9); //用于校正IMU姿态速度与位置的值,yaw,pitch,roll，ve,vn,vu,lan,lon,h
    g_IMUcalib_I.resize(9, 9);
    g_delta_IMUcalib = VectorXd::Zero(9);
    g_IMUcalib_I = MatrixXd::Zero(9, 9);

    //场端距离停位点位置初始化
    g_flidar_info.agv_distance.x = 10000;
    g_flidar_info.agv_distance.y = 10000;

    int g_IMU_count = 0; //记录纯惯导模式时长(根据测试情况可能不止于纯惯导模式)
    delta_cnt = 0;
    point_to = 0;
    sys_status_cnt = 0; //组合导航系统状态计数
    
    //融合数据时间序列初始化
    g_fusion_data.header.seq = 0;
    
    vector<double *> pose_match_utm(3);                  //用于记录高频传感器相邻3个数据的位置,UTM平面坐标系
    vector<double *> vel_match_utm(3);                   //用于记录高频传感器相邻3个数据的速度，UTM平面坐标系
    double low_freq_pose_match_utm[3] = {0.0, 0.0, 0.0}; //用于记录低频传感器当前的位置，UTM平面坐标系

    //------test20191010 控制单独输出类型
    uint8_t start_flag_test = 1;         //1:选择P2输出,2:选择slam输出


    //---test
    //g_lidar_pose_confidience_cfg = 0.95;
    //g_IMU_count = 2977;

    //订阅数据
    ros::Subscriber inte_nav_sub = fusion_nh.subscribe("/drivers/can_wr/imu_gnss_msg", 100, &FusionCenter::callbackForIntegrationNavigation, this); //订阅组合导航
    ros::Subscriber dr_sub = fusion_nh.subscribe("dr/pos",30,&FusionCenter::callbackForDR,this);
    ros::Subscriber uwb_sub = fusion_nh.subscribe("/drivers/localization/uwb_msg", 5, &FusionCenter::callbackForUWB, this);                       //订阅UWB
    ros::Subscriber flidar_sub = fusion_nh.subscribe("/drivers/localization/fixed_lidar_msg", 5, &FusionCenter::callbackForFixedLidar, this);     //订阅场端Lidar
    ros::Subscriber lidar_sub = fusion_nh.subscribe("/localization/lidar_msg", 5, &FusionCenter::callbackForLidar, this);
    ros::Subscriber control_sub = fusion_nh.subscribe("/plan/ad_status", 5, &FusionCenter::callbackForADStatus, this); //订阅Lidar

    //发布融合数据
    ros::Publisher fusion_pub = fusion_nh.advertise<location_msgs::FusionDataInfo>("/localization/fusion_msg", 1000);
    ros::Rate loop_rate(100); //至少要比P2的频率大
    while (ros::ok())
    {
        ros::spinOnce();
        //sys_status = IMU_GNSS_info.system_status & 0x0F; //系统状态

        //----test20191010
        if (start_flag_test == 1)       
        {
            if (sys_status_cnt > (buffer_size-1))
            {
                //从buffer里取GNSS_IMU数据
                location_msgs::FusionDataInfo IMU_GNSS_info;
                if (delta_cnt > (buffer_size-1))
                {
                    point_to = g_record_match_count-1;
                    IMU_GNSS_info = g_vprecord_fuse.at(point_to);
                    point_to=point_to+1;
                    delta_cnt = 0;
                    
                }
                else if (delta_cnt == 0 && point_to == buffer_size)
                {
                    //ROS_INFO("gnss_imu stop updating");
                    continue;
                }
                else
                {
                    // ROS_INFO("delta_cnt2=%u",delta_cnt);
                    // ROS_INFO("point_to=%u",point_to);
                    point_to = point_to - delta_cnt;
                    IMU_GNSS_info = g_vprecord_fuse.at(point_to);
                    point_to = point_to+1;  
                }
                delta_cnt = 0;
                g_fusion_data.satellite_status = IMU_GNSS_info.satellite_status;
                g_fusion_data.system_status = IMU_GNSS_info.system_status;

                uint8_t sat_status = 0;
                sat_status = IMU_GNSS_info.satellite_status;    //卫星状态
                
                //sat_status = 4; //----------test

                if (sat_status != 4 )    //根据实际情况可能不止于纯惯导模式
                { //纯惯导模式
                    g_IMU_count++;
                }
                else
                {
                    g_IMU_count = 0;
                    g_drkint_flag = false;
                    g_ukinit_flag = false;
                    g_flkinit_flag = false;
                    g_lkinit_flag = false;
                    g_uzkinit_flag = false;
                    g_flzkinit_flag = false;
                    g_lzkinit_flag = false;
                }

                
                // g_adstatus_info.running_status = 100;
                // IMU_GNSS_update_flag = true;
                // g_UWB_info.fuwb_valid_flag = false;
                // lidar_update_flag = false;
                                
                if (IMU_GNSS_update_flag == true)
                {
                    //if (g_IMU_count > g_imu_count_cfg && g_calib_flag == c_yes)       //最后用这个
                    if (g_IMU_count > 1000 && g_calib_flag == c_yes)
                    {
                        //UWB是否在有效区域范围内
                        if (UWB_update_flag == true && fixed_lidar_update_flag == false && lidar_update_flag == false)
                        //if (UWB_update_flag == true && lidar_update_flag == false)
                        {
                            //在有效区范围内,做融合
                            processUWBFuse(pose_match_utm, vel_match_utm);
                            //当前点优化
                            PoseResult h_fm;
                            h_fm.pos.lan = IMU_GNSS_info.pose_llh.x;
                            h_fm.pos.lon = IMU_GNSS_info.pose_llh.y;
                            h_fm.pos.h = IMU_GNSS_info.pose_llh.z;
                            h_fm.vel.venu.vx = IMU_GNSS_info.velocity.linear.x;
                            h_fm.vel.venu.vy = IMU_GNSS_info.velocity.linear.y;
                            h_fm.vel.venu.vz = IMU_GNSS_info.velocity.linear.z;
                            h_fm.vel.wxyz.wx = IMU_GNSS_info.velocity.angular.x;
                            h_fm.vel.wxyz.wy = IMU_GNSS_info.velocity.angular.y;
                            h_fm.vel.wxyz.wz = IMU_GNSS_info.velocity.angular.z;
                            h_fm.att.yaw = IMU_GNSS_info.yaw;
                            h_fm.att.pitch = IMU_GNSS_info.pitch;
                            h_fm.att.roll = IMU_GNSS_info.roll;
                            h_fm.accel.ax = IMU_GNSS_info.accel.linear.x;
                            h_fm.accel.ay = IMU_GNSS_info.accel.linear.y;
                            h_fm.accel.az = IMU_GNSS_info.accel.linear.z;

                            UTC time_match;
                            time_match.hour = IMU_GNSS_info.hour;
                            time_match.min = IMU_GNSS_info.min;
                            time_match.sec = IMU_GNSS_info.sec;
                            time_match.msec = IMU_GNSS_info.msec;
                            g_pcalibimu->g_last_tm = g_IMUlast_tm;
                            g_pcalibimu->calculateFilter(h_fm, time_match, g_fusion_data_calib);
                            g_IMUlast_tm = time_match;
                            transFusionLLHtoHarbourENU(g_fusion_data_calib);
                            g_calib_flag = c_no;
                            g_fusion_data.header.seq++;
                            g_fusion_data.header.stamp = IMU_GNSS_info.header.stamp;
                            fusion_pub.publish(g_fusion_data);
                            UWB_update_flag = false;
                        }
                        else if (fixed_lidar_update_flag == true && UWB_update_flag == false && lidar_update_flag == false)
                        {
                            //在有效区范围内,做融合
                            processFixedLidarFuse(pose_match_utm, vel_match_utm);
                            //当前点优化
                            PoseResult h_fm;
                            h_fm.pos.lan = IMU_GNSS_info.pose_llh.x;
                            h_fm.pos.lon = IMU_GNSS_info.pose_llh.y;
                            h_fm.pos.h = IMU_GNSS_info.pose_llh.z;
                            h_fm.vel.venu.vx = IMU_GNSS_info.velocity.linear.x;
                            h_fm.vel.venu.vy = IMU_GNSS_info.velocity.linear.y;
                            h_fm.vel.venu.vz = IMU_GNSS_info.velocity.linear.z;
                            h_fm.vel.wxyz.wx = IMU_GNSS_info.velocity.angular.x;
                            h_fm.vel.wxyz.wy = IMU_GNSS_info.velocity.angular.y;
                            h_fm.vel.wxyz.wz = IMU_GNSS_info.velocity.angular.z;
                            h_fm.att.yaw = IMU_GNSS_info.yaw;
                            h_fm.att.pitch = IMU_GNSS_info.pitch;
                            h_fm.att.roll = IMU_GNSS_info.roll;
                            h_fm.accel.ax = IMU_GNSS_info.accel.linear.x;
                            h_fm.accel.ay = IMU_GNSS_info.accel.linear.y;
                            h_fm.accel.az = IMU_GNSS_info.accel.linear.z;

                            UTC time_match;
                            time_match.hour = IMU_GNSS_info.hour;
                            time_match.min = IMU_GNSS_info.min;
                            time_match.sec = IMU_GNSS_info.sec;
                            time_match.msec = IMU_GNSS_info.msec;
                            g_pcalibimu->g_last_tm = g_IMUlast_tm;
                            g_pcalibimu->calculateFilter(h_fm, time_match, g_fusion_data_calib);
                            g_IMUlast_tm = time_match;
                            transFusionLLHtoHarbourENU(g_fusion_data_calib);
                            g_calib_flag = c_no;
                            g_fusion_data.header.seq++;
                            g_fusion_data.header.stamp = IMU_GNSS_info.header.stamp;
                            fusion_pub.publish(g_fusion_data);
                            fixed_lidar_update_flag = false;
                            // ROS_INFO("yaw=%f,pitch=%f,roll=%f ", g_fusion_data.yaw, g_fusion_data.pitch, g_fusion_data.roll);
                            // ROS_INFO("ve=%f,vn=%f,vh=%f ", g_fusion_data.velocity.linear.x, g_fusion_data.velocity.linear.y, g_fusion_data.velocity.linear.z);
                            // ROS_INFO("lan=%.8f,lon=%.8f,h=%.8f ", g_fusion_data.pose_llh.x, g_fusion_data.pose_llh.y, g_fusion_data.pose_llh.z);
                            // ROS_INFO("le=%f,ln=%f,lu=%f ", g_fusion_data.pose.x, g_fusion_data.pose.y, g_fusion_data.pose.z);
                        }
                        else if (lidar_update_flag == true && fixed_lidar_update_flag == false && UWB_update_flag == false)
                        {
                            processLidarFuse(pose_match_utm, vel_match_utm);
                            //当前点优化
                            PoseResult h_fm;
                            h_fm.pos.lan = IMU_GNSS_info.pose_llh.x;
                            h_fm.pos.lon = IMU_GNSS_info.pose_llh.y;
                            h_fm.pos.h = IMU_GNSS_info.pose_llh.z;
                            h_fm.vel.venu.vx = IMU_GNSS_info.velocity.linear.x;
                            h_fm.vel.venu.vy = IMU_GNSS_info.velocity.linear.y;
                            h_fm.vel.venu.vz = IMU_GNSS_info.velocity.linear.z;
                            h_fm.vel.wxyz.wx = IMU_GNSS_info.velocity.angular.x;
                            h_fm.vel.wxyz.wy = IMU_GNSS_info.velocity.angular.y;
                            h_fm.vel.wxyz.wz = IMU_GNSS_info.velocity.angular.z;
                            h_fm.att.yaw = IMU_GNSS_info.yaw;
                            h_fm.att.pitch = IMU_GNSS_info.pitch;
                            h_fm.att.roll = IMU_GNSS_info.roll;
                            h_fm.accel.ax = IMU_GNSS_info.accel.linear.x;
                            h_fm.accel.ay = IMU_GNSS_info.accel.linear.y;
                            h_fm.accel.az = IMU_GNSS_info.accel.linear.z;

                            UTC time_match;
                            time_match.hour = IMU_GNSS_info.hour;
                            time_match.min = IMU_GNSS_info.min;
                            time_match.sec = IMU_GNSS_info.sec;
                            time_match.msec = IMU_GNSS_info.msec;
                            g_pcalibimu->g_last_tm = g_IMUlast_tm;
                            g_pcalibimu->calculateFilter(h_fm, time_match, g_fusion_data_calib);
                            g_IMUlast_tm = time_match;
                            transFusionLLHtoHarbourENU(g_fusion_data_calib);
                            g_calib_flag = c_no;
                            g_fusion_data.header.seq++;
                            g_fusion_data.header.stamp = IMU_GNSS_info.header.stamp;
                            fusion_pub.publish(g_fusion_data);
                            lidar_update_flag = false;
                            // ROS_INFO("yaw=%f,pitch=%f,roll=%f ", g_fusion_data.yaw, g_fusion_data.pitch, g_fusion_data.roll);
                            // ROS_INFO("ve=%f,vn=%f,vh=%f ", g_fusion_data.velocity.linear.x, g_fusion_data.velocity.linear.y, g_fusion_data.velocity.linear.z);
                            // ROS_INFO("lan=%.8f,lon=%.8f,h=%.8f ", g_fusion_data.pose_llh.x, g_fusion_data.pose_llh.y, g_fusion_data.pose_llh.z);
                            // ROS_INFO("le=%f,ln=%f,lu=%f ", g_fusion_data.pose.x, g_fusion_data.pose.y, g_fusion_data.pose.z);
                        }
                        else if (DR_update_flag == true && lidar_update_flag == false && fixed_lidar_update_flag == false && UWB_update_flag == false)
                        {
                            //融合点优化
                            processDRFuse(pose_match_utm, vel_match_utm);
                            //当前点优化
                            PoseResult h_fm;
                            h_fm.pos.lan = IMU_GNSS_info.pose_llh.x;
                            h_fm.pos.lon = IMU_GNSS_info.pose_llh.y;
                            h_fm.pos.h = IMU_GNSS_info.pose_llh.z;
                            h_fm.vel.venu.vx = IMU_GNSS_info.velocity.linear.x;
                            h_fm.vel.venu.vy = IMU_GNSS_info.velocity.linear.y;
                            h_fm.vel.venu.vz = IMU_GNSS_info.velocity.linear.z;
                            h_fm.vel.wxyz.wx = IMU_GNSS_info.velocity.angular.x;
                            h_fm.vel.wxyz.wy = IMU_GNSS_info.velocity.angular.y;
                            h_fm.vel.wxyz.wz = IMU_GNSS_info.velocity.angular.z;
                            h_fm.att.yaw = IMU_GNSS_info.yaw;
                            h_fm.att.pitch = IMU_GNSS_info.pitch;
                            h_fm.att.roll = IMU_GNSS_info.roll;
                            h_fm.accel.ax = IMU_GNSS_info.accel.linear.x;
                            h_fm.accel.ay = IMU_GNSS_info.accel.linear.y;
                            h_fm.accel.az = IMU_GNSS_info.accel.linear.z;

                            UTC time_match;
                            time_match.hour = IMU_GNSS_info.hour;
                            time_match.min = IMU_GNSS_info.min;
                            time_match.sec = IMU_GNSS_info.sec;
                            time_match.msec = IMU_GNSS_info.msec;
                            g_pcalibimu->g_last_tm = g_IMUlast_tm;
                            g_pcalibimu->calculateFilter(h_fm, time_match, g_fusion_data_calib);
                            g_IMUlast_tm = time_match;
                            transFusionLLHtoHarbourENU(g_fusion_data_calib);
                            g_calib_flag = c_no;
                            g_fusion_data.header.seq++;
                            g_fusion_data.header.stamp = IMU_GNSS_info.header.stamp;
                            fusion_pub.publish(g_fusion_data);
                            DR_update_flag = false;
                        }
                    }
                    //else if (g_IMU_count > g_imu_count_cfg && g_calib_flag == c_no)   //最后用这个
                    else if (g_IMU_count > 1000 && g_calib_flag == c_no)
                    {
                        processIMUCalibrate();
                        g_fusion_data.header.seq++;
                        g_fusion_data.header.stamp = IMU_GNSS_info.header.stamp;
                        fusion_pub.publish(g_fusion_data);
                    }
                    else
                    {
                        g_fusion_data = IMU_GNSS_info;
                        // g_fusion_data.pose_llh.x = 29.9381679;
                        // g_fusion_data.pose_llh.y = 121.9369869;
                        // g_fusion_data.pose_llh.z = 19.284;
                        // g_fusion_data.yaw = 56.43;
                        transFusionLLHtoHarbourENU(g_fusion_data);
                        g_fusion_data.header.seq++;
                        g_fusion_data.header.stamp = IMU_GNSS_info.header.stamp;
                        g_fusion_data.pose_confidence = 1;
                        fusion_pub.publish(g_fusion_data);
                        // ROS_INFO("yaw=%f,pitch=%f,roll=%f ",g_fusion_data.yaw,g_fusion_data.pitch,g_fusion_data.roll);
                        // ROS_INFO("vx=%f,vy=%f,vz=%f ",g_fusion_data.velocity.linear.x,g_fusion_data.velocity.linear.y,g_fusion_data.velocity.linear.z);
                        ROS_INFO("lan=%.8f,lon=%.8f,h=%.8f ",g_fusion_data.pose_llh.x,g_fusion_data.pose_llh.y,g_fusion_data.pose_llh.z);
                        ROS_INFO("hx=%f,hy=%f,hz=%f ",g_fusion_data.pose.x,g_fusion_data.pose.y,g_fusion_data.pose.z);
                    }
                    IMU_GNSS_update_flag = false;
                }
                else
                {
                    //ROS_INFO("WARNIMG:IMU_GNSS stop updating");
                    continue;
                }
            
                sys_status_cnt = buffer_size + 1;   //为防止sys_status_cnt超出int型容量
            }
        }
        else if (start_flag_test == 2)
        {
            if (lidar_update_flag == true)
            {
                g_fusion_data.pose_llh.x = g_lidar_info.pose_cov.pose.position.x;
                g_fusion_data.pose_llh.y = g_lidar_info.pose_cov.pose.position.y;
                g_fusion_data.pose_llh.z = g_lidar_info.pose_cov.pose.position.z;
                g_fusion_data.velocity.linear.x = g_lidar_info.vel_cov.twist.linear.x;
                g_fusion_data.velocity.linear.y = g_lidar_info.vel_cov.twist.linear.y;
                g_fusion_data.velocity.linear.z = g_lidar_info.vel_cov.twist.linear.z;
                g_fusion_data.velocity.angular.x = g_lidar_info.vel_cov.twist.angular.x;
                g_fusion_data.velocity.angular.y = g_lidar_info.vel_cov.twist.angular.y;
                g_fusion_data.velocity.angular.z = g_lidar_info.vel_cov.twist.angular.z;
                g_fusion_data.yaw = g_lidar_info.yaw;
                g_fusion_data.pitch = g_lidar_info.pitch;
                g_fusion_data.roll = g_lidar_info.roll;
                transFusionLLHtoHarbourENU(g_fusion_data);
                g_fusion_data.header.seq++;
                g_fusion_data.header.stamp = g_lidar_info.header.stamp;
                fusion_pub.publish(g_fusion_data);
            }
            else
            {
                //ROS_INFO("WARNIMG:Lidar SLAM stop updating");
                continue;
            }
        }

        //--------最后应用20191010
        // if (sys_status_cnt > (buffer_size-1))
        // {
        //     //从buffer里取GNSS_IMU数据
        //     location_msgs::FusionDataInfo IMU_GNSS_info;
        //     if (delta_cnt > (buffer_size-1))
        //     {
        //         point_to = g_record_match_count-1;
        //         IMU_GNSS_info = g_vprecord_fuse.at(point_to);
        //         point_to=point_to+1;
        //         delta_cnt = 0;
                
        //     }
        //     else if (delta_cnt == 0 && point_to == buffer_size)
        //     {
        //         //ROS_INFO("gnss_imu stop updating");
        //         continue;
        //     }
        //     else
        //     {
        //         // ROS_INFO("delta_cnt2=%u",delta_cnt);
        //         // ROS_INFO("point_to=%u",point_to);
        //         point_to = point_to - delta_cnt;
        //         IMU_GNSS_info = g_vprecord_fuse.at(point_to);
        //         point_to = point_to+1;  
        //     }
        //     delta_cnt = 0;
        //     g_fusion_data.satellite_status = IMU_GNSS_info.satellite_status;
        //     g_fusion_data.system_status = IMU_GNSS_info.system_status;

        //     uint8_t sat_status = 0;
        //     sat_status = IMU_GNSS_info.satellite_status;    //卫星状态
            
        //     //sat_status = 4; //----------test

        //     if (sat_status != 4 )    //根据实际情况可能不止于纯惯导模式
        //     { //纯惯导模式
        //         g_IMU_count++;
        //     }
        //     else
        //     {
        //         g_IMU_count = 0;
        //         g_drkint_flag = false;
        //         g_ukinit_flag = false;
        //         g_flkinit_flag = false;
        //         g_lkinit_flag = false;
        //         g_uzkinit_flag = false;
        //         g_flzkinit_flag = false;
        //         g_lzkinit_flag = false;
        //     }

            
        //     // g_adstatus_info.running_status = 100;
        //     // IMU_GNSS_update_flag = true;
        //     // g_UWB_info.fuwb_valid_flag = false;
        //     // lidar_update_flag = false;
                              
        //     if (IMU_GNSS_update_flag == true)
        //     {
        //         //if (g_IMU_count > g_imu_count_cfg && g_calib_flag == c_yes)       //最后用这个
        //         if (g_IMU_count > 1000 && g_calib_flag == c_yes)
        //         {
        //             //UWB是否在有效区域范围内
        //             if (UWB_update_flag == true && fixed_lidar_update_flag == false && lidar_update_flag == false)
        //             //if (UWB_update_flag == true && lidar_update_flag == false)
        //             {
        //                 //在有效区范围内,做融合
        //                 processUWBFuse(pose_match_utm, vel_match_utm);
        //                 //当前点优化
        //                 PoseResult h_fm;
        //                 h_fm.pos.lan = IMU_GNSS_info.pose_llh.x;
        //                 h_fm.pos.lon = IMU_GNSS_info.pose_llh.y;
        //                 h_fm.pos.h = IMU_GNSS_info.pose_llh.z;
        //                 h_fm.vel.venu.vx = IMU_GNSS_info.velocity.linear.x;
        //                 h_fm.vel.venu.vy = IMU_GNSS_info.velocity.linear.y;
        //                 h_fm.vel.venu.vz = IMU_GNSS_info.velocity.linear.z;
        //                 h_fm.vel.wxyz.wx = IMU_GNSS_info.velocity.angular.x;
        //                 h_fm.vel.wxyz.wy = IMU_GNSS_info.velocity.angular.y;
        //                 h_fm.vel.wxyz.wz = IMU_GNSS_info.velocity.angular.z;
        //                 h_fm.att.yaw = IMU_GNSS_info.yaw;
        //                 h_fm.att.pitch = IMU_GNSS_info.pitch;
        //                 h_fm.att.roll = IMU_GNSS_info.roll;
        //                 h_fm.accel.ax = IMU_GNSS_info.accel.linear.x;
        //                 h_fm.accel.ay = IMU_GNSS_info.accel.linear.y;
        //                 h_fm.accel.az = IMU_GNSS_info.accel.linear.z;

        //                 UTC time_match;
        //                 time_match.hour = IMU_GNSS_info.hour;
        //                 time_match.min = IMU_GNSS_info.min;
        //                 time_match.sec = IMU_GNSS_info.sec;
        //                 time_match.msec = IMU_GNSS_info.msec;
        //                 g_pcalibimu->g_last_tm = g_IMUlast_tm;
        //                 g_pcalibimu->calculateFilter(h_fm, time_match, g_fusion_data_calib);
        //                 g_IMUlast_tm = time_match;
        //                 transFusionLLHtoHarbourENU(g_fusion_data_calib);
        //                 g_calib_flag = c_no;
        //                 g_fusion_data.header.seq++;
        //                 g_fusion_data.header.stamp = IMU_GNSS_info.header.stamp;
        //                 fusion_pub.publish(g_fusion_data);
        //                 UWB_update_flag = false;
        //             }
        //             else if (fixed_lidar_update_flag == true && UWB_update_flag == false && lidar_update_flag == false)
        //             {
        //                 //在有效区范围内,做融合
        //                 processFixedLidarFuse(pose_match_utm, vel_match_utm);
        //                 //当前点优化
        //                 PoseResult h_fm;
        //                 h_fm.pos.lan = IMU_GNSS_info.pose_llh.x;
        //                 h_fm.pos.lon = IMU_GNSS_info.pose_llh.y;
        //                 h_fm.pos.h = IMU_GNSS_info.pose_llh.z;
        //                 h_fm.vel.venu.vx = IMU_GNSS_info.velocity.linear.x;
        //                 h_fm.vel.venu.vy = IMU_GNSS_info.velocity.linear.y;
        //                 h_fm.vel.venu.vz = IMU_GNSS_info.velocity.linear.z;
        //                 h_fm.vel.wxyz.wx = IMU_GNSS_info.velocity.angular.x;
        //                 h_fm.vel.wxyz.wy = IMU_GNSS_info.velocity.angular.y;
        //                 h_fm.vel.wxyz.wz = IMU_GNSS_info.velocity.angular.z;
        //                 h_fm.att.yaw = IMU_GNSS_info.yaw;
        //                 h_fm.att.pitch = IMU_GNSS_info.pitch;
        //                 h_fm.att.roll = IMU_GNSS_info.roll;
        //                 h_fm.accel.ax = IMU_GNSS_info.accel.linear.x;
        //                 h_fm.accel.ay = IMU_GNSS_info.accel.linear.y;
        //                 h_fm.accel.az = IMU_GNSS_info.accel.linear.z;

        //                 UTC time_match;
        //                 time_match.hour = IMU_GNSS_info.hour;
        //                 time_match.min = IMU_GNSS_info.min;
        //                 time_match.sec = IMU_GNSS_info.sec;
        //                 time_match.msec = IMU_GNSS_info.msec;
        //                 g_pcalibimu->g_last_tm = g_IMUlast_tm;
        //                 g_pcalibimu->calculateFilter(h_fm, time_match, g_fusion_data_calib);
        //                 g_IMUlast_tm = time_match;
        //                 transFusionLLHtoHarbourENU(g_fusion_data_calib);
        //                 g_calib_flag = c_no;
        //                 g_fusion_data.header.seq++;
        //                 g_fusion_data.header.stamp = IMU_GNSS_info.header.stamp;
        //                 fusion_pub.publish(g_fusion_data);
        //                 fixed_lidar_update_flag = false;
        //                 // ROS_INFO("yaw=%f,pitch=%f,roll=%f ", g_fusion_data.yaw, g_fusion_data.pitch, g_fusion_data.roll);
        //                 // ROS_INFO("ve=%f,vn=%f,vh=%f ", g_fusion_data.velocity.linear.x, g_fusion_data.velocity.linear.y, g_fusion_data.velocity.linear.z);
        //                 // ROS_INFO("lan=%.8f,lon=%.8f,h=%.8f ", g_fusion_data.pose_llh.x, g_fusion_data.pose_llh.y, g_fusion_data.pose_llh.z);
        //                 // ROS_INFO("le=%f,ln=%f,lu=%f ", g_fusion_data.pose.x, g_fusion_data.pose.y, g_fusion_data.pose.z);
        //             }
        //             else if (lidar_update_flag == true && fixed_lidar_update_flag == false && UWB_update_flag == false)
        //             {
        //                 processLidarFuse(pose_match_utm, vel_match_utm);
        //                 //当前点优化
        //                 PoseResult h_fm;
        //                 h_fm.pos.lan = IMU_GNSS_info.pose_llh.x;
        //                 h_fm.pos.lon = IMU_GNSS_info.pose_llh.y;
        //                 h_fm.pos.h = IMU_GNSS_info.pose_llh.z;
        //                 h_fm.vel.venu.vx = IMU_GNSS_info.velocity.linear.x;
        //                 h_fm.vel.venu.vy = IMU_GNSS_info.velocity.linear.y;
        //                 h_fm.vel.venu.vz = IMU_GNSS_info.velocity.linear.z;
        //                 h_fm.vel.wxyz.wx = IMU_GNSS_info.velocity.angular.x;
        //                 h_fm.vel.wxyz.wy = IMU_GNSS_info.velocity.angular.y;
        //                 h_fm.vel.wxyz.wz = IMU_GNSS_info.velocity.angular.z;
        //                 h_fm.att.yaw = IMU_GNSS_info.yaw;
        //                 h_fm.att.pitch = IMU_GNSS_info.pitch;
        //                 h_fm.att.roll = IMU_GNSS_info.roll;
        //                 h_fm.accel.ax = IMU_GNSS_info.accel.linear.x;
        //                 h_fm.accel.ay = IMU_GNSS_info.accel.linear.y;
        //                 h_fm.accel.az = IMU_GNSS_info.accel.linear.z;

        //                 UTC time_match;
        //                 time_match.hour = IMU_GNSS_info.hour;
        //                 time_match.min = IMU_GNSS_info.min;
        //                 time_match.sec = IMU_GNSS_info.sec;
        //                 time_match.msec = IMU_GNSS_info.msec;
        //                 g_pcalibimu->g_last_tm = g_IMUlast_tm;
        //                 g_pcalibimu->calculateFilter(h_fm, time_match, g_fusion_data_calib);
        //                 g_IMUlast_tm = time_match;
        //                 transFusionLLHtoHarbourENU(g_fusion_data_calib);
        //                 g_calib_flag = c_no;
        //                 g_fusion_data.header.seq++;
        //                 g_fusion_data.header.stamp = IMU_GNSS_info.header.stamp;
        //                 fusion_pub.publish(g_fusion_data);
        //                 lidar_update_flag = false;
        //                 // ROS_INFO("yaw=%f,pitch=%f,roll=%f ", g_fusion_data.yaw, g_fusion_data.pitch, g_fusion_data.roll);
        //                 // ROS_INFO("ve=%f,vn=%f,vh=%f ", g_fusion_data.velocity.linear.x, g_fusion_data.velocity.linear.y, g_fusion_data.velocity.linear.z);
        //                 // ROS_INFO("lan=%.8f,lon=%.8f,h=%.8f ", g_fusion_data.pose_llh.x, g_fusion_data.pose_llh.y, g_fusion_data.pose_llh.z);
        //                 // ROS_INFO("le=%f,ln=%f,lu=%f ", g_fusion_data.pose.x, g_fusion_data.pose.y, g_fusion_data.pose.z);
        //             }
        //             else if (DR_update_flag == true && lidar_update_flag == false && fixed_lidar_update_flag == false && UWB_update_flag == false)
        //             {
        //                 //融合点优化
        //                 processDRFuse(pose_match_utm, vel_match_utm);
        //                 //当前点优化
        //                 PoseResult h_fm;
        //                 h_fm.pos.lan = IMU_GNSS_info.pose_llh.x;
        //                 h_fm.pos.lon = IMU_GNSS_info.pose_llh.y;
        //                 h_fm.pos.h = IMU_GNSS_info.pose_llh.z;
        //                 h_fm.vel.venu.vx = IMU_GNSS_info.velocity.linear.x;
        //                 h_fm.vel.venu.vy = IMU_GNSS_info.velocity.linear.y;
        //                 h_fm.vel.venu.vz = IMU_GNSS_info.velocity.linear.z;
        //                 h_fm.vel.wxyz.wx = IMU_GNSS_info.velocity.angular.x;
        //                 h_fm.vel.wxyz.wy = IMU_GNSS_info.velocity.angular.y;
        //                 h_fm.vel.wxyz.wz = IMU_GNSS_info.velocity.angular.z;
        //                 h_fm.att.yaw = IMU_GNSS_info.yaw;
        //                 h_fm.att.pitch = IMU_GNSS_info.pitch;
        //                 h_fm.att.roll = IMU_GNSS_info.roll;
        //                 h_fm.accel.ax = IMU_GNSS_info.accel.linear.x;
        //                 h_fm.accel.ay = IMU_GNSS_info.accel.linear.y;
        //                 h_fm.accel.az = IMU_GNSS_info.accel.linear.z;

        //                 UTC time_match;
        //                 time_match.hour = IMU_GNSS_info.hour;
        //                 time_match.min = IMU_GNSS_info.min;
        //                 time_match.sec = IMU_GNSS_info.sec;
        //                 time_match.msec = IMU_GNSS_info.msec;
        //                 g_pcalibimu->g_last_tm = g_IMUlast_tm;
        //                 g_pcalibimu->calculateFilter(h_fm, time_match, g_fusion_data_calib);
        //                 g_IMUlast_tm = time_match;
        //                 transFusionLLHtoHarbourENU(g_fusion_data_calib);
        //                 g_calib_flag = c_no;
        //                 g_fusion_data.header.seq++;
        //                 g_fusion_data.header.stamp = IMU_GNSS_info.header.stamp;
        //                 fusion_pub.publish(g_fusion_data);
        //                 DR_update_flag = false;
        //             }
        //         }
        //         //else if (g_IMU_count > g_imu_count_cfg && g_calib_flag == c_no)   //最后用这个
        //         else if (g_IMU_count > 1000 && g_calib_flag == c_no)
        //         {
        //             processIMUCalibrate();
        //             g_fusion_data.header.seq++;
        //             g_fusion_data.header.stamp = IMU_GNSS_info.header.stamp;
        //             fusion_pub.publish(g_fusion_data);
        //         }
        //         else
        //         {
        //             g_fusion_data = IMU_GNSS_info;
        //             transFusionLLHtoHarbourENU(g_fusion_data);
        //             g_fusion_data.header.seq++;
        //             g_fusion_data.header.stamp = IMU_GNSS_info.header.stamp;
        //             g_fusion_data.pose_confidence = 1;
        //             fusion_pub.publish(g_fusion_data);
        //             // ROS_INFO("yaw=%f,pitch=%f,roll=%f ",g_fusion_data.yaw,g_fusion_data.pitch,g_fusion_data.roll);
        //             // ROS_INFO("vx=%f,vy=%f,vz=%f ",g_fusion_data.velocity.linear.x,g_fusion_data.velocity.linear.y,g_fusion_data.velocity.linear.z);
        //             // ROS_INFO("lan=%.8f,lon=%.8f,h=%.8f ",g_fusion_data.pose_llh.x,g_fusion_data.pose_llh.y,g_fusion_data.pose_llh.z);
        //             // ROS_INFO("hx=%f,hy=%f,hz=%f ",g_fusion_data.pose.x,g_fusion_data.pose.y,g_fusion_data.pose.z);
        //         }
        //         IMU_GNSS_update_flag = false;
        //     }
        //     else
        //     {
        //         //ROS_INFO("WARNIMG:IMU_GNSS stop updating");
        //         continue;
        //     }
        
        //     sys_status_cnt = buffer_size + 1;   //为防止sys_status_cnt超出int型容量
        // }
        //--------最后应用20191010

        loop_rate.sleep();
    }
}

FusionCenter::~FusionCenter(void)
{
    if (g_pmatch_for_time)
    {
        delete g_pmatch_for_time;
    }
    if (g_pfsdr)
    {
        delete g_pfsdr;
    }
    if (g_pfsuwb)
    {
        delete g_pfsuwb;
    }
    if (g_pfsflidar)
    {
        delete g_pfsflidar;
    }
    if (g_pfslidar)
    {
        delete g_pfslidar;
    }
    if (g_pfszuptuwb)
    {
        delete g_pfszuptuwb;
    }
    if (g_pfszuptflidar)
    {
        delete g_pfszuptflidar;
    }
    if (g_pfszuptlidar)
    {
        delete g_pfszuptlidar;
    }
    if (g_pcalibimu)
    {
        delete g_pcalibimu;
    }
}

// **************
// 功能:组合导航的回调函数
// 输入:integrated_nav_msgs::IMUAndGNSSInfoConstPtr型信息 inte_nav_info
// 输出:integrated_nav_msgs::IMUAndGNSSInfo型信息 g_IMU_GNSS_info
// 无返回
// ***************
void FusionCenter::callbackForIntegrationNavigation(const location_sensor_msgs::IMUAndGNSSInfoConstPtr &inte_nav_info)
{
    IMU_GNSS_update_flag = true;
    g_IMU_GNSS_info.year = inte_nav_info->year;
    g_IMU_GNSS_info.month = inte_nav_info->month;
    g_IMU_GNSS_info.day = inte_nav_info->day;
    g_IMU_GNSS_info.hour = inte_nav_info->hour;
    g_IMU_GNSS_info.min = inte_nav_info->min;
    g_IMU_GNSS_info.sec = inte_nav_info->sec;
    g_IMU_GNSS_info.msec = inte_nav_info->msec;
    g_IMU_GNSS_info.header = inte_nav_info->header;
    g_IMU_GNSS_info.system_status = inte_nav_info->system_status;
    g_IMU_GNSS_info.satellite_status = inte_nav_info->satellite_status;
    g_IMU_GNSS_info.pose_llh.x = inte_nav_info->pose.x;
    g_IMU_GNSS_info.pose_llh.y = inte_nav_info->pose.y;
    g_IMU_GNSS_info.pose_llh.z = inte_nav_info->pose.z;
    g_IMU_GNSS_info.yaw = inte_nav_info->yaw;
    g_IMU_GNSS_info.pitch = inte_nav_info->pitch;
    g_IMU_GNSS_info.roll = inte_nav_info->roll;
    g_IMU_GNSS_info.velocity.linear.x = inte_nav_info->velocity.x;
    g_IMU_GNSS_info.velocity.linear.y = inte_nav_info->velocity.y;
    g_IMU_GNSS_info.velocity.linear.z = inte_nav_info->velocity.z;
    g_IMU_GNSS_info.velocity.angular.x = inte_nav_info->accelgyro.angular.x;
    g_IMU_GNSS_info.velocity.angular.y = inte_nav_info->accelgyro.angular.y;
    g_IMU_GNSS_info.velocity.angular.z = inte_nav_info->accelgyro.angular.z;
    g_IMU_GNSS_info.accel.linear.x = inte_nav_info->accelgyro.linear.x;
    g_IMU_GNSS_info.accel.linear.y = inte_nav_info->accelgyro.linear.y;
    g_IMU_GNSS_info.accel.linear.z = inte_nav_info->accelgyro.linear.z;
    g_IMU_GNSS_info.fuse_state = 1;
    /*记录组合导航相邻100组数据的位置与速度*/
    recordIMUGNSSData();
}

void FusionCenter::callbackForDR(const location_sensor_msgs::DRInfoConstPtr &dr_info)
{
    if (dr_info->dr_valid_flag == 2)
    {
        g_DR_info.dr_valid_flag = dr_info->dr_valid_flag;
        g_DR_info.header = dr_info->header;
        g_DR_info.year = dr_info->year;
        g_DR_info.month = dr_info->month;
        g_DR_info.day = dr_info->day;
        g_DR_info.hour = dr_info->hour;
        g_DR_info.min = dr_info->min;
        g_DR_info.sec = dr_info->sec;
        g_DR_info.msec = dr_info->msec;
        g_DR_info.heading = dr_info->heading;   //目前给不了
        g_DR_info.pitch = dr_info->pitch;       //目前给不了
        g_DR_info.roll = dr_info->roll;         //目前给不了
        g_DR_info.x = dr_info->x;               //港区坐标系下x
        g_DR_info.y = dr_info->y;               //港区坐标系下y
        g_DR_info.z = dr_info->z;               //港区坐标系下z,目前给不了
        g_DR_info.velocity_east = dr_info->velocity_east;
        g_DR_info.velocity_north = dr_info->velocity_north;
        g_DR_info.velocity_up = dr_info->velocity_up;   //目前给不了
        DR_update_flag = true;
        g_calib_flag = c_yes;
    }
    else
    {
        DR_update_flag = false;
        g_calib_flag = c_no;
    }
    
    
}

// **************
// 功能:UWB数据的回调函数
// 输入:fusion_data_msgs::UWBInfoConstPtr型信息 inte_nav_info
// 输出:fusion_data_msgs::UWBInfo型信息 g_IMU_GNSS_info
// 无返回
// ***************
void FusionCenter::callbackForUWB(const location_sensor_msgs::UWBInfoConstPtr &uwb_info)
{
    g_UWB_info.year = uwb_info->year;
    g_UWB_info.month = uwb_info->month;
    g_UWB_info.day = uwb_info->day;
    g_UWB_info.hour = uwb_info->hour;
    g_UWB_info.min = uwb_info->min;
    g_UWB_info.sec = uwb_info->sec;
    g_UWB_info.msec = uwb_info->msec;
    g_UWB_info.header = uwb_info->header;
    g_UWB_info.fuwb_valid_flag = uwb_info->fuwb_valid_flag;
    g_UWB_info.pose_cov = uwb_info->pose_cov;
    g_UWB_info.yaw = uwb_info->yaw;
    g_UWB_info.pitch = uwb_info->pitch;
    g_UWB_info.roll = uwb_info->roll;
    g_UWB_info.vel_cov = uwb_info->vel_cov;
    g_UWB_info.pose_confidence = uwb_info->pose_confidence;
    g_UWB_info.orien_confidence = uwb_info->orien_confidence;
    g_UWB_info.linear_confidence = uwb_info->linear_confidence;
    g_UWB_info.angular_confidence = uwb_info->angular_confidence;
    if (g_UWB_info.fuwb_valid_flag == true)
    {
        UWB_update_flag = true;
        g_calib_flag = c_yes;
    }
    else
    {
        UWB_update_flag = false;
        g_calib_flag = c_no;
    }
    
}

// **************
// 功能:场端lidar数据的回调函数
// 输入:fusion_data_msgs::FixedLidarInfoConstPtr &flidar_info
// 输出:fusion_data_msgs::FixedLidarInfo g_flidar_info
// 无返回
// ***************
void FusionCenter::callbackForFixedLidar(const location_sensor_msgs::FixedLidarInfoConstPtr &flidar_info)
{
    g_flidar_info.year = flidar_info->year;
    g_flidar_info.month = flidar_info->month;
    g_flidar_info.day = flidar_info->day;
    g_flidar_info.hour = flidar_info->hour;
    g_flidar_info.min = flidar_info->min;
    g_flidar_info.sec = flidar_info->sec;
    g_flidar_info.msec = flidar_info->msec;
    g_flidar_info.header = flidar_info->header;
    g_flidar_info.flidar_id = flidar_info->flidar_id;
    g_flidar_info.tager_position.x = flidar_info->tager_position.x;
    g_flidar_info.tager_position.y = flidar_info->tager_position.y;
    g_flidar_info.tager_position.z = flidar_info->tager_position.z;
    g_flidar_info.agv_distance.x = flidar_info->agv_distance.x;
    g_flidar_info.agv_distance.y = flidar_info->agv_distance.y;
    g_flidar_info.heading = flidar_info->heading;
    g_flidar_info.Vx = flidar_info->Vx;
    g_flidar_info.Vy = flidar_info->Vy;
    g_flidar_info.line_heading = flidar_info->line_heading;
    if (g_flidar_info.agv_distance.x <=10 && g_flidar_info.agv_distance.y <=10)   //!!!-----此判断条件根据实际情况改进
    {
        fixed_lidar_update_flag = true;
        g_calib_flag = c_yes;
    }
    else
    {
        fixed_lidar_update_flag = false;
        g_calib_flag = c_no;
    }
    
}

void FusionCenter::callbackForLidar(const location_sensor_msgs::LidarInfoConstPtr &lidar_info)
{
    g_lidar_info.year = lidar_info->year;
    g_lidar_info.month = lidar_info->month;
    g_lidar_info.day = lidar_info->day;
    g_lidar_info.hour = lidar_info->hour;
    g_lidar_info.min = lidar_info->min;
    g_lidar_info.sec = lidar_info->sec;
    g_lidar_info.msec = lidar_info->msec;
    g_lidar_info.header = lidar_info->header;
    g_lidar_info.pose_cov = lidar_info->pose_cov;
    g_lidar_info.yaw = lidar_info->yaw;
    g_lidar_info.pitch = lidar_info->pitch;
    g_lidar_info.roll = lidar_info->roll;
    g_lidar_info.vel_cov = lidar_info->vel_cov;
    g_lidar_info.pose_confidence = lidar_info->pose_confidence;
    g_lidar_info.orien_confidence = lidar_info->orien_confidence;
    g_lidar_info.linear_confidence = lidar_info->linear_confidence;
    g_lidar_info.angular_confidence = lidar_info->angular_confidence;
    if (g_lidar_info.pose_confidence >= g_lidar_pose_confidience_cfg)
    {
        lidar_update_flag = true;
        g_calib_flag = c_yes;
    }
    else
    {
        lidar_update_flag = false;
        g_calib_flag = c_no;
    }
}

void FusionCenter::callbackForADStatus(const hmi_msgs::ADStatusConstPtr &adstatus_info)
{
    g_adstatus_info.running_status = adstatus_info->running_status;
}

// **************
// 功能:记录高频传感器(组合导航)相邻100组数据的位置与速度
// 输入:无
// 输出:vector <location_msgs::FusionDataInfo> g_vprecord_fuse 10组IMU数据
// 无返回
// ***************
void FusionCenter::recordIMUGNSSData()
{
    delta_cnt=delta_cnt+1;
    //ROS_INFO("delta_cnt1=%u",delta_cnt);
    uint8_t sys_status = g_IMU_GNSS_info.system_status;
    if (sys_status == 2)
    {
        sys_status_cnt=sys_status_cnt+1;
        //ROS_INFO("sys_status_cnt=%u",sys_status_cnt);
    }
    if (g_record_match_count < buffer_size)
    {
        g_vprecord_fuse.push_back(g_IMU_GNSS_info);
        g_record_match_count=g_record_match_count+1;
    }
    else
    {
        for (int i = 0; i < buffer_size - 1; i++)
        {
            g_vprecord_fuse[i] = g_vprecord_fuse.at(i + 1);
        }
        g_vprecord_fuse.erase(g_vprecord_fuse.end()-1);
        g_vprecord_fuse.push_back(g_IMU_GNSS_info);
    }
}

// **************
// 功能:处理无零速修正组合导航与UWB的数据融合,并将融合位置转换为港区坐标输出
// 输入:vector <double*> pose_match_wgs用于记录高频传感器相邻3个数据的位置,WGS84坐标系
//     vector <double*> vel_match_wgs 用于记录高频传感器相邻3个数据的速度，WGS84坐标系
// 输出:无
// 无返回
// ***************
void FusionCenter::processUWBFuse(vector<double *> &pose_match_utm, vector<double *> &vel_match_utm)
{
    /*--------时间配准--------*/
    //ROS_INFO("Begin to UWBFuse");
    PoseResult low_freq_match;
    low_freq_match.att.yaw = g_UWB_info.yaw;
    low_freq_match.att.pitch = g_UWB_info.pitch;
    low_freq_match.vel.venu.vx = g_UWB_info.vel_cov.twist.linear.x;
    low_freq_match.vel.venu.vy = g_UWB_info.vel_cov.twist.linear.y;
    low_freq_match.pos.lan = g_UWB_info.pose_cov.pose.position.x;
    low_freq_match.pos.lon = g_UWB_info.pose_cov.pose.position.y;

    UTC low_fre_utc;
    low_fre_utc.hour = g_UWB_info.hour;
    low_fre_utc.min = g_UWB_info.min;
    low_fre_utc.sec = g_UWB_info.sec;
    low_fre_utc.msec = g_UWB_info.msec;

    double delta_update_time = 0;
    location_msgs::FusionDataInfo fuse_temp;
    bool update_date_flag = false;
    fuse_temp = g_record_fuse;
    int tm_delay = 0; //！！！这个对应通讯延时要改
    double lon0 = 0.0;
    findIMUDataForMatch(tm_delay, pose_match_utm, vel_match_utm,low_fre_utc,lon0);
    delta_update_time = g_record_fuse.hour * 3600 + g_record_fuse.min * 60 + g_record_fuse.sec + g_record_fuse.msec / 1000 -
                        (fuse_temp.hour * 3600 + fuse_temp.min * 60 + fuse_temp.sec + fuse_temp.msec / 1000);
    if (g_record_fuse.year == fuse_temp.year && g_record_fuse.month == fuse_temp.month && g_record_fuse.day == fuse_temp.day)
    {
        update_date_flag = false;
    }
    else
    {
        update_date_flag = true;
    }
    //判断何种匹配方式
    if (g_adstatus_info.running_status == 0) //！！！车辆状态可能会因为通讯延时有滞后性，后续再想解决办法
    {
        high_freq_type = integrated_nav;
        low_freq_type = uwb;
        g_pmatch_for_time->MatchForStop(g_high_freq_match);
    }
    else if (g_adstatus_info.running_status == 2)
    {
        high_freq_type = integrated_nav;
        low_freq_type = uwb;
        if (delta_update_time > 0.015 || update_date_flag == true)
        {
            g_pmatch_for_time->MatchForStop(g_high_freq_match);
        }
        else
        {
            g_pmatch_for_time->MatchForCircular(pose_match_utm, vel_match_utm, g_high_freq_match,lon0);
        }
    }
    else
    {
        high_freq_type = integrated_nav;
        low_freq_type = uwb;
        //--------test
        g_pmatch_for_time->MatchForStop(g_high_freq_match);
        // if (delta_update_time > 0.015 || update_date_flag == true)
        // {
        //     g_pmatch_for_time->MatchForStop(g_high_freq_match);
        // }
        // else
        // {
        //     g_pmatch_for_time->MatchForLinear(pose_match_utm, vel_match_utm, g_high_freq_match,lon0);
        // }
    }

    //卡尔曼滤波融合
    UTC time_match;
    time_match.year = g_UWB_info.year;
    time_match.month = g_UWB_info.month;
    time_match.day = g_UWB_info.day;
    time_match.hour = g_UWB_info.hour;
    time_match.min = g_UWB_info.min;
    time_match.sec = g_UWB_info.sec;
    time_match.msec = g_UWB_info.msec;
    VectorXd IMUcalib_(13);
    IMUcalib_ = VectorXd::Zero(13);
    MatrixXd IMUcalib_I_(13, 13);
    IMUcalib_I_ = MatrixXd::Zero(13, 13);
    //判断是否加入零速修正
    if (g_adstatus_info.running_status == 0)
    {
        zupt_type = zupt_stop;
        g_uzkinit_flag = g_pfszuptuwb->calculateFilter(g_high_freq_match, low_freq_match, time_match, g_fusion_data);
        IMUcalib_ = g_pfszuptuwb->g_fusion_lX;
        IMUcalib_I_ = g_pfszuptuwb->g_fusion_lI;
        g_IMUlast_tm = g_pfszuptuwb->g_last_tm;
    }
    else if (g_adstatus_info.running_status == 1 || g_adstatus_info.running_status == 3)
    {
        zupt_type = zupt_linear;
        g_uzkinit_flag = g_pfszuptuwb->calculateFilter(g_high_freq_match, low_freq_match, time_match, g_fusion_data);
        IMUcalib_ = g_pfszuptuwb->g_fusion_lX;
        IMUcalib_I_ = g_pfszuptuwb->g_fusion_lI;
        g_IMUlast_tm = g_pfszuptuwb->g_last_tm;
    }
    else
    {
        g_ukinit_flag = g_pfsuwb->calculateFilter(g_high_freq_match, low_freq_match, time_match, g_fusion_data);
        IMUcalib_ = g_pfsuwb->g_fusion_lX;
        IMUcalib_I_ = g_pfsuwb->g_fusion_lI;
        g_IMUlast_tm = g_pfsuwb->g_last_tm;
    }
    transFusionLLHtoHarbourENU(g_fusion_data);
    for (int i = 0; i < 9; i++)
    {
        g_delta_IMUcalib(i) = IMUcalib_(i);
        for (int j = 0; j < 9; j++)
        {
            g_IMUcalib_I(i, j) = IMUcalib_I_(i, j);
        }
    }
    g_pcalibimu->g_calib_I = g_IMUcalib_I;
    g_calib_flag = c_no;
    for (int i = 0; i < 3; i++)
    {
        free(pose_match_utm[i]);
        free(vel_match_utm[i]);
    }
}

// **************
// 功能:处理无零速修正组合导航与场端Lidar的数据融合,并将融合位置转换为港区坐标输出
// 输入:vector <double*> pose_match_utm用于记录高频传感器相邻3个数据的位置,UTM坐标系
//     vector <double*> vel_match_wgs 用于记录高频传感器相邻3个数据的速度，UTM坐标系
// 输出:无
// 无返回
// ***************
void FusionCenter::processFixedLidarFuse(vector<double *> &pose_match_utm, vector<double *> &vel_match_utm)
{
    /*--------时间配准--------*/
    PoseResult low_freq_match;
    low_freq_match.att.yaw = g_flidar_info.heading;
    low_freq_match.att.line_heading = g_flidar_info.line_heading;
    low_freq_match.fl_tager_pos.tpx = g_flidar_info.tager_position.x;
    low_freq_match.fl_tager_pos.tpy = g_flidar_info.tager_position.y;
    low_freq_match.fl_tager_pos.tpz = g_flidar_info.tager_position.z;
    low_freq_match.agv_dist.agv_x = g_flidar_info.agv_distance.x;
    low_freq_match.agv_dist.agv_y = g_flidar_info.agv_distance.y;
    low_freq_match.fl_vel.Vx = g_flidar_info.Vx;
    low_freq_match.fl_vel.Vy = g_flidar_info.Vy;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      

    //寻找与场端时间相匹配的组合导航数据
    location_msgs::FusionDataInfo fuse_temp;
    fuse_temp = g_record_fuse;
    int tm_delay = 0; //！！！这个对应通讯延时要改
    double utm_pose[3]; //84椭球体UTM坐标位置
    //double WGS84_vel[3];  //84坐标速度
    location_msgs::FusionDataInfo fuse_tp;
    double LLH[3];
    double VENU[3];
    fuse_tp = g_vprecord_fuse.at((buffer_size -1) - tm_delay);
    LLH[0] = fuse_tp.pose_llh.x;
    LLH[1] = fuse_tp.pose_llh.y;
    LLH[2] = fuse_tp.pose_llh.z;
    VENU[0] = fuse_tp.velocity.linear.x;
    VENU[1] = fuse_tp.velocity.linear.y;
    VENU[2] = fuse_tp.velocity.linear.z;
    double lon0 = 0;
    lon0=transForLLHtoUTM(LLH,utm_pose);
    g_record_fuse = fuse_tp;
    g_IMUcalib_count = (buffer_size -1) - tm_delay;  
    g_pmatch_for_time->MatchForStop(g_high_freq_match);
    
    //卡尔曼滤波融合
    UTC time_match;
    time_match.year = g_flidar_info.year;
    time_match.month = g_flidar_info.month;
    time_match.day = g_flidar_info.day;
    time_match.hour = g_flidar_info.hour;
    time_match.min = g_flidar_info.min;
    time_match.sec = g_flidar_info.sec;
    time_match.msec = g_flidar_info.msec;
    VectorXd IMUcalib_(13);
    IMUcalib_ = VectorXd::Zero(13);
    MatrixXd IMUcalib_I_(13, 13);
    IMUcalib_I_ = MatrixXd::Zero(13, 13);
    //判断是否加入零速修正
    if (g_adstatus_info.running_status == 0)
    {
        zupt_type = zupt_stop;
        g_flzkinit_flag = g_pfszuptflidar->calculateFilter(g_high_freq_match, low_freq_match, time_match, g_fusion_data,lon0);
        IMUcalib_ = g_pfszuptflidar->g_fusion_lX;
        IMUcalib_I_ = g_pfszuptflidar->g_fusion_lI;
        g_IMUlast_tm = g_pfszuptflidar->g_last_tm;
    }
    else
    {
        g_flkinit_flag = g_pfsflidar->calculateFilter(g_high_freq_match, low_freq_match, time_match, g_fusion_data,lon0);
        IMUcalib_ = g_pfsflidar->g_fusion_lX;
        IMUcalib_I_ = g_pfsflidar->g_fusion_lI;
        g_IMUlast_tm = g_pfsflidar->g_last_tm;
    }
    transFusionLLHtoHarbourENU(g_fusion_data);
    for (int i = 0; i < 9; i++)
    {
        g_delta_IMUcalib(i) = IMUcalib_(i);
        for (int j = 0; j < 9; j++)
        {
            g_IMUcalib_I(i,j) = IMUcalib_I_(i,j);
        }
    }
    g_pcalibimu->g_calib_I = g_IMUcalib_I;
    g_calib_flag = c_no;
}

// **************
// 功能:处理无零速修正组合导航与车端Lidar的数据融合,并将融合位置转换为港区坐标输出
// 输入:vector <double*> pose_match_wgs用于记录高频传感器相邻3个数据的位置,WGS84坐标系
//     vector <double*> vel_match_wgs 用于记录高频传感器相邻3个数据的速度，WGS84坐标系
// 输出:无
// 无返回
// ***************
void FusionCenter::processLidarFuse(vector<double *> &pose_match_utm, vector<double *> &vel_match_utm)
{
    /*--------时间配准--------*/
    PoseResult low_freq_match;
    low_freq_match.att.yaw = g_lidar_info.yaw;
    low_freq_match.att.pitch = g_lidar_info.pitch;
    low_freq_match.att.roll = g_lidar_info.roll;
    low_freq_match.pos.lan = g_lidar_info.pose_cov.pose.position.x;
    low_freq_match.pos.lon = g_lidar_info.pose_cov.pose.position.y;
    low_freq_match.pos.h = g_lidar_info.pose_cov.pose.position.z;
    low_freq_match.vel.venu.vx = g_lidar_info.vel_cov.twist.linear.x;
    low_freq_match.vel.venu.vy = g_lidar_info.vel_cov.twist.linear.y;
    low_freq_match.vel.venu.vz = g_lidar_info.vel_cov.twist.linear.z;
    UTC low_fre_utc;
    low_fre_utc.hour = g_lidar_info.hour;
    low_fre_utc.min = g_lidar_info.min;
    low_fre_utc.sec = g_lidar_info.sec;
    low_fre_utc.msec = g_lidar_info.msec;

    double delta_update_time = 0;
    location_msgs::FusionDataInfo fuse_temp;
    bool update_date_flag = false;
    fuse_temp = g_record_fuse;
    int tm_delay = 0; //！！！这个对应通讯延时要改
    double lon0 = 0;    //当前中央子午线经度
    findIMUDataForMatch(tm_delay, pose_match_utm, vel_match_utm,low_fre_utc,lon0);
    delta_update_time = g_record_fuse.hour * 3600 + g_record_fuse.min * 60 + g_record_fuse.sec + g_record_fuse.msec / 1000 -
                        (fuse_temp.hour * 3600 + fuse_temp.min * 60 + fuse_temp.sec + fuse_temp.msec / 1000);
    if (g_record_fuse.year == fuse_temp.year && g_record_fuse.month == fuse_temp.month && g_record_fuse.day == fuse_temp.day)
    {
        update_date_flag = false;
    }
    else
    {
        update_date_flag = true;
    }

    //判断何种匹配方式
    if (g_adstatus_info.running_status == 0) //！！！车辆状态可能会因为通讯延时有滞后性，后续再想解决办法
    {
        high_freq_type = integrated_nav;
        low_freq_type = lidar;
        g_pmatch_for_time->MatchForStop(g_high_freq_match);
    }
    else if (g_adstatus_info.running_status == 2)
    {
        high_freq_type = integrated_nav;
        low_freq_type = lidar;
        if (delta_update_time > 0.015 || update_date_flag == true)
        {
            g_pmatch_for_time->MatchForStop(g_high_freq_match);
        }
        else
        {
            g_pmatch_for_time->MatchForCircular(pose_match_utm, vel_match_utm, g_high_freq_match,lon0);
        }
    }
    else
    {
        high_freq_type = integrated_nav;
        low_freq_type = lidar;
        //--------test
        g_pmatch_for_time->MatchForStop(g_high_freq_match);
        // if (delta_update_time > 0.015 || update_date_flag == true)
        // {
        //     g_pmatch_for_time->MatchForStop(g_high_freq_match);
        // }
        // else
        // {
        //     g_pmatch_for_time->MatchForLinear(pose_match_utm, vel_match_utm, g_high_freq_match,lon0);
        // }
    }
    //卡尔曼滤波融合
    UTC time_match;
    time_match.year = g_lidar_info.year;
    time_match.month = g_lidar_info.month;
    time_match.day = g_lidar_info.day;
    time_match.hour = g_lidar_info.hour;
    time_match.min = g_lidar_info.min;
    time_match.sec = g_lidar_info.sec;
    time_match.msec = g_lidar_info.msec;
    VectorXd IMUcalib_(15);
    IMUcalib_ = VectorXd::Zero(15);
    MatrixXd IMUcalib_I_(15, 15);
    IMUcalib_I_ = MatrixXd::Zero(15, 15);

    //判断是否加入零速修正
    if (g_adstatus_info.running_status == 0)
    {
        zupt_type = zupt_stop;
        g_lzkinit_flag = g_pfszuptlidar->calculateFilter(g_high_freq_match, low_freq_match, time_match, g_fusion_data);
        IMUcalib_ = g_pfszuptlidar->g_fusion_lX;
        IMUcalib_I_ = g_pfszuptlidar->g_fusion_lI;
        g_IMUlast_tm = g_pfszuptlidar->g_last_tm;
    }
    else if (g_adstatus_info.running_status == 1 || g_adstatus_info.running_status == 3)
    {
        zupt_type = zupt_linear;
        g_lzkinit_flag = g_pfszuptlidar->calculateFilter(g_high_freq_match, low_freq_match, time_match, g_fusion_data);
        IMUcalib_ = g_pfszuptlidar->g_fusion_lX;
        IMUcalib_I_ = g_pfszuptlidar->g_fusion_lI;
        g_IMUlast_tm = g_pfszuptlidar->g_last_tm;
    }
    else
    {
        g_lkinit_flag = g_pfslidar->calculateFilter(g_high_freq_match, low_freq_match, time_match, g_fusion_data);
        IMUcalib_ = g_pfslidar->g_fusion_lX;
        IMUcalib_I_ = g_pfslidar->g_fusion_lI;
        g_IMUlast_tm = g_pfslidar->g_last_tm;
    }
    transFusionLLHtoHarbourENU(g_fusion_data);
    for (int i = 0; i < 9; i++)
    {
        g_delta_IMUcalib(i) = IMUcalib_(i);
        for (int j = 0; j < 9; j++)
        {
            g_IMUcalib_I(i, j) = IMUcalib_I_(i, j);
        }
    }
    g_pcalibimu->g_calib_I = g_IMUcalib_I;
    g_calib_flag = c_no;
    for (int i = 0; i < 3; i++)
    {
        free(pose_match_utm[i]);
        free(vel_match_utm[i]);
    }
}

// **************
// 功能:处理组合导航与航位推算的数据融合,并将融合位置转换为港区坐标输出
// 输入:vector <double*> pose_match_wgs用于记录高频传感器相邻3个数据的位置,WGS84坐标系
//     vector <double*> vel_match_wgs 用于记录高频传感器相邻3个数据的速度，WGS84坐标系
// 输出:无
// 无返回
// ***************
void FusionCenter::processDRFuse(vector <double*> &pose_match_utm,vector <double*> &vel_match_utm)
{
    /*--------时间配准--------*/
    PoseResult low_freq_match;
    low_freq_match.att.yaw = g_DR_info.heading;     //暂时木有
    low_freq_match.att.pitch = g_DR_info.pitch;     //暂时木有
    low_freq_match.att.roll = g_DR_info.roll;       //暂时木有
    low_freq_match.pos.lan = g_DR_info.x;           //港区坐标
    low_freq_match.pos.lon = g_DR_info.y;           //港区坐标
    low_freq_match.pos.h = g_DR_info.z;             //港区坐标(暂无)
    low_freq_match.vel.venu.vx = g_DR_info.velocity_east;
    low_freq_match.vel.venu.vy = g_DR_info.velocity_north;
    low_freq_match.vel.venu.vz = g_DR_info.velocity_up;     //暂时木有
    UTC low_fre_utc;
    low_fre_utc.hour = g_DR_info.hour;
    low_fre_utc.min = g_DR_info.min;
    low_fre_utc.sec = g_DR_info.sec;
    low_fre_utc.msec = g_DR_info.msec;

    double delta_update_time = 0;
    location_msgs::FusionDataInfo fuse_temp;
    bool update_date_flag = false;
    fuse_temp = g_record_fuse;
    int tm_delay = 0; //！！！这个对应通讯延时要改
    double lon0 = 0;    //当前中央子午线经度
    findIMUDataForMatch(tm_delay, pose_match_utm, vel_match_utm,low_fre_utc,lon0);
    delta_update_time = g_record_fuse.hour * 3600 + g_record_fuse.min * 60 + g_record_fuse.sec + g_record_fuse.msec / 1000 -
                        (fuse_temp.hour * 3600 + fuse_temp.min * 60 + fuse_temp.sec + fuse_temp.msec / 1000);
    if (g_record_fuse.year == fuse_temp.year && g_record_fuse.month == fuse_temp.month && g_record_fuse.day == fuse_temp.day)
    {
        update_date_flag = false;
    }
    else
    {
        update_date_flag = true;
    }

    //判断何种匹配方式
    if (g_adstatus_info.running_status == 0) //！！！车辆状态可能会因为通讯延时有滞后性，后续再想解决办法
    {
        high_freq_type = integrated_nav;
        low_freq_type = dr;
        g_pmatch_for_time->MatchForStop(g_high_freq_match);
    }
    else if (g_adstatus_info.running_status == 2)
    {
        high_freq_type = integrated_nav;
        low_freq_type = dr;
        if (delta_update_time > 0.015 || update_date_flag == true)
        {
            g_pmatch_for_time->MatchForStop(g_high_freq_match);
        }
        else
        {
            g_pmatch_for_time->MatchForCircular(pose_match_utm, vel_match_utm, g_high_freq_match,lon0);
        }
    }
    else
    {
        high_freq_type = integrated_nav;
        low_freq_type = dr;
        //--------test
        g_pmatch_for_time->MatchForStop(g_high_freq_match);
        // if (delta_update_time > 0.015 || update_date_flag == true)
        // {
        //     g_pmatch_for_time->MatchForStop(g_high_freq_match);
        // }
        // else
        // {
        //     g_pmatch_for_time->MatchForLinear(pose_match_utm, vel_match_utm, g_high_freq_match,lon0);
        // }
    }
    //卡尔曼滤波融合
    UTC time_match;
    time_match.year = g_DR_info.year;
    time_match.month = g_DR_info.month;
    time_match.day = g_DR_info.day;
    time_match.hour = g_DR_info.hour;
    time_match.min = g_DR_info.min;
    time_match.sec = g_DR_info.sec;
    time_match.msec = g_DR_info.msec;
    VectorXd IMUcalib_(6);
    IMUcalib_ = VectorXd::Zero(6);
    MatrixXd IMUcalib_I_(6, 6);
    IMUcalib_I_ = MatrixXd::Zero(6, 6);

    g_drkint_flag = g_pfsdr->calculateFilter(g_high_freq_match,low_freq_match,time_match,g_fusion_data);
    IMUcalib_ = g_pfsdr->g_fusion_lX;
    IMUcalib_I_ = g_pfsdr->g_fusion_lI;
    g_IMUlast_tm = g_pfsdr->g_last_tm;
    transFusionLLHtoHarbourENU(g_fusion_data);

    g_delta_IMUcalib.resize(9); //用于校正IMU姿态速度与位置的值,yaw,pitch,roll，ve,vn,vu,lan,lon,h
    g_IMUcalib_I.resize(9, 9);
    g_delta_IMUcalib = VectorXd::Zero(9);
    g_IMUcalib_I = MatrixXd::Zero(9, 9);
    for (int i = 0; i < 6; i++)
    {
        g_delta_IMUcalib(i+3) = IMUcalib_(i);
        for (int j = 0; j < 6; j++)
        {
            g_IMUcalib_I(i+3, j+3) = IMUcalib_I_(i, j);
        }
    }
    g_pcalibimu->g_calib_I = g_IMUcalib_I;
    g_calib_flag = c_no;
    for (int i = 0; i < 3; i++)
    {
        free(pose_match_utm[i]);
        free(vel_match_utm[i]);
    }
}

//**************
// 功能:在组合导航只有IMU输出时,传感器两次融合之间对于IMU输出速度与位置的校正,并输出港区定位结果
// 输入:double delta_IMUcalib[6]校正值,fusion_data_msgs::FusionDataInfo IMU_in 组合导航IMU的值
// 输出:无
// 无返回
//***************
void FusionCenter::processIMUCalibrate()
{
    PoseResult h_fm;
    location_msgs::FusionDataInfo fuse_temp;
    fuse_temp = g_vprecord_fuse.at(point_to-1);
    h_fm.pos.lan = fuse_temp.pose_llh.x;
    h_fm.pos.lon = fuse_temp.pose_llh.y;
    h_fm.pos.h = fuse_temp.pose_llh.z;
    h_fm.vel.venu.vx = fuse_temp.velocity.linear.x;
    h_fm.vel.venu.vy = fuse_temp.velocity.linear.y;
    h_fm.vel.venu.vz = fuse_temp.velocity.linear.z;
    h_fm.vel.wxyz.wx = fuse_temp.velocity.angular.x;
    h_fm.vel.wxyz.wy = fuse_temp.velocity.angular.y;
    h_fm.vel.wxyz.wz = fuse_temp.velocity.angular.z;
    h_fm.att.yaw = fuse_temp.yaw;
    h_fm.att.pitch = fuse_temp.pitch;
    h_fm.att.roll = fuse_temp.roll;
    h_fm.accel.ax = fuse_temp.accel.linear.x;
    h_fm.accel.ay = fuse_temp.accel.linear.y;
    h_fm.accel.az = fuse_temp.accel.linear.z;
    //卡尔曼滤波融合
    UTC time_match;
    time_match.hour = fuse_temp.hour;
    time_match.min = fuse_temp.min;
    time_match.sec = fuse_temp.sec;
    time_match.msec = fuse_temp.msec;
    g_pcalibimu->g_last_tm = g_IMUlast_tm;
    g_pcalibimu->calculateFilter(h_fm, time_match, g_fusion_data_calib);
    g_IMUlast_tm = time_match;
    transFusionLLHtoHarbourENU(g_fusion_data_calib);
    g_calib_flag = c_no;
}


// **************
// 功能:寻找用于融合时间匹配的IMU数据并转换成WGS84的ECEF坐标位置与速度
// 输入:tm_delay 时间延时
// 输出:pose_match_wgs用于匹配的ECEF位置坐标,vel_match_wgs用于匹配的ECEF速度,lon0中央子午线经度
// 无返回
// ***************
void FusionCenter::findIMUDataForMatch(int &tm_delay, vector<double *> &pose_match_utm, vector<double *> &vel_match_utm,UTC &low_fre_utc,double &lon0)
{
    //寻找其他传感器与组合导航融合时间相匹配的点
    vector<location_msgs::FusionDataInfo> vp_match;
    int len = 21;
    memset(&vp_match,0,sizeof(location_msgs::FusionDataInfo)*len);
    vector<location_msgs::FusionDataInfo>::iterator it = g_vprecord_fuse.begin()+ (point_to-11);
    for (int i = 0; i < len; i++)
    {
        if (it!=g_vprecord_fuse.end())
        {
            vp_match[i] = g_vprecord_fuse.at(i + (point_to-11) - tm_delay);
            it++;
        }
        else
        {
            break;
        }   
    }
    location_msgs::FusionDataInfo tp_imu0;
    tp_imu0 = g_vprecord_fuse.at((point_to-11) - tm_delay);
    double delta_tm1 = fabs(tp_imu0.hour*3600 + tp_imu0.min*60 + tp_imu0.sec + tp_imu0.msec/1000 
                     - (low_fre_utc.hour*3600 + low_fre_utc.min*60 + low_fre_utc.sec + low_fre_utc.msec/1000));
    int cnt = 0;
    int v_size = vp_match.size();
    for (int i = 1; i < v_size; i++)
    {
        location_msgs::FusionDataInfo tp_imu;
        tp_imu = g_vprecord_fuse.at(i+(point_to-11) - tm_delay);
        double delta_tm2 = fabs(tp_imu.hour*3600 + tp_imu.min*60 + tp_imu.sec + tp_imu.msec/1000 
                        - (low_fre_utc.hour*3600 + low_fre_utc.min*60 + low_fre_utc.sec + low_fre_utc.msec/1000));
        double delta_tm = 0;
        delta_tm = (delta_tm1<=delta_tm2?delta_tm1:delta_tm2);
        if (delta_tm == delta_tm1)
        {
            cnt = i-1 + (point_to-11) - tm_delay;
        }
        else
        {
            cnt = i + (point_to-11) - tm_delay;
            delta_tm1 = delta_tm2;
        }  
    }

    //计录组合导航匹配点三个的位置用作时间匹配
    for (int i = 0; i < 3; i++)
    {
        double utm_pose[3]={0.0,0.0,0.0}; //84椭球体UTM坐标位置
        //double WGS84_vel[3];  //84坐标速度
        location_msgs::FusionDataInfo fuse_temp;
        double LLH[3]={0.0,0.0,0.0};
        double VENU[3]={0.0,0.0,0.0};
        fuse_temp = g_vprecord_fuse.at(i + cnt - 2);
        LLH[0] = fuse_temp.pose_llh.x;
        LLH[1] = fuse_temp.pose_llh.y;
        LLH[2] = fuse_temp.pose_llh.z;
        VENU[0] = fuse_temp.velocity.linear.x;
        VENU[1] = fuse_temp.velocity.linear.y;
        VENU[2] = fuse_temp.velocity.linear.z;
        lon0=transForLLHtoUTM(LLH,utm_pose);
        pose_match_utm[i] = (double *)malloc(sizeof(double) * 3);
        vel_match_utm[i] = (double *)malloc(sizeof(double) * 3);
        memcpy(pose_match_utm[i], utm_pose, sizeof(double) * 3);
        memcpy(vel_match_utm[i], VENU, sizeof(double) * 3);
        if (i == 2)
        {
            g_record_fuse = fuse_temp;
            g_IMUcalib_count = i + cnt - 2;
        }
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "FusionCenter_node");

    ros::NodeHandle private_nh3("~IMU_GLOBAL");
    ros::NodeHandle private_nh4("~IMU_UWB");
    ros::NodeHandle private_nh5("~IMU_LIDAR");
    ros::NodeHandle private_nh7("~IMU_ZUPT_UWB");
    ros::NodeHandle private_nh8("~IMU_ZUPT_LIDAR");
    //  ros::NodeHandle private_nh9("~IMU_ZUPT_FLIDAR");
    //  ros::NodeHandle private_nh6("~IMU_FLIDAR");

    boost::shared_ptr<dynamic_reconfigure::Server<IMU_config_package::IMU_global_Config>> srv3_;
    srv3_ = boost::make_shared<dynamic_reconfigure::Server<IMU_config_package::IMU_global_Config>>(private_nh3);
    dynamic_reconfigure::Server<IMU_config_package::IMU_global_Config>::CallbackType f3;
    f3 = boost::bind(&IMU_GLOBALcallback, _1, _2);
    srv3_->setCallback(f3);

    boost::shared_ptr<dynamic_reconfigure::Server<IMU_config_package::IMU_UWB_Config>> srv4_;
    srv4_ = boost::make_shared<dynamic_reconfigure::Server<IMU_config_package::IMU_UWB_Config>>(private_nh4);
    dynamic_reconfigure::Server<IMU_config_package::IMU_UWB_Config>::CallbackType f4;
    f4 = boost::bind(&IMU_UWBcallback, _1, _2);
    srv4_->setCallback(f4);

    boost::shared_ptr<dynamic_reconfigure::Server<IMU_config_package::IMU_Lidar_Config>> srv5_;
    srv5_ = boost::make_shared<dynamic_reconfigure::Server<IMU_config_package::IMU_Lidar_Config>>(private_nh5);
    dynamic_reconfigure::Server<IMU_config_package::IMU_Lidar_Config>::CallbackType f5;
    f5 = boost::bind(&IMU_LIDARcallback, _1, _2);
    srv5_->setCallback(f5);

    // IMU_ZUPT_UWB_Config
    boost::shared_ptr<dynamic_reconfigure::Server<IMU_config_package::IMU_ZUPT_UWB_Config>> srv7_;
    srv7_ = boost::make_shared<dynamic_reconfigure::Server<IMU_config_package::IMU_ZUPT_UWB_Config>>(private_nh7);
    dynamic_reconfigure::Server<IMU_config_package::IMU_ZUPT_UWB_Config>::CallbackType f7;
    f7 = boost::bind(&IMU_ZUPT_UWBcallback, _1, _2);
    srv7_->setCallback(f7);

    // IMU_ZUPT_Lidar_Config
    boost::shared_ptr<dynamic_reconfigure::Server<IMU_config_package::IMU_ZUPT_Lidar_Config>> srv8_;
    srv8_ = boost::make_shared<dynamic_reconfigure::Server<IMU_config_package::IMU_ZUPT_Lidar_Config>>(private_nh8);
    dynamic_reconfigure::Server<IMU_config_package::IMU_ZUPT_Lidar_Config>::CallbackType f8;
    f8 = boost::bind(&IMU_ZUPT_LIDARcallback, _1, _2);
    srv8_->setCallback(f8);

    //     IMU_Fixed_Lidar_Config
    //     boost::shared_ptr< dynamic_reconfigure::Server< IMU_config_package::IMU_Fixed_Lidar_Config > > srv6_;
    //     srv6_ = boost::make_shared< dynamic_reconfigure::Server< IMU_config_package::IMU_Fixed_Lidar_Config >
    //     >(private_nh6);
    //     dynamic_reconfigure::Server< IMU_config_package::IMU_Fixed_Lidar_Config >::CallbackType f6;
    //     f6 = boost::bind(&IMU_FLIDARcallback, _1, _2);
    //     srv6_->setCallback(f6);

    //   IMU_ZUPT_FLidar_Config
    //     boost::shared_ptr< dynamic_reconfigure::Server< IMU_config_package::IMU_ZUPT_FLidar_Config > > srv9_;
    //     srv9_ = boost::make_shared< dynamic_reconfigure::Server< IMU_config_package::IMU_ZUPT_FLidar_Config >
    //     >(private_nh9);
    //     dynamic_reconfigure::Server< IMU_config_package::IMU_ZUPT_FLidar_Config >::CallbackType f9;
    //     f9 = boost::bind(&IMU_ZUPT_FLIDARcallback, _1, _2);
    //     srv9_->setCallback(f9);

    ros::NodeHandle FusionCenter_node_nh; //必须在main函数创建一个节点句柄给构造函数使用
    FusionCenter m_fusion_center(FusionCenter_node_nh);

    return 0;
}
