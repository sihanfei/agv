#ifndef GLOBALVARI_H
#define GLOBALVARI_H

#include <ros/ros.h>
#include <string>

using namespace std;

struct CfgUWB_P0
{
    double Fai_yaw;
    double Fai_pitch;
    double Fai_roll;
    double delta_ve;
    double delta_vn;
    double delta_vu;
    double delta_lan;
    double delta_lon;
    double delta_h;
    double delta_Uve;
    double delta_Uvn;
    double delta_Ulan;
    double delta_Ulon;
    CfgUWB_P0(){
        memset(this ,0.0,sizeof(CfgUWB_P0));
    }
};

struct CfgUWB_Q0
{
    double Fai_yaw;
    double Fai_pitch;
    double Fai_roll;
    double delta_ve;
    double delta_vn;
    double delta_vu;
    double delta_lan;
    double delta_lon;
    double delta_h;
    double delta_Uve;
    double delta_Uvn;
    double delta_Ulan;
    double delta_Ulon;
    CfgUWB_Q0(){
        memset(this ,0.0,sizeof(CfgUWB_Q0));
    }

};

struct CfgUWB_X0
{
    double Fai_yaw;
    double Fai_pitch;
    double Fai_roll;
    double delta_ve;
    double delta_vn;
    double delta_vu;
    double delta_lan;
    double delta_lon;
    double delta_h;
    double delta_Uve;
    double delta_Uvn;
    double delta_Ulan;
    double delta_Ulon;
    CfgUWB_X0(){
        memset(this ,0.0,sizeof(CfgUWB_X0));
    }

};

struct CfgUWB_R
{
    double delta_yaw;
    double delta_Cve;
    double delta_Cvn;
    double delta_Clan;
    double delta_Clon;
    CfgUWB_R(){
        memset(this,0.0,sizeof(CfgUWB_R));
    }
};

struct CfgUWB_Tao
{
    double delta_Uve;
    double delta_Uvn;
    double delta_Ulan;
    double delta_Ulon;
    CfgUWB_Tao(){
        memset(this,0.0,sizeof(CfgUWB_Tao));
    }
};

struct Cfg_UWB
{
    CfgUWB_P0 P0;
    CfgUWB_P0 q0;
    CfgUWB_X0 X0;
    CfgUWB_R R;
    CfgUWB_Tao Tao;
   Cfg_UWB(){
        memset(this,0.0,sizeof(Cfg_UWB));
    }
};

struct CfgZUPTUWB_P0
{
    double Fai_yaw;
    double Fai_pitch;
    double Fai_roll;
    double delta_ve;
    double delta_vn;
    double delta_vu;
    double delta_lan;
    double delta_lon;
    double delta_h;
    double delta_Uve;
    double delta_Uvn;
    double delta_Ulan;
    double delta_Ulon;
    CfgZUPTUWB_P0(){
        memset(this ,0.0,sizeof(CfgZUPTUWB_P0));
    }
};

struct CfgZUPTUWB_Q0
{
    double Fai_yaw;
    double Fai_pitch;
    double Fai_roll;
    double delta_ve;
    double delta_vn;
    double delta_vu;
    double delta_lan;
    double delta_lon;
    double delta_h;
    double delta_Uve;
    double delta_Uvn;
    double delta_Ulan;
    double delta_Ulon;
    CfgZUPTUWB_Q0(){
        memset(this ,0.0,sizeof(CfgZUPTUWB_Q0));
    }

};

struct CfgZUPTUWB_X0
{
    double Fai_yaw;
    double Fai_pitch;
    double Fai_roll;
    double delta_ve;
    double delta_vn;
    double delta_vu;
    double delta_lan;
    double delta_lon;
    double delta_h;
    double delta_Uve;
    double delta_Uvn;
    double delta_Ulan;
    double delta_Ulon;
    CfgZUPTUWB_X0(){
        memset(this ,0.0,sizeof(CfgZUPTUWB_X0));
    }

};

struct CfgZUPTSTOP_UWB_R
{
    double delta_yaw;
    double delta_ve;
    double delta_vn;
    double delta_vu;
    double delta_Uve;
    double delta_Uvn;
    double delta_Clan;
    double delta_Clon;
    CfgZUPTSTOP_UWB_R(){
        memset(this,0.0,sizeof(CfgZUPTSTOP_UWB_R));
    }
};

struct CfgZUPTLIN_UWB_R
{
    double delta_yaw;
    double delta_Cve;
    double delta_Cvn;
    double delta_Clan;
    double delta_Clon;
    double Bvx;
    double Bvz;
    CfgZUPTLIN_UWB_R(){
        memset(this,0.0,sizeof(CfgZUPTLIN_UWB_R));
    }
};

struct CfgZUPTUWB_Tao
{
    double delta_Uve;
    double delta_Uvn;
    double delta_Ulan;
    double delta_Ulon;
    CfgZUPTUWB_Tao(){
        memset(this,0.0,sizeof(CfgZUPTUWB_Tao));
    }
};

struct Cfg_ZUPTUWB
{
    CfgZUPTUWB_P0 P0;
    CfgZUPTUWB_Q0 q0;
    CfgZUPTUWB_X0 X0;
    CfgZUPTSTOP_UWB_R ZSTOP_R;
    CfgZUPTLIN_UWB_R   ZLIN_R;
    CfgZUPTUWB_Tao Tao;
    Cfg_ZUPTUWB(){
        memset(this,0.0,sizeof(Cfg_ZUPTUWB));
    }
};

struct CfgLidar_P0
{
    double Fai_yaw;
    double Fai_pitch;
    double Fai_roll;
    double delta_ve;
    double delta_vn;
    double delta_vu;
    double delta_lan;
    double delta_lon;
    double delta_h;
    double delta_Lve;
    double delta_Lvn;
    double delta_Lvu;
    double delta_Llan;
    double delta_Llon;
    double delta_Lh;
    CfgLidar_P0(){
        memset(this ,0.0,sizeof(CfgLidar_P0));
    }
};

struct CfgLidar_Q0
{
    double Fai_yaw;
    double Fai_pitch;
    double Fai_roll;
    double delta_ve;
    double delta_vn;
    double delta_vu;
    double delta_lan;
    double delta_lon;
    double delta_h;
    double delta_Lve;
    double delta_Lvn;
    double delta_Lvu;
    double delta_Llan;
    double delta_Llon;
    double delta_Lh;
    CfgLidar_Q0(){
        memset(this ,0.0,sizeof(CfgLidar_Q0));
    }

};

struct CfgLidar_X0
{
    double Fai_yaw;
    double Fai_pitch;
    double Fai_roll;
    double delta_ve;
    double delta_vn;
    double delta_vu;
    double delta_lan;
    double delta_lon;
    double delta_h;
    double delta_Lve;
    double delta_Lvn;
    double delta_Lvu;
    double delta_Llan;
    double delta_Llon;
    double delta_Lh;
    CfgLidar_X0(){
        memset(this ,0.0,sizeof(CfgLidar_X0));
    }
};

struct CfgLidar_R
{
    double delta_yaw;
    double delta_pitch;
    double delta_roll;
    double delta_Cve;
    double delta_Cvn;
    double delta_Cvu;
    double delta_Clan;
    double delta_Clon;
    double delta_Ch;
    CfgLidar_R(){
        memset(this,0.0,sizeof(CfgLidar_R));
    }
};

struct CfgLidar_Tao
{
    double delta_Lve;
    double delta_Lvn;
    double delta_Lvu;
    double delta_Llan;
    double delta_Llon;
    double delta_Lh;
    CfgLidar_Tao(){
        memset(this,0.0,sizeof(CfgLidar_Tao));
    }
};

struct Cfg_Lidar
{
    CfgLidar_P0 P0;
    CfgLidar_Q0 q0;
    CfgLidar_X0 X0;
    CfgLidar_R R;
    CfgLidar_Tao Tao;
    Cfg_Lidar(){
        memset(this,0.0,sizeof(Cfg_Lidar));
    }
};

struct CfgZUPTLidar_P0
{
    double Fai_yaw;
    double Fai_pitch;
    double Fai_roll;
    double delta_ve;
    double delta_vn;
    double delta_vu;
    double delta_lan;
    double delta_lon;
    double delta_h;
    double delta_Lve;
    double delta_Lvn;
    double delta_Lvu;
    double delta_Llan;
    double delta_Llon;
    double delta_Lh;
    CfgZUPTLidar_P0(){
        memset(this ,0.0,sizeof(CfgZUPTLidar_P0));
    }
};

struct CfgZUPTLidar_Q0
{
    double Fai_yaw;
    double Fai_pitch;
    double Fai_roll;
    double delta_ve;
    double delta_vn;
    double delta_vu;
    double delta_lan;
    double delta_lon;
    double delta_h;
    double delta_Lve;
    double delta_Lvn;
    double delta_Lvu;
    double delta_Llan;
    double delta_Llon;
    double delta_Lh;
    CfgZUPTLidar_Q0(){
        memset(this ,0.0,sizeof(CfgZUPTLidar_Q0));
    }

};

struct CfgZUPTLidar_X0
{
    double Fai_yaw;
    double Fai_pitch;
    double Fai_roll;
    double delta_ve;
    double delta_vn;
    double delta_vu;
    double delta_lan;
    double delta_lon;
    double delta_h;
    double delta_Lve;
    double delta_Lvn;
    double delta_Lvu;
    double delta_Llan;
    double delta_Llon;
    double delta_Lh;
    CfgZUPTLidar_X0(){
        memset(this ,0.0,sizeof(CfgZUPTLidar_X0));
    }

};

struct CfgZUPTSTOP_Lidar_R
{
    double delta_yaw;
    double delta_pitch;
    double delta_roll;
    double delta_ve;
    double delta_vn;
    double delta_vu;
    double delta_Lve;
    double delta_Lvn;
    double delta_Lvu;
    double delta_Clan;
    double delta_Clon;
    double delta_Ch;
    CfgZUPTSTOP_Lidar_R(){
        memset(this,0.0,sizeof(CfgZUPTSTOP_Lidar_R));
    }
};

struct CfgZUPTLIN_Lidar_R
{
    double delta_yaw;
    double delta_pitch;
    double delta_roll;
    double delta_Cve;
    double delta_Cvn;
    double delta_Cvu;
    double delta_Clan;
    double delta_Clon;
    double delta_Ch;
    double Bvx;
    double Bvz;
    CfgZUPTLIN_Lidar_R(){
        memset(this,0.0,sizeof(CfgZUPTLIN_Lidar_R));
    }
};

struct CfgZUPTLidar_Tao
{
    double delta_Lve;
    double delta_Lvn;
    double delta_Lvu;
    double delta_Llan;
    double delta_Llon;
    double delta_Lh;
    CfgZUPTLidar_Tao(){
        memset(this,0.0,sizeof(CfgZUPTLidar_Tao));
    }
};

struct Cfg_ZUPTLidar
{
    CfgZUPTLidar_P0 P0;
    CfgZUPTLidar_Q0 q0;
    CfgZUPTLidar_X0 X0;
    CfgZUPTSTOP_Lidar_R ZSTOP_R;
    CfgZUPTLIN_Lidar_R ZLIN_R;
    CfgZUPTLidar_Tao Tao;
    Cfg_ZUPTLidar(){
        memset(this,0.0,sizeof(Cfg_ZUPTLidar));
    }
};

extern Cfg_UWB g_cfg_uwb;
extern Cfg_ZUPTUWB g_cfg_zuptuwb;
extern Cfg_Lidar g_cfg_lidar;
extern Cfg_ZUPTLidar g_cfg_zuptlidar;
extern int g_imu_count_cfg;
extern float g_lidar_pose_confidience_cfg; //!!!0.95调参量

#endif