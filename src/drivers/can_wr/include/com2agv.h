#ifndef _COM2AGV_H
#define _COM2AGV_H

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <string>
#include <vector>
#include "adcuSDK.h"
#include "control_msgs/com2veh.h"
#include "control_algo.h"
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8MultiArray.h>

 #define Test_TurnPeriod 20      //unit: s 转角测试标志，如果不需要请注释该行
 #define Test_TurnAngle  20      //unit: degree 转角测试标志，如果不需要请注释该行

//  #define Test_StraightLength 50       //unit: m 直线测试标志，如果不需要请注释该行

#define SteerAngle_MAX 45		//unit: degree
#define ExpSpeed_MAX 0.5		//unit: m/s
#define Freq_com2agv 100        //unit: HZ

#define ID_VCU_2LCD 257
#define ID_LCD      385
#define ID_IEPS     273
#define ID_TB       1321

typedef enum
{
  VCU2C111= 0x111,	    //IEPS TO VCU ID
  VCU2C112= 0x112		//VCU TO IEPS ID
}AGV_FiltID;

enum Gears
{
    Drive = 0x1,
    Park,
    Neutral,
    Reverse
};

enum LeftModule
{
    Rise = 0x1,
    Stop,
    NoAction,
    Land
};


union CANdataUN
{
	adcuCanData canrecvbuf;
	uint8_t candata[16];
}candatareceive;

struct AGV2ControlInfo
{
    float ActualSpd;            //实际车速
    float ActualAgl_R;          //实际后转角
    float ActualAgl_F;          //实际前转角
    uint8_t VEHMode;            //车辆工作模式
    uint8_t VEHFlt;             //车辆故障状态
    uint8_t LiftStatus;         //举升系统状态
    bool HVStatus;              //整车高压状态
    bool EStopStatus;           //紧急停车状态
    bool EPBStatus;             //手刹状态
    uint8_t Dir_PRND;           //档位装状态
    uint8_t SOC;                //电池电量剩余
    uint8_t Rolling_Counter;    //心跳信号
}AGV2ControlInfo;
struct Control2AGVInfo
{
    float Vel_Req;              //车辆速度请求
    LeftModule LiftCmd;         //顶升控制请求
    Gears Dir_PRND;             //档位控制请求
    float VehAgl_F;             //车辆前转向角度请求
    bool EPB;                   //手刹启动命令
    bool EStop;                 //紧急停车指令
    bool AutoDrive_Enable;      //自动驾驶使能标志
    float VehAgl_R;             //车辆后转向角度请求
    uint8_t Rolling_Counter;    //心跳信号

}Control2AGVInfo;
// {
//     10.2f,
//     2,
//     2,
//     -22.0f,
//     true,
//     false,
//     false,
//     30.1f,
//     7
// };

struct VCU_2LCD_Info
{
    uint8_t VCU_2LCD_SOCAlarm  : 1;
    uint8_t VCU_2LCD_HVStatus  : 1;
    uint8_t VCU_2LCD_Fault     : 1;
    uint8_t VCU_2LCD_ChgStatus : 1;
    
    VCU_2LCD_Info()
    {
        memset(this,0,sizeof(VCU_2LCD_Info));
    }
};

struct LCD_Info
{
    uint8_t LCD_CnnctFlag : 1;
    uint8_t LCD_Dir_PRND  : 3;
    uint8_t LCD_SteerMode : 4;
    uint8_t LCD_DriveMode : 3;
    uint8_t LCD_EStop     : 1;
    uint8_t LCD_LiftCmd   : 3;
    uint8_t LCD_SpdLimit  : 3;

    LCD_Info()
    {
        memset(this,0,sizeof(LCD_Info));
    }
};

struct IEPS_Info
{
    uint8_t IEPS_AutoDrive_Enable : 1;
    uint8_t IEPS_EStop            : 1;
    uint8_t IEPS_EPB              : 1;
    uint8_t IEPS_Dir_PRND         : 3;
    uint8_t IEPS_LiftCmd          : 3;
    IEPS_Info()
    {
        memset(this,0,sizeof(IEPS_Info));
    }
};

struct TB_Info
{
    uint8_t TB_EN   : 1;
    uint8_t TB_Mode : 4;

    TB_Info()
    {
        memset(this,0,sizeof(TB_Info));
    }
};

#endif
