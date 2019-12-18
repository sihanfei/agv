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

#define SteerTurnTest           //转角测试标志，如果不需要请注释该行
#ifdef SteerTurnTest
#define turntest_Period 10      //s
#define turntest_Angle  30      //degree
#endif
#define SteerAngle_MAX 45		//单位：度
#define ExpSpeed_MAX 2				//单位：m/s

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

#endif