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

using namespace std;

#define SteerTurnTest false
#define SteerAngle_MAX 406		//单位：度
#define ExpSpeed_MAX 20				//单位：km/h

uint32_t timecount = 0;
uint32_t starttime = 0;
uint32_t control_stamp;
ros::Publisher com2veh_pub;
ros::Subscriber control_vcu_sub_;
control_msgs::com2veh msg;
PID speedcontrol;

typedef enum
{
  vel_spd= 0x265,
  steer_angle= 0xA1
}H7_FiltID;

union CANdataUN
{
	adcuCanData canrecvbuf;
	uint8_t candata[16];
}candatareceive;

float Vkm2mConst = 1/3.6f;
float VelSpeed = 0.0f;        //车辆速度单位是： is: Km/h
float SteerAngle = 0.0f;
float ExpVelSpeed = 6.0f;
float ExpDecel = 0.0f;        //deceleration 
float ExpAcce = 0.0f;
float ExpAngle=0.0f;
bool BrakeEnable = false;
bool AccelEnable = false;
bool SteerEnable = false;
bool AutodriveEnable = false;

void paraminit(void)
{
  speedcontrol.kp = 0.8f;//0.8f;
  speedcontrol.ki = 0.0f;
  speedcontrol.kd = 0.0f;
  speedcontrol.I_max = 5000.0f;
  speedcontrol.D_max = 50.0f;
  speedcontrol.Out_max = 5.0f;
  speedcontrol.Out_min = -4.0f;
}
void recvControlVCUCallback(const control_msgs::com2veh& msg)
{
	ExpVelSpeed = msg.ExpSpeed;			//读取期望速度
	ROS_INFO("readExpVelSpeed:%lf",ExpVelSpeed);
	ValueConstrain(ExpVelSpeed,0,ExpSpeed_MAX);
	ExpAngle = msg.SteerAngle;	//读取期望转角
	ValueConstrain(ExpAngle,-SteerAngle_MAX,SteerAngle_MAX);
	//ROS_INFO("readExpAngle:%lf",ExpAngle);
	//control_stamp = msg.header.stamp.toSec();
	AutodriveEnable = true;
	SteerEnable = true;
}

pthread_t ReadThread;
void *readLoop(void *pdata)
{
  int deviceid = *(int *)pdata;
  int packageCounter = 0;
  paraminit();

  while (1)
  {
    uint8_t buffer[1024];
    int length = 0;
		
    length = adcuDevRead(deviceid, buffer, 1000);
    if (length > 0)
    {
    //   packageCounter++;
    //   printf("RX %9d:", packageCounter);
    //   for (int i = 0; i < length; i++)
    //   {
    //     printf("%02X ", buffer[i]);
    //   }
    //   printf("\n");
		memcpy(candatareceive.candata, &buffer, length);
        if (candatareceive.canrecvbuf.id == vel_spd || candatareceive.canrecvbuf.id == steer_angle)
        {
					uint8_t rec_buf[sizeof(adcuCanData)]={0};
          for(int i = 0; i < sizeof(adcuCanData); i++)
          {
            rec_buf[i] = candatareceive.canrecvbuf.can_data[i];
            // printf(" %02X", rec_buf[i]);
          }
          
          uint8_t sign=0;
          switch (candatareceive.canrecvbuf.id)
          {
            case vel_spd:
              VelSpeed = (float)((rec_buf[0] & 0x1f <<8) | rec_buf[1]) * 0.05625f;
              //printf("velspeed is : %f \n", VelSpeed);
              msg.VelSpeed = VelSpeed;
              break;
            case steer_angle:
              sign = rec_buf[2] & 0x01;
              SteerAngle = (float)((rec_buf[1] <<7) | (rec_buf[2] >>1)) * 0.1f;
              if (!sign) {
                SteerAngle = -SteerAngle;
              }
              msg.SteerAngle = SteerAngle;
              //printf("SteerAngle is : %f \n", SteerAngle);
              break;
            default:
              break;
          }
          pid_loop(&speedcontrol, ExpVelSpeed * Vkm2mConst, VelSpeed * Vkm2mConst, ros::Time::now().toSec());
          //printf("speedOutput is : %f \n", speedcontrol.Output);
          if (speedcontrol.Output > 0.02f)
          {
            ExpAcce = speedcontrol.Output;
            ExpDecel = 0.0f;
            AccelEnable = true;
            BrakeEnable = false;
          }
          else if (speedcontrol.Output < -0.08f)
          {
            ExpAcce = 0.0f;
            ExpDecel = -speedcontrol.Output;  // * 1.2f;
            if (ExpDecel > 6.5f)
            {
              ExpDecel = 6.5f;
            }
            
            AccelEnable = false;
            BrakeEnable = true;
          }
          else
          {
            ExpAcce = 0.0f;
            ExpDecel = 0.0f;
            AccelEnable = false;
            BrakeEnable = false;
          }
					if (SteerTurnTest)
					{
							ExpAngle = 30 * sin((ros::Time::now().toSec() - starttime)/3);
							msg.ExpAngle = ExpAngle;
					}
          msg.ExpSpeed =  ExpVelSpeed;                   
          msg.ACCexp = speedcontrol.Output;
          com2veh_pub.publish(msg);
        }
    }
  }
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "com2veh");
	ros::NodeHandle n;
	com2veh_pub = n.advertise<control_msgs::com2veh>("com2vehmsg", 1000);
	control_vcu_sub_ = n.subscribe("/control/control_vcu",10,recvControlVCUCallback);
	int devid;

	uint8_t data[100];
	int len = 0;
	uint32_t delayTime = 0x0FFFFF;

	adcuDeviceType deviceType;
	int channel;

	adcuSDKInit();
	if (argc != 2)
	{
		printf("please input correct parameters<channelNumber>\n");
		return 0;
	}
	channel = atoi(argv[1]);
	if (channel <= 0 || channel >= ADCU_CHANNEL_MAX)
	{
		printf("please input correct channel number<%d ~ %d>\n", ADCU_CHANNEL_1, ADCU_CHANNEL_MAX - 1);
		return 0;
	}

	bzero(data, 100);
	adcuCanData canbuf;
	canbuf.ide = 0x00;
	canbuf.dlc = 0x08;
	canbuf.rtr = 0x00;
	canbuf.prio = 0x00;
	canbuf.id = 0x238;

	deviceType = adcuCAN;
	len = sizeof(adcuCanData);
	memcpy(data, (uint8_t *)&canbuf, len);
	delayTime = 1000 / 60;

	devid = adcuDevOpen(deviceType, (adcuDeviceChannel)channel);
	pthread_create(&ReadThread, NULL, readLoop, &devid);


	ros::Rate loop_rate(100);
	starttime = ros::Time::now().toSec();

	while (ros::ok())
	{
		if (adcuDevStatus(devid) == ADCU_DEV_STATUS_ABNORMAL)
		{
			break;
		}

		uint8_t send_buf[8]={0};
		if (SteerTurnTest)
		{
				AutodriveEnable=true;
				SteerEnable=true;
		}

		uint8_t WorkMode=AutodriveEnable, BrkReq=BrakeEnable, DriReq=AccelEnable, StrReq=SteerEnable;

		send_buf[0] = WorkMode<<6 | BrkReq<<3 | DriReq<<2 | StrReq<<1;
		float TrgDriAcc = ExpAcce;
		send_buf[4] = (uint8_t)(TrgDriAcc * 10.0f);
		float TrgBrkAcc = ExpDecel;
		send_buf[5] = (uint8_t)(TrgBrkAcc * 10.0f);
		float TrgStrAngle =ExpAngle;
		if (TrgStrAngle < 0.0f) {
			TrgStrAngle = -TrgStrAngle;
			send_buf[6] = ((uint16_t)(TrgStrAngle * 10.0f)) >>8 | 0x80;
		} 
		else 
		{
			send_buf[6] = ((uint16_t)(TrgStrAngle * 10.0f)) >>8;
		}
		//TrgStrAngle need 2 bytes and it is intel format          
		send_buf[7] = (uint16_t)(TrgStrAngle * 10.0f) & 0x00ff;
		for(int i = 0; i < canbuf.dlc; i++)
		{
			canbuf.can_data[i] = send_buf[i];
		}
		
		bzero(data, 100);
		memcpy(data, (uint8_t *)&canbuf, len);
		if (adcuDevWrite(devid, data, len) <= 0)
		{
			printf("main write error\n");
			goto __end;
		} 
		else
		{
			ROS_INFO("CAN1 TX ID:0x%08X data:0x %02X %02X %02X %02X %02X %02X %02X %02X", canbuf.id,canbuf.can_data[0],canbuf.can_data[1],canbuf.can_data[2],canbuf.can_data[3],canbuf.can_data[4],canbuf.can_data[5],canbuf.can_data[6],canbuf.can_data[7]);

		}
		
		//chatter_pub.publish(msg);
		ros::spinOnce();

		loop_rate.sleep();
	}
	__end:
		printf("dinit all \n");
		adcuDevClose(devid);
		adcuSDKDeinit();
		return 0;
}