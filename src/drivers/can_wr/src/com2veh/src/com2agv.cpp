#include "com2agv.h"

using namespace std;

uint32_t timecount = 0;
uint32_t starttime = 0;
uint32_t control_stamp;
ros::Publisher com2veh_pub;
ros::Subscriber control_vcu_sub_;
control_msgs::com2veh msg;
PID speedcontrol;

float Vkm2mConst = 1/3.6f;
float VelSpeed = 0.0f;        //车辆速度单位是： is: Km/h
float SteerAngle = 0.0f;
float ExpVelSpeed = 6.0f;
float ExpDecel = 0.0f;        //deceleration 
float ExpAcce = 0.0f;
float ExpAngle_F=0.0f;
float ExpAngle_R=0.0f;
bool BrakeEnable = false;
bool AccelEnable = false;
bool SteerEnable = false;

void paraminit(void)
{
  speedcontrol.kp = 0.8f;//0.8f;
  speedcontrol.ki = 0.0f;
  speedcontrol.kd = 0.0f;
  speedcontrol.I_max = 5000.0f;
  speedcontrol.D_max = 50.0f;
  speedcontrol.Out_max = 5.0f;
  speedcontrol.Out_min = -4.0f;

  Control2AGVInfo.AutoDrive_Enable = false;
  Control2AGVInfo.Dir_PRND = Park;
  Control2AGVInfo.LiftCmd = Stop;
}
void recvControlVCUCallback(const control_msgs::com2veh& msg)
{
	ExpVelSpeed = msg.ExpSpeed;			//读取期望速度
	ROS_INFO("readExpVelSpeed:%lf",ExpVelSpeed);
	ValueConstrain(ExpVelSpeed,0,ExpSpeed_MAX);
	Control2AGVInfo.VehAgl_F = msg.SteerAngle;	//读取期望转角
	ValueConstrain(Control2AGVInfo.VehAgl_F,-SteerAngle_MAX,SteerAngle_MAX);
	//ROS_INFO("readExpAngle:%lf",ExpAngle);
	//control_stamp = msg.header.stamp.toSec();
	Control2AGVInfo.AutoDrive_Enable = true;
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
        memcpy(candatareceive.candata, &buffer, length);
        if (candatareceive.canrecvbuf.id == VCU2C112)
        {
		uint8_t rec_buf[CAN_DATA_SIZE]={0};
		for(int i = 0; i < CAN_DATA_SIZE; i++)
		{
			rec_buf[i] = candatareceive.canrecvbuf.can_data[i];
			// printf(" %02X", rec_buf[i]);
		}
        // printf("\n");
		AGV2ControlInfo.ActualSpd = (float)(rec_buf[0]<<4 | rec_buf[1]>>4) * 0.01f;
		msg.ActualSpd = AGV2ControlInfo.ActualSpd;
		AGV2ControlInfo.ActualAgl_R = (float)(((rec_buf[1] & 0x0f)  << 6) | rec_buf[2] >> 2) * 0.1f - 45.0f;
		msg.ActualAgl_R = AGV2ControlInfo.ActualAgl_R;
		AGV2ControlInfo.ActualAgl_F = (float)(((rec_buf[2] & 0x03) << 8) | rec_buf[3] ) * 0.1f - 45.0f;
		msg.ActualAgl_F = AGV2ControlInfo.ActualAgl_F;
		AGV2ControlInfo.VEHMode = rec_buf[4] & 0x03;
		msg.VEHMode = AGV2ControlInfo.VEHMode;
		AGV2ControlInfo.VEHFlt = (rec_buf[4] & 0x0c) >> 2;
		msg.VEHFlt = AGV2ControlInfo.VEHFlt;
		AGV2ControlInfo.LiftStatus = (rec_buf[4] & 0x30) >> 4;
		msg.LiftStatus = AGV2ControlInfo.LiftStatus;
		AGV2ControlInfo.HVStatus = (rec_buf[4] & 0x40) >> 6;
		msg.HVStatus = AGV2ControlInfo.HVStatus;
		AGV2ControlInfo.EStopStatus = (rec_buf[4] & 0x80) >> 7;
		msg.EStopStatus = AGV2ControlInfo.EStopStatus;
		AGV2ControlInfo.EPBStatus = rec_buf[5] & 0x01;
		msg.EPBStatus = AGV2ControlInfo.EPBStatus;
		AGV2ControlInfo.Dir_PRND = (rec_buf[5] & 0x0e) >> 1;
		msg.Dir_PRND = AGV2ControlInfo.Dir_PRND;
		AGV2ControlInfo.SOC = (rec_buf[5] & 0x30) >> 4;
		msg.SOC = AGV2ControlInfo.SOC;
		AGV2ControlInfo.Rolling_Counter =  rec_buf[7] & 0x0f;
		msg.Rolling_Counter= AGV2ControlInfo.Rolling_Counter;
		pid_loop(&speedcontrol, ExpVelSpeed , AGV2ControlInfo.ActualSpd, ros::Time::now().toSec());
		//printf("speedOutput is : %f \n", speedcontrol.Output);
		// if (speedcontrol.Output > 0.02f)
		// {
		// 	ExpAcce = speedcontrol.Output;
		// 	ExpDecel = 0.0f;
		// 	AccelEnable = true;
		// 	BrakeEnable = false;
		// }
		// else if (speedcontrol.Output < -0.08f)
		// {
		// 	ExpAcce = 0.0f;
		// 	ExpDecel = -speedcontrol.Output;  // * 1.2f;
		// 	if (ExpDecel > 6.5f)
		// 	{
		// 		ExpDecel = 6.5f;
		// 	}

		// 	AccelEnable = false;
		// 	BrakeEnable = true;
		// }
		// else
		// {
		// 	ExpAcce = 0.0f;
		// 	ExpDecel = 0.0f;
		// 	AccelEnable = false;
		// 	BrakeEnable = false;
		// }
		
#ifdef SteerTurnTest
        Control2AGVInfo.VehAgl_F = turntest_Angle * sin((ros::Time::now().toSec() - starttime) / turntest_Period * M_PI);
        msg.ExpAngle = Control2AGVInfo.VehAgl_F;
#endif
		msg.ExpSpeed =  ExpVelSpeed;                   
		msg.ACCexp = speedcontrol.Output;
		com2veh_pub.publish(msg);
        }
    }
  }
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "com2agv");
	ros::NodeHandle n;
	com2veh_pub = n.advertise<control_msgs::com2veh>("com2vehmsg", 1000);
	control_vcu_sub_ = n.subscribe("/control/control_vcu",10,recvControlVCUCallback);
	int devid;

	uint8_t data[100];
	int len = 0;
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
	canbuf.id = VCU2C111;

	deviceType = adcuCAN;
	len = sizeof(adcuCanData);
	memcpy(data, (uint8_t *)&canbuf, len);

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
#ifdef SteerTurnTest
        Control2AGVInfo.AutoDrive_Enable=true;
#endif
		float Vel_Req_Buf = Control2AGVInfo.Vel_Req  * 100;
		send_buf[0] = 	(((uint16_t)Vel_Req_Buf) >> 4) & 0xff;
		send_buf[1] = 	(((uint16_t)Vel_Req_Buf) & 0x0f) << 4;
		float VehAgl_F_Buf = (Control2AGVInfo.VehAgl_F + 45.0f) * 10;
		send_buf[2] = 	(((int16_t)VehAgl_F_Buf) >> 2) & 0xff;
		send_buf[3] = 	(((int16_t)VehAgl_F_Buf) & 0x03) << 6 | 
						(Control2AGVInfo.Dir_PRND & 0x07) <<3 |
						Control2AGVInfo.LiftCmd & 0x07;
		float VehAgl_R_Buf = (Control2AGVInfo.VehAgl_R + 45.0f) * 10;
		send_buf[4] = 	(((int16_t)VehAgl_R_Buf) >> 2) & 0xff;
		send_buf[5] = 	(((int16_t)VehAgl_R_Buf) & 0x03) << 6 |
						Control2AGVInfo.AutoDrive_Enable <<2 | 
						Control2AGVInfo.EStop <<1 |
						Control2AGVInfo.EPB;
        if (++Control2AGVInfo.Rolling_Counter > 15)
        {
            Control2AGVInfo.Rolling_Counter = 0;
        }
        
		send_buf[7] = 	Control2AGVInfo.Rolling_Counter;
		
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
			// ROS_INFO("CAN1 TX ID:0x%08X data:0x %02X %02X %02X %02X %02X %02X %02X %02X", canbuf.id,canbuf.can_data[0],canbuf.can_data[1],canbuf.can_data[2],canbuf.can_data[3],canbuf.can_data[4],canbuf.can_data[5],canbuf.can_data[6],canbuf.can_data[7]);

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
