#include "com2agv.h"

using namespace std;

double VCU_starttime ;
double main_starttime;
double control_stamp;
ros::Publisher com2veh_pub;
ros::Subscriber control_vcu_sub_;
ros::Subscriber KeyboardTeleop_sub_;
control_msgs::com2veh c2v_msg;
PID speedcontrol;

#ifdef Test_StraightLength
double Test_Dist = 0.0f;
#endif
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
bool VCU_Valid = false;

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
void resetcontrolparam(void)
{
  Control2AGVInfo.AutoDrive_Enable = false;
  Control2AGVInfo.Dir_PRND = Park;
  Control2AGVInfo.LiftCmd = Stop;
  Control2AGVInfo.Vel_Req = 0;
  Control2AGVInfo.VehAgl_F = 0;
  Control2AGVInfo.VehAgl_R = 0;
  Control2AGVInfo.EPB = 1;			//num 1:松开 0:闭合
  
}
#ifdef Test_TurnPeriod
void vehicletestfuction(char sign)
{
	switch (sign)
	{
		case -1:
		case 1:
			Control2AGVInfo.VehAgl_F = Control2AGVInfo.VehAgl_R = 0.0f;
			if (Test_Dist< Test_StraightLength)
			{
				//Control2AGVInfo.Dir_PRND = Drive;
				//Control2AGVInfo.Dir_PRND = Reverse;
				//Control2AGVInfo.Vel_Req= ExpSpeed_MAX;
				Control2AGVInfo.Vel_Req = 3 * (sin((ros::Time::now().toSec() - main_starttime) / Test_TurnPeriod * 2 * M_PI) + 1);
				
				if (Control2AGVInfo.Vel_Req >= 3)
				{
					Control2AGVInfo.Vel_Req = sign * 1.5;
				}
				else
				{
					Control2AGVInfo.Vel_Req = sign * 3;
				}/**/
				
				Test_Dist += 1/(float)Freq_com2agv * AGV2ControlInfo.ActualSpd;
				ROS_INFO("Test_Dist is: %lf", Test_Dist);
				Control2AGVInfo.AutoDrive_Enable=true;
			}
			else
			{
				Control2AGVInfo.Vel_Req= 0.0f;
				//Control2AGVInfo.Dir_PRND = Park;
				Control2AGVInfo.AutoDrive_Enable=true;
			}
			break;
		case 2:
			Control2AGVInfo.VehAgl_F = Test_TurnAngle * sin((ros::Time::now().toSec() - main_starttime) / Test_TurnPeriod * 2 * M_PI);
			Control2AGVInfo.AutoDrive_Enable=true;
			break;
		case 3:
			Control2AGVInfo.VehAgl_R = Test_TurnAngle * sin((ros::Time::now().toSec() - main_starttime) / Test_TurnPeriod * 2 * M_PI);
			Control2AGVInfo.AutoDrive_Enable=true;
			break;
		case 4:
			Control2AGVInfo.VehAgl_F = Test_TurnAngle * sin((ros::Time::now().toSec() - main_starttime) / Test_TurnPeriod * 2 * M_PI);
			Control2AGVInfo.VehAgl_R = Control2AGVInfo.VehAgl_F;
			Control2AGVInfo.AutoDrive_Enable=true;
			break;
		
		default:
			break;
	}
}
#endif
void recvControlVCUCallback(const control_msgs::com2veh& msg)
{
	Control2AGVInfo.Vel_Req = msg.Vel_Req;			//读取期望速度
	ROS_INFO("read .Vel_Req:%lf",Control2AGVInfo.Vel_Req);
	ValueConstrain(Control2AGVInfo.Vel_Req,0,ExpSpeed_MAX);
	Control2AGVInfo.VehAgl_F = msg.VehAgl_F;	//读取期望转角
	ValueConstrain(Control2AGVInfo.VehAgl_F,-SteerAngle_MAX,SteerAngle_MAX);
	Control2AGVInfo.VehAgl_R = msg.VehAgl_R;	//读取期望转角
	ValueConstrain(Control2AGVInfo.VehAgl_R,-SteerAngle_MAX,SteerAngle_MAX);
	//Control2AGVInfo.Dir_PRND = (Gears)(msg.Dir_PRND_Tran);
	//ROS_INFO("readExpAngle:%lf",ExpAngle);
	//control_stamp = msg.header.stamp.toSec();
	Control2AGVInfo.AutoDrive_Enable = true;
	SteerEnable = true;
}

void recvKeyboardTeleopCallback(const control_msgs::com2veh& msg)
{
	Control2AGVInfo.Vel_Req = msg.Vel_Req;			//读取期望速度
	// ROS_INFO("read .Vel_Req:%lf",Control2AGVInfo.Vel_Req);
	ValueConstrain(Control2AGVInfo.Vel_Req,-ExpSpeed_MAX,ExpSpeed_MAX);
	
	Control2AGVInfo.VehAgl_F = msg.VehAgl_F;	//读取期望转角
	ValueConstrain(Control2AGVInfo.VehAgl_F,-SteerAngle_MAX,SteerAngle_MAX);
	Control2AGVInfo.VehAgl_R = msg.VehAgl_R;	//读取期望转角
	ValueConstrain(Control2AGVInfo.VehAgl_R,-SteerAngle_MAX,SteerAngle_MAX);
	//ROS_INFO("readExpAngle:%lf",ExpAngle);
	//control_stamp = msg.header.stamp.toSec();
	Control2AGVInfo.AutoDrive_Enable = msg.AutoDrive_Enable;
	Control2AGVInfo.LiftCmd = (LeftModule) msg.LiftCmd;
	//Control2AGVInfo.Dir_PRND = (Gears) msg.Dir_PRND_Tran;
	Control2AGVInfo.EPB = msg.EPB;
	Control2AGVInfo.EStop = msg.EStop;
	SteerEnable = true;

}
pthread_t ReadThread;
void *readLoop(void *pdata)
{
  int deviceid = *(int *)pdata;
  int packageCounter = 0;
  paraminit();
  resetcontrolparam();

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
			c2v_msg.ActualSpd = AGV2ControlInfo.ActualSpd;
			AGV2ControlInfo.ActualAgl_R = (float)(((rec_buf[1] & 0x0f)  << 6) | rec_buf[2] >> 2) * 0.1f - 45.0f;
			c2v_msg.ActualAgl_R = AGV2ControlInfo.ActualAgl_R;
			AGV2ControlInfo.ActualAgl_F = (float)(((rec_buf[2] & 0x03) << 8) | rec_buf[3] ) * 0.1f - 45.0f;
			c2v_msg.ActualAgl_F = AGV2ControlInfo.ActualAgl_F;
			AGV2ControlInfo.VEHMode = rec_buf[4] & 0x03;
			c2v_msg.VEHMode = AGV2ControlInfo.VEHMode;
			AGV2ControlInfo.VEHFlt = (rec_buf[4] & 0x0c) >> 2;
			c2v_msg.VEHFlt = AGV2ControlInfo.VEHFlt;
			AGV2ControlInfo.LiftStatus = (rec_buf[4] & 0x30) >> 4;
			c2v_msg.LiftStatus = AGV2ControlInfo.LiftStatus;
			AGV2ControlInfo.HVStatus = (rec_buf[4] & 0x40) >> 6;
			c2v_msg.HVStatus = AGV2ControlInfo.HVStatus;
			AGV2ControlInfo.EStopStatus = (rec_buf[4] & 0x80) >> 7;
			c2v_msg.EStopStatus = AGV2ControlInfo.EStopStatus;
			AGV2ControlInfo.EPBStatus = rec_buf[5] & 0x01;
			c2v_msg.EPBStatus = AGV2ControlInfo.EPBStatus;
			AGV2ControlInfo.Dir_PRND = (rec_buf[5] & 0x0e) >> 1;
			c2v_msg.Dir_PRND = AGV2ControlInfo.Dir_PRND;
			AGV2ControlInfo.SOC = (rec_buf[5] & 0x30) >> 4;
			c2v_msg.SOC = AGV2ControlInfo.SOC;
			AGV2ControlInfo.Rolling_Counter =  rec_buf[7] & 0x0f;
			c2v_msg.Rolling_Counter= AGV2ControlInfo.Rolling_Counter;
			pid_loop(&speedcontrol, Control2AGVInfo.Vel_Req , AGV2ControlInfo.ActualSpd, ros::Time::now().toSec());	

			c2v_msg.ACCexp = speedcontrol.Output;
			com2veh_pub.publish(c2v_msg);
			VCU_Valid = true;
			VCU_starttime = ros::Time::now().toSec();
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
	KeyboardTeleop_sub_ = n.subscribe("com2agv/teleopcmd",10,recvKeyboardTeleopCallback);
	int devid;

	uint8_t data[100];
	int len = 0;
	adcuDeviceType deviceType;
	int channel;
	adcuSDKInit();
	if (argc < 2)
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
	char sign_ = -2;
	if (argc > 2 && !strcmp(argv[2],"test"))
	{
		sign_ = atoi(argv[3]);
		if (sign_ == 1)
		{
			printf("The AGV will into test mode and gears is Drive\n");
		}
		else if (sign_ == -1)
		{
			printf("The AGV will into test mode and gears is Reverse\n");
		}
		else if (sign_>1 && sign_<5)
		{
			printf("The AGV will into test mode and will turn steer\n");
		}
		else
		{
			printf("please input correct sign, 1 mean drive, -1 mean reverse you input:%d\n",sign_);
		    sign_ = -2;
		}
	}	
	else
	{
		sign_ = -2;
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


	ros::Rate loop_rate(Freq_com2agv);
	main_starttime = ros::Time::now().toSec();

	while (ros::ok())
	{
		if (adcuDevStatus(devid) == ADCU_DEV_STATUS_ABNORMAL)
		{
			break;
		}
#ifdef Test_TurnPeriod
		
		if (sign_ != -2)
		{
			vehicletestfuction(sign_);
		}
#endif

		uint8_t send_buf[8]={0};
		float Vel_Req_Buf = 0;
		if (Control2AGVInfo.Vel_Req > 0.001f)
		{
			if (AGV2ControlInfo.Dir_PRND == Drive)
			{
				Control2AGVInfo.Dir_PRND = Drive;
				Vel_Req_Buf = Control2AGVInfo.Vel_Req * 100;
			}
			else
			{
				if (AGV2ControlInfo.Dir_PRND != Park || AGV2ControlInfo.ActualSpd > 0.001)
				{
					Control2AGVInfo.Dir_PRND = Park;
					Vel_Req_Buf = 0;
				}
				else
				{
					Control2AGVInfo.Dir_PRND = Drive;
					Vel_Req_Buf = Control2AGVInfo.Vel_Req * 100;
				}
			}
		}
		else if (Control2AGVInfo.Vel_Req < -0.001f)
		{
			if (AGV2ControlInfo.Dir_PRND == Reverse)
			{
				Control2AGVInfo.Dir_PRND = Reverse;
				Vel_Req_Buf = (-Control2AGVInfo.Vel_Req) * 100;
			}
			else
			{
				if (AGV2ControlInfo.Dir_PRND != Park || AGV2ControlInfo.ActualSpd > 0.001)
				{
					Control2AGVInfo.Dir_PRND = Park;
					Vel_Req_Buf = 0;
				}
				else
				{
					Control2AGVInfo.Dir_PRND = Reverse;
					Vel_Req_Buf = (-Control2AGVInfo.Vel_Req) * 100;
				}
			}
		}
		else
		{
			//Control2AGVInfo.Dir_PRND = Park;
			Vel_Req_Buf = 0;
		}
		ValueConstrain(Vel_Req_Buf,0,ExpSpeed_MAX);
		send_buf[0] = 	(((uint16_t)Vel_Req_Buf) >> 4) & 0xff;
		send_buf[1] = 	(((uint16_t)Vel_Req_Buf) & 0x0f) << 4;
		float VehAgl_F_Buf = (Control2AGVInfo.VehAgl_F + 45.0f) * 10 + 0.5;
		send_buf[2] = 	(((int16_t)VehAgl_F_Buf) >> 2) & 0xff;
		send_buf[3] = 	(((int16_t)VehAgl_F_Buf) & 0x03) << 6 | 
						(Control2AGVInfo.Dir_PRND & 0x07) <<3 |
						Control2AGVInfo.LiftCmd & 0x07;
		float VehAgl_R_Buf = (Control2AGVInfo.VehAgl_R + 45.0f) * 10 + 0.5;
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
		//TODO: 如果调试完成加入VCU通信有效判断
		//if (VCU_Valid) {}
		if (adcuDevWrite(devid, data, len) <= 0)	//发送CAN报文给VCU控制车辆
		{
			printf("main write error\n");
			goto __end;
		} 
		else
		{
			// ROS_INFO("CAN1 TX ID:0x%08X data:0x %02X %02X %02X %02X %02X %02X %02X %02X", canbuf.id,canbuf.can_data[0],canbuf.can_data[1],canbuf.can_data[2],canbuf.can_data[3],canbuf.can_data[4],canbuf.can_data[5],canbuf.can_data[6],canbuf.can_data[7]);

		}

		c2v_msg.VehAgl_F = Control2AGVInfo.VehAgl_F;
		c2v_msg.VehAgl_R = Control2AGVInfo.VehAgl_R;
		c2v_msg.Vel_Req = Vel_Req_Buf/100.0f;//Control2AGVInfo.Vel_Req;
		c2v_msg.LiftCmd = Control2AGVInfo.LiftCmd;
		c2v_msg.Dir_PRND_Tran = Control2AGVInfo.Dir_PRND;
		c2v_msg.EPB = Control2AGVInfo.EPB;
		c2v_msg.EStop = Control2AGVInfo.EStop;
		c2v_msg.AutoDrive_Enable = Control2AGVInfo.AutoDrive_Enable;
		c2v_msg.header.stamp = ros::Time::now();
		c2v_msg.header.frame_id = "/com2agv";
		if (VCU_Valid && (ros::Time::now().toSec() - VCU_starttime) > VCU_timeout)
		{
			resetcontrolparam();
			VCU_Valid = false;
		}
		if (!VCU_Valid)
		{
			com2veh_pub.publish(c2v_msg);
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	__end:
		printf("dinit all \n");
		adcuDevClose(devid);
		adcuSDKDeinit();
		return 0;
}
