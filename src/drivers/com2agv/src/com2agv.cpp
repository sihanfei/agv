#include "com2agv.h"

using namespace std;
using namespace superg_agv;
using namespace drivers;

double VCU_starttime ;
double main_starttime;
double control_stamp;
ros::Publisher AGVSta_pub;
ros::Publisher AGVSta2_pub;
ros::Publisher com2veh_pub;
// ros::Publisher control_pub;
ros::Subscriber control_vcu_sub_;
ros::Subscriber plan_vcu_sub_;
ros::Subscriber operation_vcu_sub_;
ros::Subscriber KeyboardTeleop_sub_;
ros::Subscriber fsm_sub_;
control_msgs::ADControlAGV c2v_msg;
control_msgs::AGVStatus AGVSta_msg;

double Test_Dist = 0.0f;
float Test_Vel_Req;
bool vcu_valid = false;
bool keyboard_valid = false;
bool controlvcu_valid = false;
bool TestCycleStart = false;

/******param declare******/
float Test_MovePeriod_;
float Test_TurnPeriod_;
float Test_TurnAngle_;
float Test_StraightLength_;
float Test_SquareMinSpd_;
float Test_SquareMaxSpd_;
float Test_Acc_;
float Test_Time_;
/******end param declare******/

void paraminit(ros::NodeHandle &n)
{
	n.param("Test_MovePeriod", Test_MovePeriod_, 20.0f);
	n.param("Test_TurnPeriod", Test_TurnPeriod_, 20.0f);
	n.param("Test_TurnAngle", Test_TurnAngle_, 40.0f);
	n.param("Test_StraightLength", Test_StraightLength_, 50.0f);
	n.param("Test_SquareMinSpd", Test_SquareMinSpd_, 1.0f);
	n.param("Test_SquareMaxSpd", Test_SquareMaxSpd_, 2.0f);
	n.param("Test_Acc", Test_Acc_, 0.4f);

	ROS_INFO("custom param load successful!");
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

void vehicletestfuction(char sign, float dt)
{
	Test_Time_ += dt;
	switch (sign)
	{
		case -1:	//测试倒车
		case 1:		//测试前进
			Control2AGVInfo.VehAgl_F = Control2AGVInfo.VehAgl_R = 0.0f;
			if (Test_Dist< Test_StraightLength_)
			{
				float t1 = Test_SquareMaxSpd_ / Test_Acc_;
				float t2 = t1 + 0.5f * Test_MovePeriod_;
				float t3 = t1 + Test_MovePeriod_;
				float t4 = t1 + 1.5f * Test_MovePeriod_;
				float t5 = t4 + t1;
				if (Test_Time_ < t1)
				{
					Test_Vel_Req += Test_Acc_ * dt;
				}
				else if (Test_Time_ < t2)
				{
					Test_Vel_Req = Test_SquareMaxSpd_;
				}
				else if (Test_Time_ < t3)
				{
					Test_Vel_Req = Test_SquareMinSpd_;
				}
				else if (Test_Time_ < t4)
				{
					Test_Vel_Req = Test_SquareMaxSpd_;
				}
				else if (Test_Time_ < t5)
				{
					Test_Vel_Req -= Test_Acc_ * dt;
				}
				else
				{
					Test_StraightLength_ = 0.0f;
				}
				
				Test_Vel_Req = ValueConstrain(Test_Vel_Req,0,Test_SquareMaxSpd_);
				Control2AGVInfo.Vel_Req = sign * Test_Vel_Req;
				
				// Control2AGVInfo.Vel_Req = 3 * (sin((ros::Time::now().toSec() - main_starttime) / Test_MovePeriod_ * 2 * M_PI) + 1);
				
				// if (Control2AGVInfo.Vel_Req >= 3)
				// {
				// 	Control2AGVInfo.Vel_Req = sign * Test_SquareMinSpd_;
				// }
				// else
				// {
				// 	Control2AGVInfo.Vel_Req = sign * Test_SquareMaxSpd_;
				// }/**/
				
				Test_Dist += dt * AGV2ControlInfo.ActualSpd;
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
		case 2:		//测试前轮正弦转动
			Control2AGVInfo.VehAgl_F = Test_TurnAngle_ * sin((ros::Time::now().toSec() - main_starttime) / Test_TurnPeriod_ * 2 * M_PI);
			Control2AGVInfo.AutoDrive_Enable=true;
			break;
		case 3:		//测试后轮正弦转动
			Control2AGVInfo.VehAgl_R = Test_TurnAngle_ * sin((ros::Time::now().toSec() - main_starttime) / Test_TurnPeriod_ * 2 * M_PI);
			Control2AGVInfo.AutoDrive_Enable=true;
			break;
		case 4:		//测试前后轮同时正弦转动
			Control2AGVInfo.VehAgl_F = Test_TurnAngle_ * sin((ros::Time::now().toSec() - main_starttime) / Test_TurnPeriod_ * 2 * M_PI);
			Control2AGVInfo.VehAgl_R = Control2AGVInfo.VehAgl_F;
			Control2AGVInfo.AutoDrive_Enable=true;
			break;
		
		default:
			break;
	}
}

void recvControlVCUCallback(const control_msgs::ADControlAGV& msg)
{
	Control2AGVInfo.VehAgl_F = msg.VehAgl_F;	//读取期望转角
	Control2AGVInfo.VehAgl_F = ValueConstrain(Control2AGVInfo.VehAgl_F,-SteerAngle_MAX,SteerAngle_MAX);
	Control2AGVInfo.VehAgl_R = msg.VehAgl_R;	//读取期望转角
	Control2AGVInfo.VehAgl_R = ValueConstrain(Control2AGVInfo.VehAgl_R,-SteerAngle_MAX,SteerAngle_MAX);
	//Control2AGVInfo.Dir_PRND = (Gears)(msg.Dir_PRND_Tran);
	if (!keyboard_valid)
	{
		Control2AGVInfo.Vel_Req = msg.Vel_Req;			//读取期望速度
		Control2AGVInfo.Vel_Req = ValueConstrain(Control2AGVInfo.Vel_Req,0,ExpSpeed_MAX);
		Control2AGVInfo.AutoDrive_Enable = true;
		Control2AGVInfo.EStop = msg.EStop;
	}
	controlvcu_valid = true;	
}

void recvKeyboardTeleopCallback(const control_msgs::ADControlAGV& msg)
{
	Control2AGVInfo.Vel_Req = msg.Vel_Req;			//读取期望速度
	// ROS_INFO("read .Vel_Req:%lf",Control2AGVInfo.Vel_Req);
	Control2AGVInfo.Vel_Req = ValueConstrain(Control2AGVInfo.Vel_Req,-ExpSpeed_MAX,ExpSpeed_MAX);
	if (!controlvcu_valid)
	{
		Control2AGVInfo.VehAgl_F = msg.VehAgl_F;	//读取期望转角
		Control2AGVInfo.VehAgl_F = ValueConstrain(Control2AGVInfo.VehAgl_F,-SteerAngle_MAX,SteerAngle_MAX);
		Control2AGVInfo.VehAgl_R = msg.VehAgl_R;	//读取期望转角
		Control2AGVInfo.VehAgl_R = ValueConstrain(Control2AGVInfo.VehAgl_R,-SteerAngle_MAX,SteerAngle_MAX);
	}
	//ROS_INFO("readExpAngle:%lf",ExpAngle);
	Control2AGVInfo.AutoDrive_Enable = msg.AutoDrive_Enable;
	Control2AGVInfo.LiftCmd = (LeftModule) msg.LiftCmd;
	//Control2AGVInfo.Dir_PRND = (Gears) msg.Dir_PRND_Tran;
	Control2AGVInfo.EPB = msg.EPB;
	Control2AGVInfo.EStop = msg.EStop;
	keyboard_valid = true;
}

void recvPlanVCUCallback(const plan_msgs::DecisionInfo& msg)
{
	Control2AGVInfo.EStop = msg.CMD_estop;
}

void recvOperationVCUCallback(const control_msgs::ADControlAGV& msg)
{
	Control2AGVInfo.LiftCmd = (LeftModule) msg.LiftCmd;
}

//void recvfsmCallback(const hmi_msgs:: FsmControVcuDriver& msg)
//{
	//Control2AGVInfo.AutoDrive_Enable = msg.is_AD_status;
//}

pthread_t ReadThread;
void *readLoop(void *pdata)
{
  int deviceid = *(int *)pdata;
  int packageCounter = 0;
  bool recv_vcu_motor = false;
  adcuCanData canBuffer;
  control_msgs::AGVStatus2 vcu_wheel_spd;
  resetcontrolparam();

  AGV2P2 AGV2P2_;
//   control_msgs::AGVStatus agvstatus;
//   AgvWheelSpd agvwheelspd;

  while (1)
  {
    // uint8_t buffer[1024];
    int length = 0;
    length = adcuDevRead(deviceid, (uint8_t *)&canBuffer);
    if (length > 0)
    {
//        memcpy(candatareceive.candata, &buffer, length);
		bool recv_motor = false;
		uint8_t rec_buf[canBuffer.dlc]={0};
		// uint8_t rec_buf[]={0};

		// canBuffer.dlc = 8;
		// if(canBuffer.id == 0x112)
		// {
		// 	canBuffer.id = 0x113;
		// }
		// else
		// {
		// 	canBuffer.id = 0x112;
		// }
		// canBuffer.can_data[0] = 0x01;
		// canBuffer.can_data[1] = 0x02;
		// canBuffer.can_data[2] = 0x03;
		// canBuffer.can_data[3] = 0x04;
		// canBuffer.can_data[4] = 0x05;
		// canBuffer.can_data[5] = 0x06;
		// canBuffer.can_data[6] = 0x07;
		// canBuffer.can_data[7] = 0x08;

		// sleep(1);

		switch (canBuffer.id)
		{
			case VCU2C112:
				for(int i = 0; i < canBuffer.dlc; i++)
				{
					rec_buf[i] = canBuffer.can_data[i];
					// printf(" %02X", rec_buf[i]);
				}
				// printf("receive VCU2C112!\n");
				// printf("\n");
				AGV2ControlInfo.ActualSpd = (float)(rec_buf[0]<<4 | rec_buf[1]>>4) * 0.01f;
				AGVSta_msg.ActualSpd = AGV2ControlInfo.ActualSpd;
				AGV2ControlInfo.ActualAgl_R = (float)(((rec_buf[1] & 0x0f)  << 6) | rec_buf[2] >> 2) * 0.1f - 45.0f;
				AGVSta_msg.ActualAgl_R = AGV2ControlInfo.ActualAgl_R;
				AGV2ControlInfo.ActualAgl_F = (float)(((rec_buf[2] & 0x03) << 8) | rec_buf[3] ) * 0.1f - 45.0f;
				AGVSta_msg.ActualAgl_F = AGV2ControlInfo.ActualAgl_F;
				AGV2ControlInfo.VEHMode = rec_buf[4] & 0x03;
				AGVSta_msg.VEHMode = AGV2ControlInfo.VEHMode;
				AGV2ControlInfo.VEHFlt = (rec_buf[4] & 0x0c) >> 2;
				AGVSta_msg.VEHFlt = AGV2ControlInfo.VEHFlt;
				AGV2ControlInfo.LiftStatus = (rec_buf[4] & 0x30) >> 4;
				AGVSta_msg.LiftStatus = AGV2ControlInfo.LiftStatus;
				AGV2ControlInfo.HVStatus = (rec_buf[4] & 0x40) >> 6;
				AGVSta_msg.HVStatus = AGV2ControlInfo.HVStatus;
				AGV2ControlInfo.EStopStatus = (rec_buf[4] & 0x80) >> 7;
				AGVSta_msg.EStopStatus = AGV2ControlInfo.EStopStatus;
				AGV2ControlInfo.EPBStatus = rec_buf[5] & 0x01;
				AGVSta_msg.EPBStatus = AGV2ControlInfo.EPBStatus;
				AGV2ControlInfo.Dir_PRND = (rec_buf[5] & 0x0e) >> 1;
				AGVSta_msg.Dir_PRND = AGV2ControlInfo.Dir_PRND;
				AGV2ControlInfo.SOC = (rec_buf[5] & 0x30) >> 4;
				AGVSta_msg.SOC = AGV2ControlInfo.SOC;
				AGV2ControlInfo.Rolling_Counter =  rec_buf[7] & 0x0f;
				AGVSta_msg.Rolling_Counter= AGV2ControlInfo.Rolling_Counter;

				AGVSta_msg.header.stamp = ros::Time::now();
				AGVSta_msg.header.frame_id = "/driver/AGVStatus";
				AGVSta_pub.publish(AGVSta_msg);
				vcu_valid = true;
				VCU_starttime = ros::Time::now().toSec();

				AGV2P2_.setAgvstatus(AGVSta_msg);

				break;
			case VCU2C113:
				for(int i = 0; i < canBuffer.dlc; i++)
				{
					rec_buf[i] = canBuffer.can_data[i];
					// printf(" %02X", rec_buf[i]);
				}
				// printf("receive VCU2C113!\n");
				vcu_wheel_spd.WheelRotSpd_FL		= (rec_buf[1]<<8 | rec_buf[0]) - 12000;
				vcu_wheel_spd.WheelRotSpd_FR		= (rec_buf[3]<<8 | rec_buf[2]) - 12000;
				vcu_wheel_spd.WheelRotSpd_RL		= (rec_buf[5]<<8 | rec_buf[4]) - 12000;
				vcu_wheel_spd.WheelRotSpd_RR		= (rec_buf[7]<<8 | rec_buf[6]) - 12000;

				AGV2P2_.setAgvwheelspd(rec_buf,vcu_wheel_spd);
				AGV2P2_.agv2p2_(vcu_wheel_spd);

				vcu_wheel_spd.header.stamp 		= ros::Time::now();
				vcu_wheel_spd.header.frame_id 	= "/driver/AGVStatus2";

				AGVSta2_pub.publish(vcu_wheel_spd);
				recv_vcu_motor = true;

				// AGV2P2_.setAgvwheelspd(rec_buf);
				// AGV2P2_.agv2p2_(vcu_wheel_spd);

				break;
		}
		
    }
  }
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "com2agv");
	ros::NodeHandle n;
	AGVSta_pub = n.advertise<control_msgs::AGVStatus>("/drivers/com2agv/agv_status", 10);
	AGVSta2_pub = n.advertise<control_msgs::AGVStatus2>("/drivers/com2agv/agv_status2", 10);
	com2veh_pub = n.advertise<control_msgs::ADControlAGV>("/drivers/com2agv/ADControlAGV", 10);
	control_vcu_sub_ = n.subscribe("/control/control_agv",10,recvControlVCUCallback);
	plan_vcu_sub_ = n.subscribe("plan/decision_info",10,recvPlanVCUCallback);
	operation_vcu_sub_ =  n.subscribe("/non_plan/control_agv",10,recvOperationVCUCallback);
	KeyboardTeleop_sub_ = n.subscribe("/drivers/com2agv/teleopcmd",10,recvKeyboardTeleopCallback);
//	fsm_sub_ = n.subscribe("/fsm/fsm_control_vcu_driver",10,recvfsmCallback);
//	control_pub = n.advertise<std_msgs::Int8MultiArray>("/control/allinfo", 1000);
  	ros::Publisher agv_status_pub = n.advertise< status_msgs::NodeStatus >("/com2agv/com2agv_status", 10);
	status_msgs::NodeStatus ns;

	Relay relay;
	relay.openCanRelay(18,0);
	sleep(1);

	ns.node_name = ros::this_node::getName();
	ns.node_pid  = getpid();
	ns.state_num = 1;
	uint8_t pub_count = 0;
	int counter = 0;
	int devid;
	uint8_t data[100];
	uint8_t opt_buf[16];
	int len = 0;
	adcuDeviceType deviceType;
	int channel;
	adcuSDKInit();
	channel = VCUCANChannel;//atoi(argv[1]);
	if (channel <= 0 || channel >= ADCU_CHANNEL_MAX)
	{
		printf("please input correct channel number<%d ~ %d>\n", ADCU_CHANNEL_1, ADCU_CHANNEL_MAX - 1);
		return 0;
	}
	char sign_ = -2;
	if (argc > 2 && !strcmp(argv[1],"test"))
	{
		sign_ = atoi(argv[2]);
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

	if (sign_ != -2)
	{
		paraminit(n);
	}

	bzero(data, 100);
	adcuCanData canbuf;
	canbuf.ide = 0x00;			//IDE=0:Standard,IDE=1:Extended
	canbuf.dlc = 0x08;
	canbuf.rtr = 0x00;
	canbuf.prio = 0x00;
	canbuf.id = VCU2C111;

	deviceType = adcuCAN;
	len = sizeof(adcuCanData);
	memcpy(data, (uint8_t *)&canbuf, len);

	devid = adcuDevOpen(deviceType, (adcuDeviceChannel)channel);
	pthread_create(&ReadThread, NULL, readLoop, &devid);

	opt_buf[0] = channel - 1;
	opt_buf[1] = 3;				//BaudRate<1 ~ 7> : 100k 125k 250k 400k 500k 800k 1000k
	adcuDevSetOpt(devid,opt_buf,2);
	sleep(1);

	ros::Rate loop_rate(Freq_com2agv);
	main_starttime = ros::Time::now().toSec();
	double t_prev = main_starttime - 1.0 / (float)Freq_com2agv;

	relay.openCanRelay(18,1);

	while (ros::ok())
	{
		if (adcuDevStatus(devid) == ADCU_DEV_STATUS_ABNORMAL)
		{
			break;
		}

		if (sign_ != -2)
		{
			double t = ros::Time::now().toSec();
			float dt = t - t_prev;
			t_prev = t;
			vehicletestfuction(sign_, dt);
		}

		uint8_t send_buf[8]={0};
		float Vel_Req_Buf = 0;
		/**************档位切换逻辑****************/
		if (Control2AGVInfo.Vel_Req > 0.001f)
		{
			// if (AGV2ControlInfo.Dir_PRND == Drive)
			// {
				Control2AGVInfo.Dir_PRND = Drive;
				Vel_Req_Buf = Control2AGVInfo.Vel_Req * 100;
			// }
			// else
			// {
			// 	if (AGV2ControlInfo.Dir_PRND != Park || AGV2ControlInfo.ActualSpd > 0.001)
			// 	{
			// 		Control2AGVInfo.Dir_PRND = Park;
			// 		Vel_Req_Buf = 0;
			// 	}
			// 	else
			// 	{
			// 		Control2AGVInfo.Dir_PRND = Drive;
			// 		Vel_Req_Buf = Control2AGVInfo.Vel_Req * 100;
			// 	}
			// }
		}
		else if (Control2AGVInfo.Vel_Req < -0.001f)
		{
			// if (AGV2ControlInfo.Dir_PRND == Reverse)
			// {
				Control2AGVInfo.Dir_PRND = Reverse;
				Vel_Req_Buf = (-Control2AGVInfo.Vel_Req) * 100;
			// }
			// else
			// {
			// 	if (AGV2ControlInfo.Dir_PRND != Park || AGV2ControlInfo.ActualSpd > 0.001)
			// 	{
			// 		Control2AGVInfo.Dir_PRND = Park;
			// 		Vel_Req_Buf = 0;
			// 	}
			// 	else
			// 	{
			// 		Control2AGVInfo.Dir_PRND = Reverse;
			// 		Vel_Req_Buf = (-Control2AGVInfo.Vel_Req) * 100;
			// 	}
			// }
		}
		else
		{
			Vel_Req_Buf = 0;
			if (AGV2ControlInfo.ActualSpd < 0.001)
			{
				Control2AGVInfo.Dir_PRND = Park;
			}
			
		}
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
		//if (vcu_valid) {}
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
		com2veh_pub.publish(c2v_msg);

		if (vcu_valid && (ros::Time::now().toSec() - VCU_starttime) > VCU_timeout)
		{
			resetcontrolparam();
			vcu_valid = false;
		}
		//发布功能安全数据给monitor
		if (pub_count == Freq_pubstatus)
		{
			status_msgs::SafetyStatus ss;
			ss.values.clear();
			ns.status.clear();
			counter++;
			common_msgs::KeyValue kv;
			ss.message_code		= "I0401001";
			ss.counter      	= counter;
			ss.hardware_id  	= 0;
			ss.value_num    	= 2;
			kv.key       		= "Agv_status";
			kv.valuetype 		= 2;
			kv.value     		= to_string(AGV2ControlInfo.VEHMode);
			ss.values.push_back(kv);
			kv.key       		= "ad_enbale_status";
			kv.valuetype 		= 2;
			kv.value     		= to_string(Control2AGVInfo.AutoDrive_Enable);
			ss.values.push_back(kv);
			ns.status.push_back(ss);
			if (!vcu_valid)
			{
				ss.values.clear();
				ss.message_code		= "F0401001";
				ss.counter      	= counter;
				ss.hardware_id  	= 0;
				ss.value_num    	= 1;
				kv.key       		= "Agv_status";
				kv.valuetype 		= 2;
				kv.value     		= "VCU has disconnected";
				ss.values.push_back(kv);
				ns.status.push_back(ss);
			}
			ns.header.stamp 	= ros::Time::now();
			ns.header.frame_id 	= " ";
			agv_status_pub.publish(ns);
			pub_count = 0;
		}
		pub_count++;
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	__end:
		printf("dinit all \n");
		adcuDevClose(devid);
		adcuSDKDeinit();
		relay.openCanRelay(18,0);
		return 0;
}
