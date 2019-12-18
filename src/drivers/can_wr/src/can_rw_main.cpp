#include "can_drivers.h"
// #include "p2.h"
// #include "camera.h"
// #include "ultrasonic.h"
// #include "mobileye.h"
// #include "power_control.h"
// #include "driver_monitor.h"
#include "node_status.h"
#include "can_node.h"
#include "agv2p2.h"

using namespace std;
using namespace superg_agv;
using namespace drivers;

// std::queue<string> Driver_Monitor::q_sensor_type;
// std::queue<Safety_Status> Driver_Monitor::q_node_malfunction;

int array_dev[8] = {0};
uint8_t array_dev_count = 0;

void sighandle(int sig)
{
  for(uint8_t k = 0;k < array_dev_count;++k)
  {
    if(ADCU_DEV_STATUS_ABNORMAL != adcuDevStatus(array_dev[k]))
    {
      if(ADCU_DEV_STATUS_NORMAL == adcuDevClose(array_dev[k]))
      {
        ROS_INFO("devid:%d is close~!",array_dev[k]);
      }
      else
      {
        ROS_INFO("devid:%d can not close~!",array_dev[k]);
      }
    }
  }
  adcuSDKDeinit();
  exit(0);
}

int main(int argc,char* argv[])
{

  ros::init(argc, argv, "can_rw");
  ros::NodeHandle nh;

  signal(SIGINT, sighandle);

  FILE* fp = NULL;
  char buf[1024] = {0};
  string system_out_str = "";
  do
  {
    fp = popen("pgrep adcuServer", "r");
    if(NULL == fp)
    {
        perror("popen");
        exit(1);
    }
    fgets(buf, 1024, fp);
    system_out_str = buf;
    if(!system_out_str.length())
    {
      ROS_ERROR("Please check the adcuServer is alive!");
    }
    pclose(fp);
    sleep(1);
  }while(!system_out_str.length());

  usleep(1000 * 2000);

  // cout << argc << endl;
  if(argc > 1)
  {
    for(int i = 1; i < argc;++i)
    {
      cout << argv[i] << endl;
    }
  }
  //读取can配置
  auto cpu_num = thread::hardware_concurrency();
  ROS_INFO("CPU num is : %d", cpu_num);
  
  Sensor_Info sensor_info_;

  adcuSDKInit();

  usleep(1000 * 2000);

  int channel;
  int devid = 1;
  uint8_t opt_buff[16] = {0};

  CANDrivers can_rw;
  Driver_Monitor driver_monitor;
  // P2 p2;
  // CAMERA camera;
  // ULTRASONIC ultrasonic;
  // MOBILEYE mobileye;

  // node_status node_status_;
  CAN_Node CAN_Node_;

  AGV2P2 AGV2P2_(nh);

  usleep(1000 * 1000);
  printf("open channel\n");

  adcuDeviceType deviceType;

  deviceType = adcuCAN;

  uint8_t devid_count = 0;

  if(argc > 1)
  {
    for(int i = 1; i < argc;++i)
    {
      int sensor = atoi(argv[i]);
      switch(sensor)
      {
        case CHANNEL_P2:
          {
            thread t_p2_send(std::bind(&CAN_Node::p2_Sender, &CAN_Node_));
            t_p2_send.detach();

            channel                 = CHANNEL_P2;
            devid                   = adcuDevOpen(deviceType, ( adcuDeviceChannel )channel);
            opt_buff[0]             = channel - 1;
            opt_buff[1]             = 5;
            adcuDevSetOpt(devid,opt_buff,2);
            can_rw.write_num[devid_count++] = devid;
            array_dev[array_dev_count] = devid;
            printf("open %d %d\n", channel, devid);
            thread can_readp2(std::bind(&CAN_Node::CANP2, &CAN_Node_, channel ,devid));
            can_readp2.detach();
            thread can_wirtep2(std::bind(&AGV2P2::agv2p2, &AGV2P2_, channel ,devid));
            can_wirtep2.detach();
            sensor_info_.sensor_name    = "p2";
            sensor_info_.sensor_channel = CHANNEL_P2;
            sensor_info_.devid = devid;
            CAN_Node_.sensor_info_push_back(sensor_info_);
          }
          break;
        case CHANNEL_CAMERA_1:
          {
            thread t_camera_send(std::bind(&CAN_Node::camera_Sender, &CAN_Node_));
            t_camera_send.detach();

            channel                 = CHANNEL_CAMERA_1;
            devid                   = adcuDevOpen(deviceType, ( adcuDeviceChannel )channel);
            opt_buff[0]             = channel - 1;
            opt_buff[1]             = 5;
            adcuDevSetOpt(devid,opt_buff,2);
            can_rw.write_num[devid_count++] = devid;
            array_dev[array_dev_count] = devid;
            printf("open %d %d\n", channel, devid);
            thread can_readcamera1(std::bind(&CAN_Node::CANCamera, &CAN_Node_, channel, devid));
            can_readcamera1.detach();
            sensor_info_.sensor_name    = "camera_1";
            sensor_info_.sensor_channel = CHANNEL_CAMERA_1;
            sensor_info_.devid = devid;
            CAN_Node_.sensor_info_push_back(sensor_info_);

            channel                 = CHANNEL_CAMERA_2;
            devid                   = adcuDevOpen(deviceType, ( adcuDeviceChannel )channel);
            opt_buff[0]             = channel - 1;
            opt_buff[1]             = 5;
            adcuDevSetOpt(devid,opt_buff,2);
            can_rw.write_num[devid_count++] = devid;
            array_dev[array_dev_count] = devid;
            printf("open %d %d\n", channel, devid);
            thread can_readcamera2(std::bind(&CAN_Node::CANCamera, &CAN_Node_, channel, devid));
            can_readcamera2.detach();
            sensor_info_.sensor_name    = "camera_2";
            sensor_info_.sensor_channel = CHANNEL_CAMERA_2;
            sensor_info_.devid = devid;
            CAN_Node_.sensor_info_push_back(sensor_info_);
          }
          break;
        case CHANNEL_ULTRASONIC:
          {
            thread t_ultrasonic_send(std::bind(&CAN_Node::ultrasonic_Sender, &CAN_Node_));
            t_ultrasonic_send.detach();

            channel                 = CHANNEL_ULTRASONIC;
            devid                   = adcuDevOpen(deviceType, ( adcuDeviceChannel )channel);
            opt_buff[0]             = channel - 1;
            opt_buff[1]             = 5;
            adcuDevSetOpt(devid,opt_buff,2);
            can_rw.write_num[devid_count++] = devid;
            array_dev[array_dev_count] = devid;
            printf("open %d %d\n", channel, devid);
            thread can_readultrasonic(std::bind(&CAN_Node::CANUltrasonic, &CAN_Node_, channel, devid));
            can_readultrasonic.detach();
            sensor_info_.sensor_name    = "ultrasonic";
            sensor_info_.sensor_channel = CHANNEL_ULTRASONIC;
            sensor_info_.devid = devid;
            CAN_Node_.sensor_info_push_back(sensor_info_);
          }
          break;
        default:
          break;
      }
    }
  }
  else
  {
    thread t_p2_send(std::bind(&CAN_Node::p2_Sender, &CAN_Node_));
    t_p2_send.detach();

    thread t_camera_send(std::bind(&CAN_Node::camera_Sender, &CAN_Node_));
    t_camera_send.detach();

    thread t_ultrasonic_send(std::bind(&CAN_Node::ultrasonic_Sender, &CAN_Node_));
    t_ultrasonic_send.detach();

    channel                 = CHANNEL_P2;
    devid                   = adcuDevOpen(deviceType, ( adcuDeviceChannel )channel);
    opt_buff[0]             = channel - 1;
    opt_buff[1]             = 5;
    adcuDevSetOpt(devid,opt_buff,2);
    can_rw.write_num[devid_count++] = devid;
    array_dev[array_dev_count] = devid;
    printf("open %d %d\n", channel, devid);
    thread can_readp2(std::bind(&CAN_Node::CANP2, &CAN_Node_, channel ,devid));
    can_readp2.detach();
    thread can_wirtep2(std::bind(&AGV2P2::agv2p2, &AGV2P2_, channel ,devid));
    can_wirtep2.detach();
    sensor_info_.sensor_name    = "p2";
    sensor_info_.sensor_channel = CHANNEL_P2;
    sensor_info_.devid = devid;
//    CAN_Node_.sensor_info_push_back(sensor_info_);

    channel                 = CHANNEL_CAMERA_1;
    devid                   = adcuDevOpen(deviceType, ( adcuDeviceChannel )channel);
    opt_buff[0]             = channel - 1;
    opt_buff[1]             = 5;
    adcuDevSetOpt(devid,opt_buff,2);
    can_rw.write_num[devid_count++] = devid;
    array_dev[array_dev_count] = devid;
    printf("open %d %d\n", channel, devid);
    thread can_readcamera1(std::bind(&CAN_Node::CANCamera, &CAN_Node_, channel, devid));
    can_readcamera1.detach();
    sensor_info_.sensor_name    = "camera_1";
    sensor_info_.sensor_channel = CHANNEL_CAMERA_1;
    sensor_info_.devid = devid;
//    CAN_Node_.sensor_info_push_back(sensor_info_);


    channel                 = CHANNEL_CAMERA_2;
    devid                   = adcuDevOpen(deviceType, ( adcuDeviceChannel )channel);
    opt_buff[0]             = channel - 1;
    opt_buff[1]             = 5;
    adcuDevSetOpt(devid,opt_buff,2);
    can_rw.write_num[devid_count++] = devid;
    array_dev[array_dev_count] = devid;
    printf("open %d %d\n", channel, devid);
    thread can_readcamera2(std::bind(&CAN_Node::CANCamera, &CAN_Node_, channel, devid));
    can_readcamera2.detach();
    sensor_info_.sensor_name    = "camera_2";
    sensor_info_.sensor_channel = CHANNEL_CAMERA_2;
    sensor_info_.devid = devid;
//    CAN_Node_.sensor_info_push_back(sensor_info_);

    channel                 = CHANNEL_ULTRASONIC;
    devid                   = adcuDevOpen(deviceType, ( adcuDeviceChannel )channel);
    opt_buff[0]             = channel - 1;
    opt_buff[1]             = 5;
    adcuDevSetOpt(devid,opt_buff,2);
    can_rw.write_num[devid_count++] = devid;
    array_dev[array_dev_count] = devid;
    printf("open %d %d\n", channel, devid);
    thread can_readultrasonic(std::bind(&CAN_Node::CANUltrasonic, &CAN_Node_, channel, devid));
    can_readultrasonic.detach();
    sensor_info_.sensor_name    = "ultrasonic";
    sensor_info_.sensor_channel = CHANNEL_ULTRASONIC;
    sensor_info_.devid = devid;
//    CAN_Node_.sensor_info_push_back(sensor_info_);
  }


  // adcuDeviceType deviceType;

  // deviceType = adcuCAN;

  // channel                 = CHANNEL_P2;
  // devid                   = adcuDevOpen(deviceType, ( adcuDeviceChannel )channel);
  // can_rw.write_num[devid] = channel;
  // printf("open %d %d\n", channel, devid);
  // thread can_readp2(std::bind(&CAN_Node::CANP2, &CAN_Node_, channel ,devid));
  // can_readp2.detach();
  // sensor_info_.sensor_name    = "p2";
  // sensor_info_.sensor_channel = CHANNEL_P2;
  // sensor_info_.devid = devid;
  // CAN_Node_.sensor_info_push_back(sensor_info_);


  // channel                 = CHANNEL_MOBILEYE;
  // devid                   = adcuDevOpen(deviceType, ( adcuDeviceChannel )channel);
  // can_rw.write_num[devid] = channel;
  // printf("open %d %d\n", channel, devid);
  // thread can_readmoobileye(std::bind(&CAN_Node::CANMobileye, &CAN_Node_, channel, devid));
  // can_readmoobileye.detach();
  // sensor_info_.sensor_name    = "mobileye";
  // sensor_info_.sensor_channel = CHANNEL_MOBILEYE;
  // sensor_info_.devid = devid;
  // CAN_Node_.sensor_info_push_back(sensor_info_);

//  thread t_channel_check(std::bind(&CAN_Node::channel_check, &CAN_Node_));
//  t_channel_check.detach();

  thread t_monitor(std::bind(&Driver_Monitor::frequency_monitor, &driver_monitor));
  t_monitor.detach();

  thread node_status(std::bind(&CAN_Node::node_status_pub, &CAN_Node_));
  node_status.detach();


/*
  channel                 = 7;
  devid                   = adcuDevOpen(deviceType, ( adcuDeviceChannel )channel);
  can_rw.write_num[devid] = channel;
  printf("open %d %d\n", channel, devid);
  thread can_write(std::bind(&CANDrivers::writeCAN, &can_rw, devid));
  can_write.detach();
*/

//  channel                 = 3;
//  devid                   = adcuDevOpen(deviceType, ( adcuDeviceChannel )channel);
//  can_rw.write_num[devid] = channel;
//  printf("open %d %d\n", channel, devid);
//  thread can_write(std::bind(&CANDrivers::writeCAN, &can_rw, devid));
//  can_write.detach();

  

  printf("\nmain set tip\n");
  for (size_t loop_i = 0; loop_i < MAX_CAN_BUF_NUM; loop_i++)
  {
    can_rw.begain_tip[MAX_CAN_BUF_NUM - loop_i - 1] = 1;
  }
  ROS_INFO("main sleep");
  usleep(1000 * 500);
  ROS_INFO("main ~~");
  while (ros::ok())
  {
    usleep(1000 * 1000);
  }

  for(uint8_t k = 0;k < devid_count;++k)
  {
    if(ADCU_DEV_STATUS_ABNORMAL != adcuDevStatus(can_rw.write_num[k]))
    {
      if(ADCU_DEV_STATUS_NORMAL == adcuDevClose(can_rw.write_num[k]))
      {
        ROS_INFO("devid:%d is close~!",can_rw.write_num[k]);
      }
      else
      {
        ROS_INFO("devid:%d can not close~!",can_rw.write_num[k]);
      }
    }
  }

  usleep(1000 * 2000);
  adcuSDKDeinit();

  for (size_t i = 0; i < 10; i++)
  {
    usleep(1000 * 1000);
    ROS_INFO_STREAM("main will closed after " << (10 - i));
  }

  ROS_INFO("shutting down!\n");
  ros::shutdown();
  return 0;
  // thread mythread(thread_function);
}
