#include "ultrasonic.h"

namespace superg_agv
{
namespace drivers
{

ULTRASONIC::ULTRASONIC()
{
  m_pHandlerMap.insert(make_pair(ULTRASONIC_DATA_1, &ULTRASONIC::ultrasonicDataParse));
  m_pHandlerMap.insert(make_pair(ULTRASONIC_DATA_2, &ULTRASONIC::ultrasonicDataParse));
  m_pHandlerMap.insert(make_pair(ULTRASONIC_DATA_3, &ULTRASONIC::ultrasonicDataParse));
  m_pHandlerMap.insert(make_pair(ULTRASONIC_DATA_4, &ULTRASONIC::ultrasonicDataParse));
  m_pHandlerMap.insert(make_pair(ULTRASONIC_CONTROL, &ULTRASONIC::ultrasonicDataParse));
  Ultrasonic_stat = Allstart_Ultrasonic;
  map< uint32_t, pFun >::iterator it;
  for (it = m_pHandlerMap.begin(); it != m_pHandlerMap.end(); ++it)
  {
    ROS_INFO("key %u", it->first);
  }
}

ULTRASONIC::~ULTRASONIC()
{
}

void ULTRASONIC::CANUltrasonic(int &ch_, int &dev_)
{
  ROS_INFO("readultrasonic thread %d %d %d", read_num[ch_], ch_, dev_);
  uint8_t RecvDataCount = 0;
  //  unique_lock<mutex> lock(taskData.mtx_camera_r);
  uint8_t ultrasonic_status = 0;
  uint16_t controltype      = 0;
  int sensor_num            = 0;
  string logname            = "ultrasonic";
  logdata_output can_log(logname, to_string(dev_), ch_);
  openCanRelay();
  // openCanRelay_();
  while (ros::ok())
  {

    ROS_INFO("channel:%d is alive~~~~~~~~~~!", ch_);
    if (adcuDevStatus(dev_) == ADCU_DEV_STATUS_ABNORMAL)
    {
      ROS_INFO("Can %d error", ch_);
      dev_ = adcuDevOpen(adcuCAN, CHANNEL_ULTRASONIC);
      // data_Output.Closefile(logname,logname);
      // break;
    }
    adcuCanData canbuf_;
    int length = 0;
    map< uint32_t, pFun >::iterator it;
    canOrder can_order_;
    switch (Ultrasonic_stat)
    {
    case Getstatus_Ultrasonic:
      ROS_INFO("Ultrasonic_stat:Getstatus!!!");
      if (writeCANOnce(dev_, ULTRASONIC_CONTROL, ULTRASONIC_ALLGETSTATUS, 8))
      {
        Ultrasonic_stat = Waitstatus_Ultrasonic;
      }
      break;
    case Waitstatus_Ultrasonic:
      ROS_INFO("Ultrasonic_stat:Waitstatus!!!");
      // length     = adcuDevRead(dev_, ( uint8_t * )&canbuf_);
      // if (length > 0)
      // {
      //   for (int i = 0; i < 8; i++)
      //   {
      //     printf("%02X ", canbuf_.can_data[i]);
      //   }
      //   printf("\n");

      //   map< uint32_t, pFun >::iterator it;
      //   canOrder can_order_;
      //   for (it = m_pHandlerMap.begin(); it != m_pHandlerMap.end(); ++it)
      //   {
      //     if (it->first == canbuf_.id)
      //     {
      //       (this->*(it->second))(canbuf_, canbuf_.id, can_order_, ch_);
      //       break;
      //     }
      //   }
      // }
      // if((camerastatus[CAMERA_OBJECT_1] + camerastatus[CAMERA_LANE_2] + camerastatus[CAMERA_OBJECT_5] +
      // camerastatus[CAMERA_OBJECT_6]) == CAN_DEVCOUNT_MAX)
      // {
      Ultrasonic_stat = Allstart_Ultrasonic;
    // }
    // else
    // {
    //     ROS_INFO("Waiting all sensor status ok!!!");
    // }
    case Allstart_Ultrasonic:
      ROS_INFO("Ultrasonic_stat:Allstart!!!");
      if (writeCANOnce(dev_, ULTRASONIC_CONTROL, ULTRASONIC_ALLSTARTWORK, 8))
      {
        Ultrasonic_stat = Work_Ultrasonic;
      }
      break;
    case Work_Ultrasonic:
      // ROS_INFO("Ultrasonic_stat:Work!!!");
      length = adcuDevRead(dev_, ( uint8_t * )&canbuf_);
      can_log.write_log(&canbuf_.can_data[0], canbuf_.id, 8);
      // canbuf_.id = ULTRASONIC_DATA_1;
      sensor_num = int(canbuf_.id);
      q_sensor_push(sensor_num);
      printf("recv data for Ultrasonic,canid:%d",canbuf_.id);
      if (length > 0)
      {
        // printf("read:");
        // for (int i = 0; i < 8; i++)
        // {
        //   printf("%02X ", canbuf_.can_data[i]);
        // }
        // printf("\n");
        if (canbuf_.id == ULTRASONIC_DATA_1 || canbuf_.id == ULTRASONIC_DATA_2 || canbuf_.id == ULTRASONIC_DATA_3 ||
            canbuf_.id == ULTRASONIC_DATA_4)
        {
          it = m_pHandlerMap.find(canbuf_.id);
          if (it != m_pHandlerMap.end())
          {
            (this->*(it->second))(canbuf_, canbuf_.id, can_order_, ch_, controltype);
          }
          ++ultrasonicData.recv_count;
          if (ultrasonicData.recv_count >= 4)
          {
            ultrasonicData.bReady = true;
            mtx_ultrasonic_data.lock();
            ultrasonicData_ready = ultrasonicData;
            mtx_ultrasonic_data.unlock();
            ultrasonicData.ultrasonicInfo_.clear();
            ultrasonicData.bReady = false;
            cond_ultrasonic.notify_one();
            ultrasonicData.recv_count = 0;
          }
        }
      }
      break;
    case Error_Ultrasonic:

      break;
    default:
      break;
    }
    //    usleep(100 * 1);
  }
  adcuDevClose(ch_);
  adcuSDKDeinit();
  ROS_INFO("readultrasonic thread is over!");
}

void ULTRASONIC::ultrasonic_Sender()
{
  ROS_INFO("thread ultrasonic_Sender Start!");
  unique_lock< mutex > lock(mtx_ultrasonic);
  ros::NodeHandle nh;
  // ros::Publisher sersor_pub = nh.advertise<std_msgs::UInt32MultiArray>("/drivers/can_wr/sonser_info", 1000, true);
  ros::Publisher ultrasonic_pub =
      nh.advertise< perception_sensor_msgs::UltrasonicInfo >("/drivers/can_wr/sonser_info", 1000, true);

  ros::Publisher ultrasonic_obstacle_pub =
      nh.advertise< perception_sensor_msgs::ObjectList >("/drivers/perception/ultrasonic_obstacle_info", 10, true);
  float measurement_cov[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  // Ultra2Car Ultra2Car_;
  while (ros::ok())
  {
    ROS_INFO("waiting ultrasonic data!");
    cond_ultrasonic.wait(lock);

    int i = 0;
    perception_sensor_msgs::ObjectList perceptionobstacleinfo;
    common_msgs::DetectionInfo obstacleinfo;
    perception_sensor_msgs::UltrasonicInfo ultrasonic_msg;
    common_msgs::UltrasonicPoint ultrasonicInfoStr_;
    mtx_ultrasonic_data.lock();
    perceptionobstacleinfo.header.stamp            = ros::Time::now();
    perceptionobstacleinfo.header.frame_id         = "base_link";
    perceptionobstacleinfo.sensor_index            = 2;
    perceptionobstacleinfo.obstacle_num            = ultrasonicData_ready.ultrasonicInfo_.size();

    ultrasonic_msg.header.stamp    = ros::Time::now();
    ultrasonic_msg.header.frame_id = "base_link";
    ultrasonic_msg.obstacle_num    = ultrasonicData_ready.ultrasonicInfo_.size();

    for (list< ultrasonicInfoStr >::iterator ite_ultrasonicInfo = ultrasonicData_ready.ultrasonicInfo_.begin();
         ite_ultrasonicInfo != ultrasonicData_ready.ultrasonicInfo_.end(); ++ite_ultrasonicInfo)
    {
      obstacleinfo.id         = ite_ultrasonicInfo->id;
      obstacleinfo.obj_class   = 9;
      obstacleinfo.confidence  = 1;
      obstacleinfo.state[0] = 0;
      obstacleinfo.state[1] = 0;
      obstacleinfo.state[2] = 0;
      obstacleinfo.state[3] = 0;
      for (int j = 0; j < 16; ++j)
      {
        obstacleinfo.measurement_cov[j] = measurement_cov[j];
      }
      Ultra2Car_m(ite_ultrasonicInfo->distance,ite_ultrasonicInfo->id);
      obstacleinfo.peek[0].x = 
              max(get_xy(0).x,get_xy(1).x);
      obstacleinfo.peek[0].y =
              max(get_xy(0).y,get_xy(1).y);
      obstacleinfo.peek[1].x =
              max(get_xy(0).x,get_xy(1).x);
      obstacleinfo.peek[1].y =
              min(get_xy(0).y,get_xy(1).y);
      obstacleinfo.peek[2].x =
              min(get_xy(0).x,get_xy(1).x);
      obstacleinfo.peek[2].y =
              min(get_xy(0).y,get_xy(1).y);
      obstacleinfo.peek[3].x =
              min(get_xy(0).x,get_xy(1).x);
      obstacleinfo.peek[3].y =
              max(get_xy(0).y,get_xy(1).y);

      ++i;
      perceptionobstacleinfo.object_list.push_back(obstacleinfo);

      ultrasonicInfoStr_.id       = ite_ultrasonicInfo->id;
      ultrasonicInfoStr_.distance = ite_ultrasonicInfo->distance;
      ultrasonicInfoStr_.status   = ite_ultrasonicInfo->status;
      ultrasonic_msg.ult_obstacle.push_back(ultrasonicInfoStr_);

    }
    ultrasonicData_ready.ultrasonicInfo_.clear();
    ultrasonicData_ready.bReady = false;
    mtx_ultrasonic_data.unlock();
    // ROS_INFO("Publish sersor_data once!!!");
    ultrasonic_obstacle_pub.publish(perceptionobstacleinfo);

    ultrasonic_pub.publish(ultrasonic_msg);

    // perception_sensor_msgs::UltrasonicInfo ultrasonic_msg;
    // common_msgs::UltrasonicPoint ultrasonicInfoStr_;

    // mtx_ultrasonic_data.lock();
    // ultrasonic_msg.header.stamp    = ros::Time::now();
    // ultrasonic_msg.header.frame_id = "base_link";
    // ultrasonic_msg.obstacle_num    = ultrasonicData_ready.ultrasonicInfo_.size();
    // // ROS_INFO_STREAM("size():" << ultrasonic_msg.obstacle_num);
    // for (list< ultrasonicInfoStr >::iterator ite_ultrasonicInfo = ultrasonicData_ready.ultrasonicInfo_.begin();
    //      ite_ultrasonicInfo != ultrasonicData_ready.ultrasonicInfo_.end(); ++ite_ultrasonicInfo)
    // {
      // ultrasonicInfoStr_.id       = ite_ultrasonicInfo->id;
      // ultrasonicInfoStr_.distance = ite_ultrasonicInfo->distance;
      // ultrasonicInfoStr_.status   = ite_ultrasonicInfo->status;
      // ultrasonic_msg.ult_obstacle.push_back(ultrasonicInfoStr_);
    // }
    // ultrasonicData_ready.ultrasonicInfo_.clear();

    // mtx_ultrasonic_data.unlock();
    // ROS_INFO("Publish sersor_data once!!!");
    // ultrasonic_pub.publish(ultrasonic_msg);
  }
}

void ULTRASONIC::ultrasonicDataParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_,
                                     uint16_t &contype)
{
  if (can_order.reserve != 1)
  {
    // ROS_INFO("recv: %d ultrasonicDataParse", can_id);
  }
  ultrasonicData.ultrasonic1_distance = (can_buf.can_data[0] & 0x3F) * 0.05;
  ultrasonicData.ultrasonic2_distance = ((can_buf.can_data[1] << 2 & 0x3C) | (can_buf.can_data[0] >> 6 & 0x03)) * 0.05;
  ultrasonicData.ultrasonic3_distance = ((can_buf.can_data[2] << 4 & 0x30) | (can_buf.can_data[1] >> 4 & 0x0F)) * 0.05;
  ultrasonicData.ultrasonic4_distance = (can_buf.can_data[2] >> 2 & 0x3F) * 0.05;
  ultrasonicData.ultrasonic5_distance = (can_buf.can_data[3] & 0x3F) * 0.05;
  ultrasonicData.ultrasonic6_distance = ((can_buf.can_data[4] << 2 & 0x3C) | (can_buf.can_data[3] >> 6 & 0x03)) * 0.05;
  ultrasonicData.ultrasonic7_distance = ((can_buf.can_data[5] << 4 & 0x30) | (can_buf.can_data[4] >> 4 & 0x0F)) * 0.05;
  ultrasonicData.ultrasonic8_distance = (can_buf.can_data[5] >> 2 & 0x3F) * 0.05;

  ultrasonicData.ultrasonic1_status = can_buf.can_data[6] & 0x01;
  ultrasonicData.ultrasonic2_status = can_buf.can_data[6] & 0x02 >> 1;
  ultrasonicData.ultrasonic3_status = can_buf.can_data[6] & 0x04 >> 2;
  ultrasonicData.ultrasonic4_status = can_buf.can_data[6] & 0x08 >> 3;
  ultrasonicData.ultrasonic5_status = can_buf.can_data[6] & 0x10 >> 4;
  ultrasonicData.ultrasonic6_status = can_buf.can_data[6] & 0x20 >> 5;
  ultrasonicData.ultrasonic7_status = can_buf.can_data[6] & 0x40 >> 6;
  ultrasonicData.ultrasonic8_status = can_buf.can_data[6] & 0x80 >> 7;

  ultrasonicData.controller_id = can_buf.can_data[7] & 0x03;
  ultrasonicInfoStr ultrasonicInfo;
  if (ultrasonicData.ultrasonic1_distance <= ULTRASONIC_SAFEDISTANCE && ultrasonicData.ultrasonic1_distance >= ULTRASONIC_MINDISTANCE)
  {
    ultrasonicInfo.id       = toCar_num(uint32_t(1 + ultrasonicData.controller_id * 8));
    ultrasonicInfo.distance = ultrasonicData.ultrasonic1_distance;
    ultrasonicInfo.status   = ultrasonicData.ultrasonic1_status;
    ultrasonicData.ultrasonicInfo_.push_back(ultrasonicInfo);
  }
  if (ultrasonicData.ultrasonic2_distance <= ULTRASONIC_SAFEDISTANCE && ultrasonicData.ultrasonic2_distance >= ULTRASONIC_MINDISTANCE)
  {
    ultrasonicInfo.id       = toCar_num(uint32_t(2 + ultrasonicData.controller_id * 8));
    ultrasonicInfo.distance = ultrasonicData.ultrasonic2_distance;
    ultrasonicInfo.status   = ultrasonicData.ultrasonic2_status;
    ultrasonicData.ultrasonicInfo_.push_back(ultrasonicInfo);
  }
  if (ultrasonicData.ultrasonic3_distance <= ULTRASONIC_SAFEDISTANCE && ultrasonicData.ultrasonic3_distance >= ULTRASONIC_MINDISTANCE)
  {
    ultrasonicInfo.id       = toCar_num(uint32_t(3 + ultrasonicData.controller_id * 8));
    ultrasonicInfo.distance = ultrasonicData.ultrasonic3_distance;
    ultrasonicInfo.status   = ultrasonicData.ultrasonic3_status;
    ultrasonicData.ultrasonicInfo_.push_back(ultrasonicInfo);
  }
  if (ultrasonicData.ultrasonic4_distance <= ULTRASONIC_SAFEDISTANCE && ultrasonicData.ultrasonic4_distance >= ULTRASONIC_MINDISTANCE)
  {
    ultrasonicInfo.id       = toCar_num(uint32_t(4 + ultrasonicData.controller_id * 8));
    ultrasonicInfo.distance = ultrasonicData.ultrasonic4_distance;
    ultrasonicInfo.status   = ultrasonicData.ultrasonic4_status;
    ultrasonicData.ultrasonicInfo_.push_back(ultrasonicInfo);
  }
  if (ultrasonicData.ultrasonic5_distance <= ULTRASONIC_SAFEDISTANCE && ultrasonicData.ultrasonic5_distance >= ULTRASONIC_MINDISTANCE)
  {
    ultrasonicInfo.id       = toCar_num(uint32_t(5 + ultrasonicData.controller_id * 8));
    ultrasonicInfo.distance = ultrasonicData.ultrasonic5_distance;
    ultrasonicInfo.status   = ultrasonicData.ultrasonic5_status;
    ultrasonicData.ultrasonicInfo_.push_back(ultrasonicInfo);
  }
  if (ultrasonicData.ultrasonic6_distance <= ULTRASONIC_SAFEDISTANCE && ultrasonicData.ultrasonic6_distance >= ULTRASONIC_MINDISTANCE)
  {
    ultrasonicInfo.id       = toCar_num(uint32_t(6 + ultrasonicData.controller_id * 8));
    ultrasonicInfo.distance = ultrasonicData.ultrasonic6_distance;
    ultrasonicInfo.status   = ultrasonicData.ultrasonic6_status;
    ultrasonicData.ultrasonicInfo_.push_back(ultrasonicInfo);
  }
  if (ultrasonicData.ultrasonic7_distance <= ULTRASONIC_SAFEDISTANCE && ultrasonicData.ultrasonic7_distance >= ULTRASONIC_MINDISTANCE)
  {
    ultrasonicInfo.id       = toCar_num(uint32_t(7 + ultrasonicData.controller_id * 8));
    ultrasonicInfo.distance = ultrasonicData.ultrasonic7_distance;
    ultrasonicInfo.status   = ultrasonicData.ultrasonic7_status;
    ultrasonicData.ultrasonicInfo_.push_back(ultrasonicInfo);
  }
  if (ultrasonicData.ultrasonic8_distance <= ULTRASONIC_SAFEDISTANCE && ultrasonicData.ultrasonic8_distance >= ULTRASONIC_MINDISTANCE)
  {
    ultrasonicInfo.id       = toCar_num(uint32_t(8 + ultrasonicData.controller_id * 8));
    ultrasonicInfo.distance = ultrasonicData.ultrasonic8_distance;
    ultrasonicInfo.status   = ultrasonicData.ultrasonic8_status;
    ultrasonicData.ultrasonicInfo_.push_back(ultrasonicInfo);
  }
  if (can_order.reserve != 1)
  {
    // ROS_INFO(
    //     "controller_id: %d, sensor1_dist: %d, sensor2_dist: %d sensor3_dist: %d sensor4_dist: %d sensor5_dist: "
    //     "%d,sensor6_dist: %d sensor7_dist: %d sensor8_dist: %d,sensor1_status: %d,sensor2_status: %d,sensor3_status:
    //     "
    //     "%d,sensor4_status: %d,sensor5_status: %d,sensor6_status: %d,sensor7_status: %d,sensor8_status: %d,",
    //     ultrasonicData.controller_id, ultrasonicData.ultrasonic1_distance, ultrasonicData.ultrasonic2_distance,
    //     ultrasonicData.ultrasonic3_distance, ultrasonicData.ultrasonic4_distance,
    //     ultrasonicData.ultrasonic5_distance,
    //     ultrasonicData.ultrasonic6_distance, ultrasonicData.ultrasonic7_distance,
    //     ultrasonicData.ultrasonic8_distance,
    //     ultrasonicData.ultrasonic1_status, ultrasonicData.ultrasonic2_status, ultrasonicData.ultrasonic3_status,
    //     ultrasonicData.ultrasonic4_status, ultrasonicData.ultrasonic5_status, ultrasonicData.ultrasonic6_status,
    //     ultrasonicData.ultrasonic7_status, ultrasonicData.ultrasonic8_status);

    if (ultrasonicData.ultrasonic1_distance < 0.45)
    {
      ROS_ERROR("controller_id: %u : sensor1_dist: %f -- sensor1_status: %d", ultrasonicData.controller_id,
                ultrasonicData.ultrasonic1_distance, ultrasonicData.ultrasonic1_status);
    }
    else
    {
      // ROS_INFO("controller_id: %d : sensor1_dist: %2d -- sensor1_status: %2d", ultrasonicData.controller_id,
      //          ultrasonicData.ultrasonic1_distance, ultrasonicData.ultrasonic1_status);
    }
    // sensor 2
    if (ultrasonicData.ultrasonic2_distance < 0.45)
    {
      ROS_ERROR("controller_id: %u : sensor2_dist: %f -- sensor2_status: %d", ultrasonicData.controller_id,
                ultrasonicData.ultrasonic2_distance, ultrasonicData.ultrasonic2_status);
    }
    else
    {
      // ROS_INFO("controller_id: %d : sensor2_dist: %2d -- sensor2_status: %2d", ultrasonicData.controller_id,
      //          ultrasonicData.ultrasonic2_distance, ultrasonicData.ultrasonic2_status);
    }
    // sensor 3
    if (ultrasonicData.ultrasonic3_distance < 0.45)
    {
      ROS_ERROR("controller_id: %u : sensor3_dist: %f -- sensor3_status: %d", ultrasonicData.controller_id,
                ultrasonicData.ultrasonic3_distance, ultrasonicData.ultrasonic3_status);
    }
    else
    {
      // ROS_INFO("controller_id: %d : sensor3_dist: %2d -- sensor3_status: %2d", ultrasonicData.controller_id,
      //          ultrasonicData.ultrasonic3_distance, ultrasonicData.ultrasonic3_status);
    }
    // sensor 4
    if (ultrasonicData.ultrasonic4_distance < 0.45)
    {
      ROS_ERROR("controller_id: %u : sensor4_dist: %f -- sensor4_status: %d", ultrasonicData.controller_id,
                ultrasonicData.ultrasonic4_distance, ultrasonicData.ultrasonic4_status);
    }
    else
    {
      // ROS_INFO("controller_id: %d : sensor4_dist: %2d -- sensor4_status: %2d", ultrasonicData.controller_id,
      //          ultrasonicData.ultrasonic4_distance, ultrasonicData.ultrasonic4_status);
    }
    // 5
    if (ultrasonicData.ultrasonic5_distance < 0.45)
    {
      ROS_ERROR("controller_id: %u : sensor5_dist: %f -- sensor5_status: %d", ultrasonicData.controller_id,
                ultrasonicData.ultrasonic5_distance, ultrasonicData.ultrasonic5_status);
    }
    else
    {
      // ROS_INFO("controller_id: %d : sensor5_dist: %2d -- sensor5_status: %2d", ultrasonicData.controller_id,
      //          ultrasonicData.ultrasonic5_distance, ultrasonicData.ultrasonic5_status);
    }
    // 6
    if (ultrasonicData.ultrasonic6_distance < 0.45)
    {
      ROS_ERROR("controller_id: %u : sensor6_dist: %f -- sensor6_status: %d", ultrasonicData.controller_id,
                ultrasonicData.ultrasonic6_distance, ultrasonicData.ultrasonic6_status);
    }
    else
    {
      // ROS_INFO("controller_id: %d : sensor6_dist: %2d -- sensor6_status: %2d", ultrasonicData.controller_id,
      //          ultrasonicData.ultrasonic6_distance, ultrasonicData.ultrasonic6_status);
    }
    // 7
    if (ultrasonicData.ultrasonic7_distance < 0.45)
    {
      ROS_ERROR("controller_id: %u : sensor7_dist: %f -- sensor7_status: %d", ultrasonicData.controller_id,
                ultrasonicData.ultrasonic7_distance, ultrasonicData.ultrasonic7_status);
    }
    else
    {
      // ROS_INFO("controller_id: %d : sensor7_dist: %2d -- sensor7_status: %2d", ultrasonicData.controller_id,
      //          ultrasonicData.ultrasonic7_distance, ultrasonicData.ultrasonic7_status);
    }
    // 8
    if (ultrasonicData.ultrasonic8_distance < 0.45)
    {
      ROS_ERROR("controller_id: %u : sensor8_dist: %f -- sensor8_status: %d", ultrasonicData.controller_id,
                ultrasonicData.ultrasonic8_distance, ultrasonicData.ultrasonic8_status);
    }
    else
    {
      // ROS_INFO("controller_id: %d : sensor8_dist: %2d -- sensor8_status: %2d", ultrasonicData.controller_id,
      //          ultrasonicData.ultrasonic8_distance, ultrasonicData.ultrasonic8_status);
    }

    // sensor 1
    if (ultrasonicData.ultrasonic1_status == 1)
    {
      ROS_ERROR("controller_id: %u : sensor1_dist: %f -- sensor1_status: %2d", ultrasonicData.controller_id,
                ultrasonicData.ultrasonic1_distance, ultrasonicData.ultrasonic1_status);
    }
    else
    {
      // ROS_INFO("controller_id: %d : sensor1_dist: %2d -- sensor1_status: %2d", ultrasonicData.controller_id,
      //          ultrasonicData.ultrasonic1_distance, ultrasonicData.ultrasonic1_status);
    }
    // sensor 2
    if (ultrasonicData.ultrasonic2_status == 1)
    {
      ROS_ERROR("controller_id: %u : sensor2_dist: %f -- sensor2_status: %2d", ultrasonicData.controller_id,
                ultrasonicData.ultrasonic2_distance, ultrasonicData.ultrasonic2_status);
    }
    else
    {
      // ROS_INFO("controller_id: %d : sensor2_dist: %2d -- sensor2_status: %2d", ultrasonicData.controller_id,
      //          ultrasonicData.ultrasonic2_distance, ultrasonicData.ultrasonic2_status);
    }
    // sensor 3
    if (ultrasonicData.ultrasonic3_status == 1)
    {
      ROS_ERROR("controller_id: %u : sensor3_dist: %f -- sensor3_status: %2d", ultrasonicData.controller_id,
                ultrasonicData.ultrasonic3_distance, ultrasonicData.ultrasonic3_status);
    }
    else
    {
      // ROS_INFO("controller_id: %d : sensor3_dist: %2d -- sensor3_status: %2d", ultrasonicData.controller_id,
      //          ultrasonicData.ultrasonic3_distance, ultrasonicData.ultrasonic3_status);
    }
    // sensor 4
    if (ultrasonicData.ultrasonic4_status == 1)
    {
      ROS_ERROR("controller_id: %u : sensor4_dist: %f -- sensor4_status: %2d", ultrasonicData.controller_id,
                ultrasonicData.ultrasonic4_distance, ultrasonicData.ultrasonic4_status);
    }
    else
    {
      // ROS_INFO("controller_id: %d : sensor4_dist: %2d -- sensor4_status: %2d", ultrasonicData.controller_id,
      //          ultrasonicData.ultrasonic4_distance, ultrasonicData.ultrasonic4_status);
    }
    // 5
    if (ultrasonicData.ultrasonic5_status == 1)
    {
      ROS_ERROR("controller_id: %u : sensor5_dist: %f -- sensor5_status: %2d", ultrasonicData.controller_id,
                ultrasonicData.ultrasonic5_distance, ultrasonicData.ultrasonic5_status);
    }
    else
    {
      // ROS_INFO("controller_id: %d : sensor5_dist: %2d -- sensor5_status: %2d", ultrasonicData.controller_id,
      //          ultrasonicData.ultrasonic5_distance, ultrasonicData.ultrasonic5_status);
    }
    // 6
    if (ultrasonicData.ultrasonic6_status == 1)
    {
      ROS_ERROR("controller_id: %u : sensor6_dist: %f -- sensor6_status: %2d", ultrasonicData.controller_id,
                ultrasonicData.ultrasonic6_distance, ultrasonicData.ultrasonic6_status);
    }
    else
    {
      // ROS_INFO("controller_id: %d : sensor6_dist: %2d -- sensor6_status: %2d", ultrasonicData.controller_id,
      //          ultrasonicData.ultrasonic6_distance, ultrasonicData.ultrasonic6_status);
    }
    // 7
    if (ultrasonicData.ultrasonic7_status == 1)
    {
      ROS_ERROR("controller_id: %u : sensor7_dist: %f -- sensor7_status: %2d", ultrasonicData.controller_id,
                ultrasonicData.ultrasonic7_distance, ultrasonicData.ultrasonic7_status);
    }
    else
    {
      // ROS_INFO("controller_id: %d : sensor7_dist: %2d -- sensor7_status: %2d", ultrasonicData.controller_id,
      //          ultrasonicData.ultrasonic7_distance, ultrasonicData.ultrasonic7_status);
    }
    // 8
    if (ultrasonicData.ultrasonic8_status == 1)
    {
      ROS_ERROR("controller_id: %u : sensor8_dist: %f -- sensor8_status: %2d", ultrasonicData.controller_id,
                ultrasonicData.ultrasonic8_distance, ultrasonicData.ultrasonic8_status);
    }
    else
    {
      // ROS_INFO("controller_id: %d : sensor8_dist: %2d -- sensor8_status: %2d", ultrasonicData.controller_id,
      //          ultrasonicData.ultrasonic8_distance, ultrasonicData.ultrasonic8_status);
    }
  }
}

void ULTRASONIC::openCanRelay()
{
  ros::NodeHandle nh_1;
  ros::Publisher pub_canrelay = nh_1.advertise<power_control_msgs::PowerControlCmd>("/power_control/power_control_cmd", 1000, true);
  power_control_msgs::PowerControlCmd PowerControlCmd_msg;
  PowerControlCmd_msg.header.stamp            = ros::Time::now();
  PowerControlCmd_msg.header.frame_id         = "base_link";
  PowerControlCmd_msg.length                  = 20;
//  while(ros::ok())
//  {
  sleep(1);
  for(int i = 0;i < 20;++i)
  {
    if(17 == i)
    {
      PowerControlCmd_msg.data.push_back(1);
      ROS_INFO("1");
    }
    else
    {
      PowerControlCmd_msg.data.push_back(2);
      ROS_INFO("2");
    }
  }
  pub_canrelay.publish(PowerControlCmd_msg);
  sleep(1);
//  }
}

void ULTRASONIC::openCanRelay_()
{
  ros::NodeHandle nh_1;
  ros::Publisher pub_canrelay = nh_1.advertise<power_control_msgs::PowerControlCmd>("/power_control/power_control_cmd", 1000, true);
  power_control_msgs::PowerControlCmd PowerControlCmd_msg;
  PowerControlCmd_msg.header.stamp            = ros::Time::now();
  PowerControlCmd_msg.header.frame_id         = "base_link";
  PowerControlCmd_msg.length                  = 20;
//  while(ros::ok())
//  {
  sleep(1);
  for(int i = 0;i < 20;++i)
  {
    if(15 == i)
    {
      PowerControlCmd_msg.data.push_back(1);
      ROS_INFO("1");
    }
    else
    {
      PowerControlCmd_msg.data.push_back(2);
      ROS_INFO("2");
    }
  }
  pub_canrelay.publish(PowerControlCmd_msg);
  sleep(1);
//  }
}

void ULTRASONIC::openCanRelay_0()
{
  ros::NodeHandle nh_1;
  ros::Publisher pub_canrelay = nh_1.advertise<power_control_msgs::PowerControlCmd>("/power_control/power_control_cmd", 1000, true);
  power_control_msgs::PowerControlCmd PowerControlCmd_msg;
  PowerControlCmd_msg.header.stamp            = ros::Time::now();
  PowerControlCmd_msg.header.frame_id         = "base_link";
  PowerControlCmd_msg.length                  = 20;
//  while(ros::ok())
//  {
  sleep(1);
  for(int i = 0;i < 20;++i)
  {
    if(15 == i)
    {
      PowerControlCmd_msg.data.push_back(0);
      ROS_INFO("1");
    }
    else
    {
      PowerControlCmd_msg.data.push_back(2);
      ROS_INFO("2");
    }
  }
  pub_canrelay.publish(PowerControlCmd_msg);
  sleep(1);
//  }
}

void ULTRASONIC::ultrasonicControl(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_,
                                   uint16_t &contype)
{
  ROS_INFO("send ultrasonicControl!");
}

Ultra2Car::Ultra2Car()
{
  // front
  for (uint8_t i = 0; i < 32; ++i)
  {
    if (i < 4)
    {
      transformation_info[i].rotate = 0;
      transformation_info[i].radian = angle_to_radian(transformation_info[i].rotate);
      // transformation_info[i].dx     = 2.712 - (i * 1.808);
      switch(i)
      {
        case 0:
          transformation_info[i].dx     = -0.7412;
          break;
        case 1:
          transformation_info[i].dx     = -0.2712;
        case 2:
          transformation_info[i].dx     = 0.2712;
          break;
        case 3:
          transformation_info[i].dx     = 0.7412;
        default:
          break;
      }
      // transformation_info[i].dy     = 7.735;
      transformation_info[i].dy     = 7.735;
    }
    else if (i < 16)
    {
      transformation_info[i].rotate =-90;
      transformation_info[i].radian = angle_to_radian(transformation_info[i].rotate);
      transformation_info[i].dx     = 1.087;
      transformation_info[i].dy     = 3.3804 - ((i - 4) * 0.6146);
    }
    else if (i < 20)
    {
      transformation_info[i].rotate = 180;
      transformation_info[i].radian = angle_to_radian(transformation_info[i].rotate);
      // transformation_info[i].dx     = -2.712 + ((i - 16) * 1.808);
      switch(i)
      {
        case 16:
          transformation_info[i].dx     = 0.7412;
          break;
        case 17:
          transformation_info[i].dx     = 0.2712;
        case 18:
          transformation_info[i].dx     = -0.2712;
          break;
        case 19:
          transformation_info[i].dx     = -0.7412;
        default:
          break;
      }
      transformation_info[i].dy     = -7.735;
    }
    else
    {
      transformation_info[i].rotate = 90;
      transformation_info[i].radian = angle_to_radian(transformation_info[i].rotate);
      transformation_info[i].dx     = -1.087;
      transformation_info[i].dy     = -3.380 + ((i - 20) * 0.6146);
    }
    // rotate_m << cos(transformation_info[i].radian), sin(transformation_info[i].radian), 0,
    //     -sin(transformation_info[i].radian), cos(transformation_info[i].radian), 0, 0, 0, 1;
    rotate_m << cos(transformation_info[i].radian), -sin(transformation_info[i].radian), 0,
        sin(transformation_info[i].radian), cos(transformation_info[i].radian), 0, 0, 0, 1;
    pan_m << 1, 0, 0, 0, 1, 0, transformation_info[i].dy, transformation_info[i].dx, 1;
    tarns_m[i] = rotate_m * pan_m;
    // cout << "i:" << i << endl;
    // cout << rotate_m << endl;
    // cout << endl;
    // cout << pan_m << endl;
    // cout << endl;
    // cout << tarns_m[i] << endl;
    // cout << endl;
  }
}

Ultra2Car::~Ultra2Car()
{
  ;
}

uint32_t Ultra2Car::toCar_num(uint32_t num)
{
  uint32_t car_num = 0;
  switch(num)
  {
    // case 1:
    //   car_num = 31;
    //   break;
    // case 2:
    //   car_num = 30;
    //   break;
    // case 3:
    //   car_num = 29;
    //   break;
    // case 4:
    //   car_num = 28;
    //   break;
    // case 5:
    //   car_num = 24;
    //   break;
    // case 6:
    //   car_num = 25;
    //   break;
    // case 7:
    //   car_num = 26;
    //   break;
    // case 8:
    //   car_num = 27;
    //   break;
    // case 9:
    //   car_num = 23;
    //   break;
    // case 10:
    //   car_num = 22;
    //   break;
    // case 11:
    //   car_num = 21;
    //   break;
    // case 12:
    //   car_num = 20;
    //   break;
    // case 13:
    //   car_num = 19;
    //   break;
    // case 14:
    //   car_num = 18;
    //   break;
    // case 15:
    //   car_num = 17;
    //   break;
    // case 16:
    //   car_num = 16;
    //   break;
    // case 17:
    //   car_num = 12;
    //   break;
    // case 18:
    //   car_num = 13;
    //   break;
    // case 19:
    //   car_num = 14;
    //   break;
    // case 20:
    //   car_num = 15;
    //   break;
    // case 21:
    //   car_num = 8;
    //   break;
    // case 22:
    //   car_num = 9;
    //   break;
    // case 23:
    //   car_num = 10;
    //   break;
    // case 24:
    //   car_num = 11;
    //   break;
    // case 25:
    //   car_num = 7;
    //   break;
    // case 26:
    //   car_num = 6;
    //   break;
    // case 27:
    //   car_num = 5;
    //   break;
    // case 28:
    //   car_num = 4;
    //   break;
    // case 29:
    //   car_num = 3;
    //   break;
    // case 30:
    //   car_num = 2;
    //   break;
    // case 31:
    //   car_num = 1;
    //   break;
    // case 32:
    //   car_num = 0;
    //   break;
    // default:
    //   break;
    case 1:
      car_num = 31;
      break;
    case 2:
      car_num = 30;
      break;
    case 3:
      car_num = 29;
      break;
    case 4:
      car_num = 28;
      break;
    case 5:
      car_num = 27;
      break;
    case 6:
      car_num = 26;
      break;
    case 7:
      car_num = 25;
      break;
    case 8:
      car_num = 24;
      break;
    case 9:
      car_num = 20;
      break;
    case 10:
      car_num = 21;
      break;
    case 11:
      car_num = 22;
      break;
    case 12:
      car_num = 23;
      break;
    case 13:
      car_num = 19;
      break;
    case 14:
      car_num = 18;
      break;
    case 15:
      car_num = 17;
      break;
    case 16:
      car_num = 16;
      break;
    case 17:
      car_num = 15;
      break;
    case 18:
      car_num = 14;
      break;
    case 19:
      car_num = 13;
      break;
    case 20:
      car_num = 12;
      break;
    case 21:
      car_num = 11;
      break;
    case 22:
      car_num = 10;
      break;
    case 23:
      car_num = 9;
      break;
    case 24:
      car_num = 8;
      break;
    case 25:
      car_num = 4;
      break;
    case 26:
      car_num = 5;
      break;
    case 27:
      car_num = 6;
      break;
    case 28:
      car_num = 7;
      break;
    case 29:
      car_num = 3;
      break;
    case 30:
      car_num = 2;
      break;
    case 31:
      car_num = 1;
      break;
    case 32:
      car_num = 0;
      break;
    default:
      break;
  }
  return car_num;
}

double Ultra2Car::angle_to_radian(double degree)
{
  double flag = (degree < 0) ? -1.0 : 1.0; //判断正负
  if (degree < 0)
  {
    degree = degree * (-1.0);
  }
  double angle  = degree;
  double result = flag * (angle * PI) / 180;
  return result;
}

XY Ultra2Car::get_xy(uint8_t index)
{
  return xy[index];
}

void Ultra2Car::Ultra2Car_m(float distance, uint8_t num)
{
  // XY xy_1{0.0,0.0};
  ultra_m << distance + D_DISTANCE, -(distance + D_DISTANCE) * sin(angle_to_radian(60)), 1;
  // cout << "ultra:" << distance + D_DISTANCE << "," << -(distance + D_DISTANCE) * sin(angle_to_radian(60)) << endl;
  car_m   = ultra_m * tarns_m[num];
  xy[0].y = car_m(0, 0);
  xy[0].x = car_m(0, 1);

  // ultra_m << distance * cos(angle_to_radian(60) , (distance + D_DISTANCE) * sin(angle_to_radian(60)) , 1;
  ultra_m(0, 0) = distance * cos(angle_to_radian(60));
  ultra_m(0, 1) = (distance + D_DISTANCE) * sin(angle_to_radian(60));
  ultra_m(0, 2) = 1;
  // cout << "ultra:" << distance * cos(angle_to_radian(60)) << "," << (distance + D_DISTANCE) * sin(angle_to_radian(60))
      //  << endl;
  car_m   = ultra_m * tarns_m[num];
  xy[1].y = car_m(0, 0);
  xy[1].x = car_m(0, 1);

  // cout << "x1:" << xy[0].x << ",y1:" << xy[0].y << ",x2:" << xy[1].x << ",y2:" << xy[1].y << endl;
}
}
}
