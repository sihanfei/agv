#include "p2.h"

namespace superg_agv
{
namespace drivers
{

P2::P2()
{
  rate = 100;
  m_pHandlerMap.insert(make_pair(P2_TIME, &P2::p2TimeParse));
  m_pHandlerMap.insert(make_pair(P2_ANG_RATE_RAW_IMU, &P2::p2AngRateRawIMUParse));
  m_pHandlerMap.insert(make_pair(P2_ACCEL_IMU_RAW, &P2::p2AccelIMURawParse));
  m_pHandlerMap.insert(make_pair(P2_INS_STATUS, &P2::p2InsStatusParse));
  m_pHandlerMap.insert(make_pair(P2_LATITUDE_LONGITUDE, &P2::p2LatitudeLongitudeParse));
  m_pHandlerMap.insert(make_pair(P2_ALTITUDE, &P2::p2AltitudeParse));
  m_pHandlerMap.insert(make_pair(P2_POS_SIGMA, &P2::p2PosSigmaParse));
  m_pHandlerMap.insert(make_pair(P2_VELOCITY_LEVEL, &P2::p2VelocityLevelParse));
  m_pHandlerMap.insert(make_pair(P2_VELOCITY_LEVEL_SIGMA, &P2::p2VelocityLevelSigmaParse));
  m_pHandlerMap.insert(make_pair(P2_ACCEL_VEHICLE, &P2::p2AccelVehicleParse));
  m_pHandlerMap.insert(make_pair(P2_HEADING_PITCH_ROLL, &P2::p2HeadingPitchRollParse));
  m_pHandlerMap.insert(make_pair(P2_HEADING_PITCH_ROLL_SIGMA, &P2::p2HeadingPitchRollSigmaParse));
  m_pHandlerMap.insert(make_pair(P2_ANG_RATE_VEHICLE, &P2::p2AngRateVehicleParse));
  map< uint32_t, pFun >::iterator it; 
  for (it = m_pHandlerMap.begin(); it != m_pHandlerMap.end(); ++it)
  {
    ROS_INFO("key %u", it->first);
  }
//   CANDrivers::
}

P2::~P2()
{

}

void P2::CANP2(int &ch_, int &dev_)
{
  ROS_INFO("readP2 thread %d %d %d", read_num[ch_], ch_, dev_);
/*获取GPS所需的变量*/
  int atFd, nmeaFd,nset1, nwrite, nread;
  char at[20], nmea[1024];
  vector<string> v;
/*获取GPS所需的变量*/
  struct timeval tv;
  time_t now;
  struct tm *timenow;
  uint16_t controltype = 0;
  uint8_t sensor_rate = 100;
  devid = dev_;
  Safety_Status safety_status;
  int sensor_num = P2_TIME;
  string logname = "p2";
  // logdata_output can_log("/home/hx/work/log",logname,to_string(dev_),ch_);
  logdata_output can_log(logname,to_string(devid),ch_);
  set_rate(sensor_rate);
  // CANData_Output data_Output(logname);
  // while(ros::ok())
  // {
  //   q_sensor_push(logname);
  //   // ROS_INFO("send queue p2!");
  //   usleep(1000 * 10);
  // }
  while (ros::ok())
  {
    if (adcuDevStatus(devid) == ADCU_DEV_STATUS_ABNORMAL)
    {
      ROS_INFO("Can %d error", ch_);
      // devid = adcuDevOpen(adcuCAN,CHANNEL_P2);
      break;
    }
    else
    {
      adcuCanData canbuf_;
      int length = 0;
      length     = adcuDevRead(devid, ( uint8_t * )&canbuf_);

      can_log.write_log(&canbuf_.can_data[0],canbuf_.id,8);
      // printf("write log\n");
      // canbuf_.id = P2_TIME;
      // usleep(1000 * 10);
      if(canbuf_.id == P2_TIME)
      {
        q_sensor_push(sensor_num);
      }
      if (length > 0)
      {
  //       ROS_INFO("CAN:%d recv:%u", ch_, canbuf_.id);
  // //      printf("Data: ");
//	 if(809 == canbuf_.id)
//	 {
        // for (int i = 0; i < 8; i++)
        // {
        //   printf("%02X ", canbuf_.can_data[i]);
        // }
        // printf("\n");
//	}

        map< uint32_t, pFun >::iterator it;
        canOrder can_order_;
        it = m_pHandlerMap.find(canbuf_.id);
        if (it != m_pHandlerMap.end())
        {
          (this->*(it->second))(canbuf_, canbuf_.id, can_order_, ch_, controltype);
          if(2 == ch_ && it->first == P2_TIME)
          {
              //get timedate
            time(&now);
            timenow = localtime(&now);
            gettimeofday(&tv,NULL);
            timeDate.year = uint32_t(timenow->tm_year + 1900);
            timeDate.month = uint8_t(timenow->tm_mon + 1);
            timeDate.day = uint8_t(timenow->tm_mday);
            timeDate.hour = uint8_t(timenow->tm_hour);
            timeDate.min = uint8_t(timenow->tm_min);
            timeDate.sec = uint8_t(timenow->tm_sec);
            timeDate.msec = float(tv.tv_usec/1000);
            recvTime = ros::Time::now();
      //       printf("%d,%d,%d,%d,%d,%d,%f\n",uint32_t(timenow->tm_year + 1900),uint8_t(timenow->tm_mon + 1),uint8_t(timenow->tm_mday),uint8_t(timenow->tm_hour), uint8_t(timenow->tm_min),uint8_t(timenow->tm_sec),
      // float(tv.tv_usec/1000));
          }
          if(CHANNEL_P2 == ch_ && it->first == P2_ANG_RATE_VEHICLE)
          {
            p2Data_send = p2Data;
            cond_p2.notify_one();//触发p2帧尾事件            
//            ROS_INFO("P2 recv over!");
          }
        }
      }
      // usleep(760);
    }
  }
  adcuDevClose(ch_);
  adcuSDKDeinit();
  ROS_INFO("readP2 thread is over!");
}

void P2::p2_Sender()
{
  ROS_INFO("thread p2_Sender Start!");
  unique_lock<mutex> lock(mtx_p2);
  ros::NodeHandle nh;
  ros::Publisher p2_pub = nh.advertise<location_sensor_msgs::IMUAndGNSSInfo>("/drivers/can_wr/imu_gnss_msg", 1000, true);
  
  while(ros::ok())
  {
    ROS_INFO("waiting p2 data!");
    cond_p2.wait(lock);
    location_sensor_msgs::IMUAndGNSSInfo imu_gnss_msg;
    mtx_p2_data.lock();
    // imu_gnss_msg.header.stamp = ros::Time::now();
    imu_gnss_msg.header.stamp = recvTime;
    imu_gnss_msg.GPS_week = p2Data_send.gps_week;
    imu_gnss_msg.GPS_sec  = p2Data_send.gps_time;
    imu_gnss_msg.base_line  = BASE_LINE;
    imu_gnss_msg.NSV1 = p2Data_send.gps_num_status;
    imu_gnss_msg.satellite_status = p2Data_send.satellite_status;
    // imu_gnss_msg.age = 1;
    imu_gnss_msg.system_status = p2Data_send.system_status;
//    imu_gnss_msg.warming = null;uint32_t
    imu_gnss_msg.year = timeDate.year;
    imu_gnss_msg.month = timeDate.month;
    imu_gnss_msg.day = timeDate.day;
    imu_gnss_msg.hour = timeDate.hour;
    imu_gnss_msg.min = timeDate.min;
    imu_gnss_msg.sec = timeDate.sec;
    imu_gnss_msg.msec = timeDate.msec;
    imu_gnss_msg.pose.x = p2Data_send.pos_lat;
    imu_gnss_msg.pose.y = p2Data_send.pos_lon;
    imu_gnss_msg.pose.z = p2Data_send.pos_alt;
    imu_gnss_msg.yaw = p2Data_send.heading;
    imu_gnss_msg.pitch =  p2Data_send.pitch;
    imu_gnss_msg.roll = p2Data_send.roll;
    imu_gnss_msg.velocity.x = p2Data_send.vel_e;
    imu_gnss_msg.velocity.y = p2Data_send.vel_n;
    imu_gnss_msg.velocity.z = p2Data_send.vel_u;
    imu_gnss_msg.accelgyro.linear.x = p2Data_send.accel_vel_x;
    imu_gnss_msg.accelgyro.linear.y = p2Data_send.accel_vel_y;
    imu_gnss_msg.accelgyro.linear.z = p2Data_send.accel_vel_z;
    imu_gnss_msg.accelgyro.angular.x = p2Data_send.ang_rate_x;
    imu_gnss_msg.accelgyro.angular.y = p2Data_send.ang_rate_y;
    imu_gnss_msg.accelgyro.angular.z = p2Data_send.ang_rate_z;
    mtx_p2_data.unlock();
    p2_pub.publish(imu_gnss_msg);
//    ros::spinOnce();
//    ROS_INFO("p2_sender: p2 week: %u time:%u", p2Data.gps_week, p2Data.gps_time);
//    ROS_INFO("p2_sender: ang_rate_x:%f ang_rate_y:%f ang_rate_z:%f", p2Data.ang_rate_x, p2Data.ang_rate_y,
//            p2Data.ang_rate_z);
//    ROS_INFO("p2Data.year:%d",p2Data.year);
  }
}

// int P2::hex2int(uint8_t data[],uint16_t bit_start,uint16_t bit_end)
// {
//   int hex2int_data = 0;
//   uint8_t byte_num = (bit_end - bit_start)/sizeof(uint8_t);
//   uint8_t bete_start = bit_start / 8;
//   for(uint8_t i = 0;i < byte_num;++i)
//   {
//     hex2int_data = 
//   }
// }

//800
void P2::p2TimeParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
  // ROS_INFO("recv: %d p2TimeParse", can_id);
  p2TimeStr p2_time_;
  p2_time_.gps_week = (can_buf.can_data[0]) << 8 | (can_buf.can_data[1]);
  p2_time_.gps_time = (((uint32_t)(can_buf.can_data[2]) << 24) + ((uint32_t)(can_buf.can_data[3]) << 16) +
                      ((uint32_t)(can_buf.can_data[4]) << 8) + (uint32_t)(can_buf.can_data[5]));

  p2DataStr p2_data;
  p2_data.gps_week = p2_time_.gps_week;
  p2_data.gps_time = p2_time_.gps_time;
  p2Data.gps_week = p2_time_.gps_week;
  p2Data.gps_time = (p2_time_.gps_time/10) * 0.01;
//  ROS_INFO("recv: p2 week: %u time:%f", p2_data.gps_week, p2_data.gps_time);
}
//801
void P2::p2AngRateRawIMUParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
  // ROS_INFO("recv: %d p2AngRateRawIMUParse", can_id);
  p2AngRateRawIMUStr p2_ang_rate_rawIMU_;
  // p2_ang_rate_rawIMU_.x = (can_buf.can_data[0]) << 8 | (can_buf.can_data[1]);
  // p2_ang_rate_rawIMU_.y = (can_buf.can_data[2]) << 8 | (can_buf.can_data[3]);
  // p2_ang_rate_rawIMU_.z = (can_buf.can_data[4]) << 8 | (can_buf.can_data[5]);

  if(can_buf.can_data[0] >> 7)
  {
    p2_ang_rate_rawIMU_.x = (0xFFF << 20) | (can_buf.can_data[0]) << 12 | (can_buf.can_data[1]) << 4 | ((can_buf.can_data[2]) >> 4 & 0x0F);
  }
  else
  {
    p2_ang_rate_rawIMU_.x = (can_buf.can_data[0]) << 12 | (can_buf.can_data[1]) << 4 | ((can_buf.can_data[2]) >> 4 & 0x0F);
  }
  if(((can_buf.can_data[2]) & 0x0F) >> 3)
  {
    p2_ang_rate_rawIMU_.y = (0xFFF << 20 | ((can_buf.can_data[2]) & 0x0F) << 16 | (can_buf.can_data[3]) << 8 | (can_buf.can_data[4]));
  }
  else
  {
    p2_ang_rate_rawIMU_.y = ((can_buf.can_data[2]) & 0x0F) << 16 | (can_buf.can_data[3]) << 8 | (can_buf.can_data[4]);
  }
  if(((can_buf.can_data[5]) & 0xF0) >> 7)
  {
    p2_ang_rate_rawIMU_.z = (0xFFF << 20) | (can_buf.can_data[5]) << 12 | (can_buf.can_data[6]) << 4 | ((can_buf.can_data[7]) >> 4 & 0x0F);
  }
  else
  {
    p2_ang_rate_rawIMU_.z = (can_buf.can_data[5]) << 12 | (can_buf.can_data[6]) << 4 | ((can_buf.can_data[7]) >> 4 & 0x0F);
  }

  p2DataStr p2_data;
  p2_data.ang_rate_raw_x = p2_ang_rate_rawIMU_.x;
  p2_data.ang_rate_raw_y = p2_ang_rate_rawIMU_.y;
  p2_data.ang_rate_raw_z = p2_ang_rate_rawIMU_.z;
  p2Data.ang_rate_raw_x = p2_ang_rate_rawIMU_.x * 0.01;
  p2Data.ang_rate_raw_y = p2_ang_rate_rawIMU_.y * 0.01;
  p2Data.ang_rate_raw_z = p2_ang_rate_rawIMU_.z * 0.01;
//  ROS_INFO("recv: IMU raw (%f,%f,%f)", p2_data.ang_rate_raw_x, p2_data.ang_rate_raw_y, p2_data.ang_rate_raw_z);
}
//802
void P2::p2AccelIMURawParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
  // ROS_INFO("recv: %d p2AngRateRawIMUParse", can_id);
  p2AccelIMURawStr p2_accel_rawIMU_;
  p2_accel_rawIMU_.x = (can_buf.can_data[0]) << 12 | (can_buf.can_data[1]) << 4 | ((can_buf.can_data[2]) >> 4 & 0x0F);
  p2_accel_rawIMU_.y = ((can_buf.can_data[2]) & 0x0F) << 12 | (can_buf.can_data[3]) << 8 | (can_buf.can_data[4]);
  p2_accel_rawIMU_.z = (can_buf.can_data[5]) << 12 | (can_buf.can_data[6]) << 4 | ((can_buf.can_data[7]) >> 4 & 0x0F);

  p2DataStr p2_data;
  p2_data.ang_rate_raw_x = p2_accel_rawIMU_.x;
  p2_data.ang_rate_raw_y = p2_accel_rawIMU_.y;
  p2_data.ang_rate_raw_z = p2_accel_rawIMU_.z;
  p2Data.accel_raw_x = p2_accel_rawIMU_.x * 0.001;
  p2Data.accel_raw_x = p2_accel_rawIMU_.y * 0.001;
  p2Data.accel_raw_x = p2_accel_rawIMU_.z * 0.001;
//  ROS_INFO("recv: IMU raw (%f,%f,%f)", p2_data.ang_rate_raw_x, p2_data.ang_rate_raw_y, p2_data.ang_rate_raw_z);
}
//803
void P2::p2InsStatusParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
  // ROS_INFO("recv: %d p2InsStatusParse", can_id);
  p2InsStatusStr p2_ins_status_;
  p2_ins_status_.system_status    = can_buf.can_data[0];
  p2_ins_status_.gps_num_status   = can_buf.can_data[1];
  p2_ins_status_.satellite_status = can_buf.can_data[2];

  p2DataStr p2_data;
  p2_data.system_status    = p2_ins_status_.system_status;
  p2_data.gps_num_status   = p2_ins_status_.gps_num_status;
  p2_data.satellite_status = p2_ins_status_.satellite_status;
  p2Data.system_status    = p2_ins_status_.system_status;
  p2Data.gps_num_status   = p2_ins_status_.gps_num_status;
  p2Data.satellite_status = p2_ins_status_.satellite_status;
//  ROS_INFO("recv: status: sys %u  gps num %u satellite %u", p2_data.system_status, p2_data.gps_num_status,
          // p2_data.satellite_status);
}
//804
void P2::p2LatitudeLongitudeParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
  // ROS_INFO("recv: %d p2LatitudeLongitudeParse", can_id);
  p2LatLonStr p2_lat_lon_;
  p2_lat_lon_.lat = ((uint32_t)(can_buf.can_data[0]) << 24) + ((uint32_t)(can_buf.can_data[1]) << 16) +
                    ((uint32_t)(can_buf.can_data[2]) << 8) + (uint32_t)(can_buf.can_data[3]);
  p2_lat_lon_.lon = ((uint32_t)(can_buf.can_data[4]) << 24) + ((uint32_t)(can_buf.can_data[5]) << 16) +
                    ((uint32_t)(can_buf.can_data[6]) << 8) + (uint32_t)(can_buf.can_data[7]);
  p2DataStr p2_data;
  p2_data.pos_lat = p2_lat_lon_.lat;
  p2_data.pos_lon = p2_lat_lon_.lon;
  p2Data.pos_lat = p2_lat_lon_.lat * 0.0000001;
  p2Data.pos_lon = p2_lat_lon_.lon * 0.0000001;
//  ROS_INFO("recv: lat %f lon %f", p2_data.pos_lat, p2_data.pos_lon);
}

void P2::p2AltitudeParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
  // ROS_INFO("recv: %d p2AltitudeParse", can_id);
  p2AltStr p2_alt_;
  p2_alt_.alt = ((uint32_t)(can_buf.can_data[0]) << 24) + ((uint32_t)(can_buf.can_data[1]) << 16) +
                ((uint32_t)(can_buf.can_data[2]) << 8) + (uint32_t)(can_buf.can_data[3]);

  p2DataStr p2_data;
  p2_data.pos_alt = p2_alt_.alt;
  p2Data.pos_alt = p2_alt_.alt * 0.001;
//  ROS_INFO("recv: alt %f", p2_data.pos_alt);
}
void P2::p2PosSigmaParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
  // ROS_INFO("recv: %d p2PosSigmaParse", can_id);
  p2PosSigmaStr p2_pos_sigma_;
  p2_pos_sigma_.e = (can_buf.can_data[0]) << 8 | (can_buf.can_data[1]);
  p2_pos_sigma_.n = (can_buf.can_data[2]) << 8 | (can_buf.can_data[3]);
  p2_pos_sigma_.u = (can_buf.can_data[4]) << 8 | (can_buf.can_data[5]);

  p2DataStr p2_data;
  p2_data.pos_e_sigma = p2_pos_sigma_.e;
  p2_data.pos_n_sigma = p2_pos_sigma_.n;
  p2_data.pos_u_sigma = p2_pos_sigma_.u;
  p2Data.pos_e_sigma = p2_pos_sigma_.e * 0.01;
  p2Data.pos_n_sigma = p2_pos_sigma_.n * 0.01;
  p2Data.pos_u_sigma = p2_pos_sigma_.u * 0.01;
//  ROS_INFO("recv: sigma E:%u N:%u U:%u", p2_data.pos_e_sigma, p2_data.pos_n_sigma, p2_data.pos_u_sigma);
}
void P2::p2VelocityLevelParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
  // ROS_INFO("recv: %d p2VelocityLevelParse", can_id);
  p2VelocityLevelStr p2_vel_level_;
  p2_vel_level_.e   = (can_buf.can_data[0]) << 8 | (can_buf.can_data[1]);
  p2_vel_level_.n   = (can_buf.can_data[2]) << 8 | (can_buf.can_data[3]);
  p2_vel_level_.u   = (can_buf.can_data[4]) << 8 | (can_buf.can_data[5]);
  p2_vel_level_.vel = (can_buf.can_data[6]) << 8 | (can_buf.can_data[7]);

  p2DataStr p2_data;
  p2_data.vel_e = p2_vel_level_.e;
  p2_data.vel_n = p2_vel_level_.n;
  p2_data.vel_u = p2_vel_level_.u;
  p2_data.vel   = p2_vel_level_.vel;
  p2Data.vel_e = p2_vel_level_.e * 0.01;
  p2Data.vel_n = p2_vel_level_.n * 0.01;
  p2Data.vel_u = p2_vel_level_.u * 0.01;
  p2Data.vel   = p2_vel_level_.vel * 0.01;
//  ROS_INFO("recv: Vel:%f E:%f N:%f U:%f", p2_data.vel, p2_data.vel_e, p2_data.vel_n, p2_data.vel_u);
}
void P2::p2VelocityLevelSigmaParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
  // ROS_INFO("recv: %d p2VelocityLevelSigmaParse", can_id);
  p2VelocityLevelSigmaStr p2_vel_level_sigma_;
  p2_vel_level_sigma_.e   = (can_buf.can_data[0]) << 8 | (can_buf.can_data[1]);
  p2_vel_level_sigma_.n   = (can_buf.can_data[2]) << 8 | (can_buf.can_data[3]);
  p2_vel_level_sigma_.u   = (can_buf.can_data[4]) << 8 | (can_buf.can_data[5]);
  p2_vel_level_sigma_.vel = (can_buf.can_data[6]) << 8 | (can_buf.can_data[7]);

  p2DataStr p2_data;
  p2_data.vel_e_sigma = p2_vel_level_sigma_.e;
  p2_data.vel_n_sigma = p2_vel_level_sigma_.n;
  p2_data.vel_u_sigma = p2_vel_level_sigma_.u;
  p2_data.vel_sigma   = p2_vel_level_sigma_.vel;
  p2Data.vel_e_sigma = p2_vel_level_sigma_.e * 0.01;
  p2Data.vel_n_sigma = p2_vel_level_sigma_.n * 0.01;
  p2Data.vel_u_sigma = p2_vel_level_sigma_.u * 0.01;
  p2Data.vel_sigma   = p2_vel_level_sigma_.vel * 0.01;
//  ROS_INFO("recv: sigma Vel:%f E:%f N:%f U:%f", p2_data.vel_sigma, p2_data.vel_e_sigma, p2_data.vel_n_sigma,
          // p2_data.vel_u_sigma);
}
void P2::p2AccelVehicleParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
//  ROS_INFO("recv: %d p2VelocityLevelSigmaParse", can_id);
  p2AccelVehicleStr p2_accel_vehicle_;
  // p2_accel_vehicle_.x = (can_buf.can_data[0]) << 12 | (can_buf.can_data[1]) << 4 | ((can_buf.can_data[2]) >> 4 & 0x0F);
  // p2_accel_vehicle_.y = ((can_buf.can_data[2]) & 0x0F) << 12 | (can_buf.can_data[3]) << 8 | (can_buf.can_data[4]);
  // p2_accel_vehicle_.z = (can_buf.can_data[5]) << 12 | (can_buf.can_data[6]) << 4 | ((can_buf.can_data[7]) >> 4 & 0x0F);

  if(((can_buf.can_data[0]) & 0xF0) >> 7)
  {
    p2_accel_vehicle_.x = (0xFFF << 20) | (can_buf.can_data[0]) << 12 | (can_buf.can_data[1]) << 4 | ((can_buf.can_data[2]) >> 4 & 0x0F);
  }
  else
  {
    p2_accel_vehicle_.x = (can_buf.can_data[0]) << 12 | (can_buf.can_data[1]) << 4 | ((can_buf.can_data[2]) >> 4 & 0x0F);
  }
  if(((can_buf.can_data[2]) & 0x0F) >> 3)
  {
    p2_accel_vehicle_.y = (0xFFF << 20 | ((can_buf.can_data[2]) & 0x0F) << 16 | (can_buf.can_data[3]) << 8 | (can_buf.can_data[4]));
  }
  else
  {
    p2_accel_vehicle_.y = ((can_buf.can_data[2]) & 0x0F) << 16 | (can_buf.can_data[3]) << 8 | (can_buf.can_data[4]);
  }
  if(((can_buf.can_data[5]) & 0xF0) >> 7)
  {
    p2_accel_vehicle_.z = (0xFFF << 20) | (can_buf.can_data[5]) << 12 | (can_buf.can_data[6]) << 4 | ((can_buf.can_data[7]) >> 4 & 0x0F);
  }
  else
  {
    p2_accel_vehicle_.z = (can_buf.can_data[5]) << 12 | (can_buf.can_data[6]) << 4 | ((can_buf.can_data[7]) >> 4 & 0x0F);
  }

  p2DataStr p2_data;
  p2_data.accel_vel_x = p2_accel_vehicle_.x;
  p2_data.accel_vel_y = p2_accel_vehicle_.y;
  p2_data.accel_vel_z = p2_accel_vehicle_.z;
  p2Data.accel_vel_x = p2_accel_vehicle_.x * 0.0001 * 9.80665;
  p2Data.accel_vel_y = p2_accel_vehicle_.y * 0.0001 * 9.80665;
  p2Data.accel_vel_z = p2_accel_vehicle_.z * 0.0001 * 9.80665;
//  ROS_INFO("recv: accel x:%f y:%f z:%f", p2_data.accel_vel_x, p2_data.accel_vel_y, p2_data.accel_vel_z);
}
void P2::p2HeadingPitchRollParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
  // ROS_INFO("recv: %d p2HeadingPitchRollParse", can_id);
  p2HeadingPitchRollStr p2_heading_pitch_roll_;
  p2_heading_pitch_roll_.heading = (can_buf.can_data[0]) << 8 | (can_buf.can_data[1]);
  p2_heading_pitch_roll_.pitch   = (can_buf.can_data[2]) << 8 | (can_buf.can_data[3]);
  p2_heading_pitch_roll_.roll    = (can_buf.can_data[4]) << 8 | (can_buf.can_data[5]);

  p2DataStr p2_data;
  p2_data.heading = p2_heading_pitch_roll_.heading;
  p2_data.pitch   = p2_heading_pitch_roll_.pitch;
  p2_data.roll    = p2_heading_pitch_roll_.roll;
  p2Data.heading = p2_heading_pitch_roll_.heading * 0.01;
  p2Data.pitch   = p2_heading_pitch_roll_.pitch * 0.01;
  p2Data.roll    = p2_heading_pitch_roll_.roll * 0.01;
//  ROS_INFO("recv: heading:%f pitch:%f roll:%f", p2_data.heading, p2_data.pitch, p2_data.roll);
}
void P2::p2HeadingPitchRollSigmaParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
  // ROS_INFO("recv: %d p2HeadingPitchRollSigmaStr", can_id);
  p2HeadingPitchRollSigmaStr p2_heading_pitch_roll_sigma_;
  p2_heading_pitch_roll_sigma_.heading = (can_buf.can_data[0]) << 8 | (can_buf.can_data[1]);
  p2_heading_pitch_roll_sigma_.pitch   = (can_buf.can_data[2]) << 8 | (can_buf.can_data[3]);
  p2_heading_pitch_roll_sigma_.roll    = (can_buf.can_data[4]) << 8 | (can_buf.can_data[5]);

  p2DataStr p2_data;
  p2_data.heading_sigma = p2_heading_pitch_roll_sigma_.heading;
  p2_data.pitch_sigma   = p2_heading_pitch_roll_sigma_.pitch;
  p2_data.roll_sigma    = p2_heading_pitch_roll_sigma_.roll;
  p2Data.heading_sigma = p2_heading_pitch_roll_sigma_.heading * 0.01;
  p2Data.pitch_sigma   = p2_heading_pitch_roll_sigma_.pitch * 0.01;
  p2Data.roll_sigma    = p2_heading_pitch_roll_sigma_.roll * 0.01;
//  ROS_INFO("recv: sigma heading:%f pitch:%f roll:%f", p2_data.heading_sigma, p2_data.pitch_sigma, p2_data.roll_sigma);
}
void P2::p2AngRateVehicleParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
  // ROS_INFO("recv: %d p2AngRateVehicleParse", can_id);
  p2AngRateVehicleStr p2_ang_vel_;
  // p2_ang_vel_.x = (can_buf.can_data[0]) << 8 | (can_buf.can_data[1]);
  // p2_ang_vel_.y = (can_buf.can_data[2]) << 8 | (can_buf.can_data[3]);
  // p2_ang_vel_.z = (can_buf.can_data[4]) << 8 | (can_buf.can_data[5]);

  if(((can_buf.can_data[0]) & 0xF0) >> 7)
  {
    p2_ang_vel_.x = (0xFFF << 20) | (can_buf.can_data[0]) << 12 | (can_buf.can_data[1]) << 4 | ((can_buf.can_data[2]) >> 4 & 0x0F);
  }
  else
  {
    p2_ang_vel_.x = (can_buf.can_data[0]) << 12 | (can_buf.can_data[1]) << 4 | ((can_buf.can_data[2]) >> 4 & 0x0F);
  }
  if(((can_buf.can_data[2]) & 0x0F) >> 3)
  {
    p2_ang_vel_.y = (0xFFF << 20 | ((can_buf.can_data[2]) & 0x0F) << 16 | (can_buf.can_data[3]) << 8 | (can_buf.can_data[4]));
  }
  else
  {
    p2_ang_vel_.y = ((can_buf.can_data[2]) & 0x0F) << 16 | (can_buf.can_data[3]) << 8 | (can_buf.can_data[4]);
  }
  if(((can_buf.can_data[5]) & 0xF0) >> 7)
  {
    p2_ang_vel_.z = (0xFFF << 20) | (can_buf.can_data[5]) << 12 | (can_buf.can_data[6]) << 4 | ((can_buf.can_data[7]) >> 4 & 0x0F);
  }
  else
  {
    p2_ang_vel_.z = (can_buf.can_data[5]) << 12 | (can_buf.can_data[6]) << 4 | ((can_buf.can_data[7]) >> 4 & 0x0F);
  }

  p2DataStr p2_data;

  p2_data.ang_rate_x = p2_ang_vel_.x;
  p2_data.ang_rate_y = p2_ang_vel_.y;
  p2_data.ang_rate_z = p2_ang_vel_.z;
  p2Data.ang_rate_x = p2_ang_vel_.x * 0.01;
  p2Data.ang_rate_y = p2_ang_vel_.y * 0.01;
  p2Data.ang_rate_z = p2_ang_vel_.z * 0.01;
//  ROS_INFO("recv: ang_rate_x:%f ang_rate_y:%f ang_rate_z:%f", p2_data.ang_rate_x, p2_data.ang_rate_y,
          // p2_data.ang_rate_z);
}

void P2::test()
{
  ROS_INFO("p2::test~~~~");
}

}
}
