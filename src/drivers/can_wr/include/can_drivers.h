#ifndef DRIVERS_CAN_WR_INCLUDE_CAN_DRIVERS_H_
#define DRIVERS_CAN_WR_INCLUDE_CAN_DRIVERS_H_

#include "adcuSDK.h"
#include "can_rw.h"
#include "common_msgs/DetectionInfo.h"
#include "geometry_msgs/Point.h"
#include "location_sensor_msgs/IMUAndGNSSInfo.h"
#include "perception_sensor_msgs/ObjectList.h"
#include "perception_sensor_msgs/UltrasonicInfo.h"
#include "protocol.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/UInt32MultiArray.h"
#include "threadpool.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <errno.h>
#include <glog/logging.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace superg_agv
{
namespace drivers
{

#define MODE_SIZE_CAR 4.0
#define MODE_SIZE_PES 0.8
#define MODE_SIZE_BICYCLE 1.8
#define MODE_SIZE_ROADBLOCK 0.5 //锥桶

#define THETA_Y 0.1
#define MAT_HEIGHT 1260
#define MAT_WIDTH 720

#define CHANNEL_P2 2
#define CHANNEL_CAMERA_1 3
#define CHANNEL_CAMERA_2 4
#define CHANNEL_ULTRASONIC 5
#define CHANNEL_MOBILEYE 6

#define CAN_DEVCOUNT_MAX 4

#define MAX_CAN_BUF_NUM 8
#define MAX_CAN_BUF_LEGTH 4 * 1024
#define MAX_CAMERA_OBJECT_NUM 32

#define ALL_START_WORK 124
#define ALL_GET_STATUS 125
#define ALL_STOP_ACQUIRE 126

#define ULTRASONIC_CONTROL 127

//非map中key，超声波控制
#define ULTRASONIC_CONTROLSTOPWORK_1 1
#define ULTRASONIC_CONTROLSTOPWORK_2 2
#define ULTRASONIC_CONTROLSTOPWORK_3 4
#define ULTRASONIC_CONTROLSTOPWORK_4 8

#define ULTRASONIC_CONTROLSTARTWORK_1 16
#define ULTRASONIC_CONTROLSTARTWORK_2 32
#define ULTRASONIC_CONTROLSTARTWORK_3 64
#define ULTRASONIC_CONTROLSTARTWORK_4 128

#define ULTRASONIC_ALLSTOPACQUIRE 256
#define ULTRASONIC_ALLGETSTATUS 512
#define ULTRASONIC_ALLSTARTWORK 1024

#define ULTRASONIC_SAFEDISTANCE 2
#define ULTRASONIC_MINDISTANCE 00
//非map中key，超声波控制

#define ULTRASONIC_DATA_1 601
#define ULTRASONIC_DATA_2 602
#define ULTRASONIC_DATA_3 603
#define ULTRASONIC_DATA_4 604

#define SYSNSTARWORK_INVALID 0
#define SYSMSTARWPRK_VALID 1
#define FRE 0 // CAMERA输出频率 0:10Hz , 1:15Hz

#define CAMERA_START_WORK 300  // CAMERA ID + 该值
#define CAMERA_STOP_WORK 301   // CAMERA ID + 该值
#define CAMERA_WORK_STATUS 302 // CAMERA ID + 该值

#define CAMERA_DATA_HEAD 200 // CAMERA ID + 该值
#define CAMERA_DATA_0 201    // CAMERA ID + 该值
#define CAMERA_DATA_1 202    // CAMERA ID + 该值

#define CAMERA_MAT_HEAD 203
#define CAMERA_MAT_RAW 204
#define CAMERA_MAT_TAIL 205
#define CAMERA_MAT_DATA_BEGIN 1101
#define CAMERA_MAT_DATA_END 1258

#define MAX_CAMERA_NUM 8

#define CAMERA_OBJECT_1 1
#define CAMERA_LANE_2 2
#define CAMERA_OBJECT_3 3
#define CAMERA_LANE_4 4
#define CAMERA_OBJECT_5 5
#define CAMERA_OBJECT_6 6
#define CAMERA_OBJECT_7 7
#define CAMERA_OBJECT_8 8

#define P2_TIME 800
#define P2_ANG_RATE_RAW_IMU 801
#define P2_ACCEL_IMU_RAW 802
#define P2_INS_STATUS 803
#define P2_LATITUDE_LONGITUDE 804
#define P2_ALTITUDE 805
#define P2_POS_SIGMA 806
#define P2_VELOCITY_LEVEL 807
#define P2_VELOCITY_LEVEL_SIGMA 808
#define P2_ACCEL_VEHICLE 809
#define P2_HEADING_PITCH_ROLL 810
#define P2_HEADING_PITCH_ROLL_SIGMA 811
#define P2_ANG_RATE_VEHICLE 812

#define BASE_LINE 1

// mobileye key
#define MOBILEYE_LANE_INFO_AND_MEASURE 1641
#define MOBILEYE_SYSTE_MWARING 1792
#define MOBILEYE_TSR_TYPE_AND_POSITION 1824
#define MOBILEYE_TSR_VISION_DECISION 1831
#define MOBILEYE_LIGHTS_LOCATION_AND_ANGLES 1833
#define MOBILEYE_LANE_INFO_MEASUREMENTS 1847
#define MOBILEYE_NUMBER_OF_OBSTACLES 1848
#define MOBILEYE_OBSTACLE_DATA_A 1849
#define MOBILEYE_OBSTACLE_DATA_B 1850
#define MOBILEYE_OBSTACLE_DATA_C 1851
#define MOBILEYE_SIGNALS_STATUS_VEHICLE 1888
#define MOBILEYE_LKA_LEFT_LANE_A 1894
#define MOBILEYE_LKA_LEFT_LANE_B 1895
#define MOBILEYE_LKA_RIGHT_LANE_A 1896
#define MOBILEYE_LKA_RIGHT_LANE_B 1897
#define MOBILEYE_REFERENCE_POINTS 1898
#define MOBILEYE_NUMBER_OF_NEXT_LANE 1899
#define MOBILEYE_LEFT_NEXT_LANE_A 1900
#define MOBILEYE_LEFT_NEXT_LANE_B 1901
#define MOBILEYE_RIGHT_NEXT_LANE_A 1902
#define MOBILEYE_RIGHT_NEXT_LANE_B 1903

// mobileye key

} // namespace drivers
} // namespace superg_agv

#endif
