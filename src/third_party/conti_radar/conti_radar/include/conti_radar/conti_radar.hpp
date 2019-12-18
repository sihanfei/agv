#ifndef CONTI_RADAR_H_
#define CONTI_RADAR_H_
#include <vector>
#include <iostream>

using namespace std;


enum ContiObjectType {
  CONTI_POINT = 0,
  CONTI_CAR = 1,
  CONTI_TRUCK = 2,
  CONTI_PEDESTRIAN = 3,
  CONTI_MOTOCYCLE = 4,
  CONTI_BICYCLE = 5,
  CONTI_WIDE = 6,
  CONTI_UNKNOWN = 7,
  CONTI_MAX_OBJECT_TYPE = 8,
};

enum OutputType
{
  OUTPUT_TYPE_NONE=0,
  OUTPUT_TYPE_OBJECTS=1,
  OUTPUT_TYPE_CLUSTERS=2,
  OUTPUT_TYPE_ERROR=3
};

enum RcsThreshold
{
  RCS_THRESHOLD_STANDARD = 0,
  RCS_THRESHOLD_HIGH_SENSITIVITY = 1,
  RCS_THRESHOLD_ERROR = 2
};

struct ContiRadarObs
{
    bool clusterortrack ; // 0 = track, 1 = cluster

    int32_t obstacle_id ; // obstacle Id
        // 0: point; 1: car; 2: truck; 3: pedestrian; 4: motorcycle; 5: bicycle; 6: wide; 7: unknown
    int32_t obstacle_class ;
    // longitude distance to the radar; (+) = forward; unit = m
    double longitude_dist ;
    // lateral distance to the radar; (+) = left; unit = m
    double lateral_dist;
    // longitude velocity to the radar; (+) = forward; unit = m/s
    double longitude_vel ;
    // lateral velocity to the radar; (+) = left; unit = m/s
    double lateral_vel;
    // obstacle Radar Cross-Section; unit = dBsm
    double rcs;//雷达散射截面
    // 0 = moving, 1 = stationary, 2 = oncoming, 3 = stationary candidate
    // 4 = unknown, 5 = crossing stationary, 6 = crossing moving, 7 = stopped
    int32_t dynprop;    //目标状态
    // longitude distance standard deviation to the radar; (+) = forward; unit = m
    double longitude_dist_rms;
    // lateral distance standard deviation to the radar; (+) = left; unit = m
    double lateral_dist_rms;
    // longitude velocity standard deviation to the radar; (+) = forward; unit = m/s
    double longitude_vel_rms;
    //lateral velocity standard deviation to the radar; (+) = left; unit = m/s
    double lateral_vel_rms;
    // obstacle probability of existence
    double probexist; //置信度
    //The following is only valid for the track object message
    // 0 = deleted, 1 = new, 2 = measured, 3 = predicted, 4 = deleted for, 5 = new from merge
    int32_t meas_state ;//测量状态,指示目标是否有效,并在新的测量周期中被集群确认
    // longitude acceleration to the radar; (+) = forward; unit = m/s2
    double longitude_accel;
    // lateral acceleration to the radar; (+) = left; unit = m/s2
    double lateral_accel;
    // orientation angle to the radar; (+) = counterclockwise; unit = degree
    double oritation_angle;
    // longitude acceleration standard deviation to the radar; (+) = forward; unit = m/s2
    double longitude_accel_rms ;
    // lateral acceleration standard deviation to the radar; (+) = left; unit = m/s2
    double lateral_accel_rms;
    // orientation angle standard deviation to the radar; (+) = counterclockwise; unit = degree
    double oritation_angle_rms; //目标的方位角标准误差

    double length; // obstacle length; unit = m

    double width;//obstacle width; unit = m

};

struct RadarState_201
{

uint32_t max_distance;
uint32_t radar_power;
OutputType output_type;
RcsThreshold rcs_threshold;
bool send_quality ;
bool send_ext_info;

};

struct ClusterListStatus_600
{

  int32_t near;  //default = 0
  int32_t far;  //default = 0
  int32_t meas_counter;  //default = -1
  int32_t interface_version; //default = 4

};

struct ObjectListStatus_60A
{

    int32_t nof_objects; //[default = 0]
    int32_t meas_counter; //[default = -1];//测量周期计数器 (从传感器启动之后就开始计数,当大于65535计数器就清零)
    int32_t interface_version;

};

class ContiRadar
{
public:
   struct RadarState_201 radar_state ;
   struct ClusterListStatus_600 cluster_list_status;
   struct ObjectListStatus_60A object_list_status;
   std::vector<ContiRadarObs>  conti_radar_obs;

};


class RadarConf
{

public:
    bool max_distance_valid ;
    bool sensor_id_valid ;
    bool radar_power_valid ;
    bool output_type_valid ;
    bool send_quality_valid ;
    bool send_ext_info_valid ;
    bool sort_index_valid ;
    bool store_in_nvm_valid;
    bool ctrl_relay_valid;
    bool rcs_threshold_valid;

    uint32_t max_distance;
    uint32_t sensor_id;
    OutputType output_type;
    uint32_t radar_power;
    uint32_t ctrl_relay;
    bool send_ext_info;
    bool send_quality;
    uint32_t sort_index ;
    uint32_t store_in_nvm ;
    RcsThreshold rcs_threshold;
};
#endif // CONTI_RADAR_H_
