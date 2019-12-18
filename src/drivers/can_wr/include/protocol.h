#ifndef DRIVERS_CAN_WR_INCLUDE_PROTOCOL_H_
#define DRIVERS_CAN_WR_INCLUDE_PROTOCOL_H_

#include <cstdio>
#include <cstdlib>
#include <list>

using namespace std;

namespace superg_agv
{
namespace drivers
{
enum class_enum
{
  NONE_OBJECT,
  CAR,
  PES,
  BICYCLE,
  ROADBLOCK
};

struct mobileyeSystemWaringStr
{
  uint8_t sound_type            : 3;
  uint8_t Peds_in_DZ            : 1;
  uint8_t Peds_FCW              : 1;
  uint8_t FCW_on                : 1;
  uint8_t Time_Indicator        : 2;
  uint8_t Error_Valid           : 1;
  uint8_t Error_Code            : 7;
  uint8_t Zero_speed            : 1;
  uint8_t Headway_Valid         : 1;
  float Headway_measurement;
  uint8_t LDW_Off               : 1;
  uint8_t Left_LDW_On           : 1;
  uint8_t Right_LDW_On          : 1;
  uint8_t Maintenance           : 1;
  uint8_t FailSafe              : 1;
  uint8_t FCW_On                : 1;
  uint8_t TSR_Enabled           : 1;
  uint8_t HW_Repeatable_Enabled : 1;
  uint8_t Headway_Warning_Level : 2;
  uint8_t TSR_Warning_Level     : 3;
  uint8_t Tamper_Alert          : 1;
};

struct mobileyeTSRTypeAndPositionStr
{
  uint8_t Vision_Only_Sign_Type   : 8;
  uint8_t Supplementary_Sign_Type : 8;
  uint8_t Sign_Position_X         : 8;
  int8_t Sign_Position_Y         : 7;
  int8_t Sign_Position_Z         : 6;
  uint8_t Filter_Type             : 8;
};

struct mobileyeTSRVisionOnlyDecisionStr
{
  uint8_t Vision_only_Sign_Type_1 : 8;
  uint8_t Vision_only_Sign_Type_2 : 8;
  uint8_t Vision_only_Sign_Type_3 : 8;
  uint8_t Vision_only_Sign_Type_4 : 8;
  uint8_t Vision_only_Supplementary_Sign_Type_1 : 8;
  uint8_t Vision_only_Supplementary_Sign_Type_2 : 8;
  uint8_t Vision_only_Supplementary_Sign_Type_3 : 8;
  uint8_t Vision_only_Supplementary_Sign_Type_4 : 8;
};

struct mobileyeLightsLocationAndAnglesStr
{
  int8_t BNDRY_DOM_BOT_NGL_HLB    : 8;
  int16_t BNDRY_DOM_NGL_LH_HLB    : 12;
  int16_t BNDRY_DOM_NGL_RH_HLB    : 12;
  uint16_t OBJ_DIST_HLB             : 8;
  uint8_t ST_BNDRY_DOM_BOT_NGL_HLB : 2;
  uint8_t ST_BNDRY_DOM_NGL_LH_HLB  : 2;
  uint8_t ST_BNDRY_DOM_NGL_RH_HLB  : 2;
  uint8_t ST_OBJ_DIST_HLB          : 2;
  uint8_t Left_Target_Change       : 1;
  uint8_t Right_Target_Change      : 1;
  uint8_t Too_Many_Cars            : 1;
  uint8_t Busy_Scene               : 1;
};

struct mobileyeLaneInfoMeasureStr
{
  double Lane_Curvature;//Units: 1/m
  float Lane_Heading;
  uint8_t CA_construction_area   : 1;
  float Pitch_Angle;//Unit: radians
  float Yaw_Angle;//Unit: radians
  uint8_t Right_LDW_Availability : 1;
  uint8_t Left_LDW_Availability  : 1;
};

struct mobileyeNumberOfObstaclesStr
{
  uint8_t Number_Of_Obstacles     : 8;
  uint8_t Timestamp               : 8;
  uint8_t Left_Close_Rang_Cut_In  : 1;
  uint8_t Right_Close_Rang_Cut_In : 1;
  uint8_t Go                      : 4;
  uint8_t Close_car               : 1;
  uint8_t Failsafe                : 4;
};

struct mobileyeObstaclesDataStr
{
  uint8_t Obstacle_ID           : 8;
  float Obstacle_Position_X;//Unit: meter
  float Obstacle_Position_Y;//Unit: meter
  float Obstacle_Relative_Velocity_X;//Unit:meter/sec
  uint8_t Obstacle_Type         : 3;
  uint8_t Obstacle_Status       : 3;
  uint8_t Obstacle_Brake_Lights : 1;
  uint8_t Cut_In_And_Out        : 3;
  uint8_t Blinker_Info          : 3;
  uint8_t Obstacle_Valid        : 2;

  float Obstacle_Length;//Unit: meter
  float Obstacle_Width;//Unit: meter
  uint8_t Obstacle_Age          : 8;
  uint8_t Obstacle_Lane         : 2;
  uint8_t CIPV_Flag             : 1;
  float Radar_Position_X;//Unit: meter
  float Radar_Velocity_X;//Unit: meter/sec
  uint8_t Radar_Match_Confidence: 3;
  uint8_t Matched_Radar_ID      : 7;

  float Obstacle_Angle_Rate;//Unit: degree
  float Obstacle_Scale_Change;//Unit: pix/sec
  float Object_Accel_X;//m/s 2
  uint8_t Obstacle_Replaced     : 1;
  float Obstacle_Angle;//Unit: degree 
};

struct mobileyeSignalsStatusStr
{
  uint8_t High_Beam           : 1;
  uint8_t Low_Beam            : 1;
  uint8_t Wipers              : 1;
  uint8_t Right_signal        : 1;
  uint8_t Left_signal         : 1;
  uint8_t Brake_signal        : 1;
  uint8_t Wipers_available    : 1;
  uint8_t Low_Beam_available  : 1;
  uint8_t High_Beam_Available : 1;
  uint8_t Speed_Available     : 1;
  uint8_t Speed               : 8;//Unit: km/h
};

struct mobileyeLKALaneStr
{
  uint8_t Lane_type               : 4;
  uint8_t Quality                 : 2;
  uint8_t Model_degree            : 2;
  float Position_Parameter_C0;//Unit: meter
  float Curvature_Parameter_C2;//Unit: n/a
  float Curvature_Derivative_Parameter_C3;//Unit: n/a
  float Width_Marking;//Unit: m

  float Heading_Angle_Parameter_C1;//Unit: radians
  float View_Range;//Unit: meter
  uint8_t View_range_availability : 1;

};

struct mobileyeReferencePointsStr
{
  float Ref_Point_1_Position;//Unit: meters
  float Ref_Point_1_Distance;//Unit: meters
  uint8_t Ref_Point_1_Validity : 1;;
  float Ref_Point_2_Position;//Unit: meters
  float Ref_Point_2_Distance;//Unit: meters
  uint8_t Ref_Point_2_Validity : 1;;
};

struct mobileyeNUmberOfNextLaneStr
{
  uint8_t Number_Of_Next_Lane_Markers_Reported : 8;
};

struct mobileyeDataStr
{
  uint8_t  lane_type_left        : 4;
  uint16_t distance_lane_left    : 12;
  uint8_t  confidence_lane_left  : 2;
  uint8_t  LDW_left              : 1;
  uint8_t  lane_type_right       : 4;
  uint16_t distance_lane_right   : 12;
  uint8_t  confidence_lane_right : 2;
  uint8_t  LDW_right             : 1;
  bool bObstaclesReady = false;
  bool bLaneReady = false;
  mobileyeSystemWaringStr mobileyeSystemWaring;

  mobileyeTSRTypeAndPositionStr mobileyeTSRTypeAndPosition;
  
  mobileyeTSRVisionOnlyDecisionStr mobileyeTSRVisionOnlyDecision;

  mobileyeLightsLocationAndAnglesStr mobileyeLightsLocationAndAngles;

  mobileyeLaneInfoMeasureStr mobileyeLaneInfoMeasure;

  mobileyeNumberOfObstaclesStr mobileyeNumberOfObstacles;

  list<mobileyeObstaclesDataStr> mobileyeObstaclesData;

  mobileyeSignalsStatusStr mobileyeSignalsStatus;

  list<mobileyeLKALaneStr> mobileyeLKALeftLane;

  list<mobileyeLKALaneStr> mobileyeLKARightLane;

  mobileyeReferencePointsStr mobileyeReferencePoints;
  
  mobileyeNUmberOfNextLaneStr mobileyeNUmberOfNextLane;
};

struct ultrasonicInfoStr
{
  uint32_t id;
  float distance;
  bool status;
};

struct ultrasonicDataStr
{
  float ultrasonic1_distance;
  float ultrasonic2_distance;
  float ultrasonic3_distance;
  float ultrasonic4_distance;
  float ultrasonic5_distance;
  float ultrasonic6_distance;
  float ultrasonic7_distance;
  float ultrasonic8_distance;
  uint8_t ultrasonic1_status : 1;
  uint8_t ultrasonic2_status : 1;
  uint8_t ultrasonic3_status : 1;
  uint8_t ultrasonic4_status : 1;
  uint8_t ultrasonic5_status : 1;
  uint8_t ultrasonic6_status : 1;
  uint8_t ultrasonic7_status : 1;
  uint8_t ultrasonic8_status : 1;
  uint8_t controller_id : 2;
  uint32_t recv_count = 0;
  list<ultrasonicInfoStr> ultrasonicInfo_;
  bool bReady = false;
};

struct ObjectData
{
  u_char id;
  class_enum obj_class = CAR;
  float position[2];
  float velocity;
  float confidence;
  float width;
  uint polygon[4];
};

struct ObjectStrHead
{
  uint obj_num : 5;
  uint r_time : 27;
};

struct ObjectStr0
{
  uint id : 8;
  uint obj_class : 8;
  uint confidence : 8;
  int position_x : 12;
  int position_y : 12;
  int velocity : 10;
};

struct ObjectStr1
{
  uint id : 8;
  uint width : 12;
  uint polygon_x_min : 12;
  uint polygon_x_max : 12;
  uint polygon_y_min : 10;
  uint polygon_y_max : 10;
};

struct laneStrHead
{
  uint lane_num : 3;
  uint r_time : 27;
};

struct laneStr0
{
  uint id : 8;
  int start_point : 16;
  int end_point : 16;
  int distance : 16;
  uint slope : 8;
};

struct laneStr1
{
  int polynomial_a : 16;
  int polynomial_b : 16;
  int polynomial_c : 16;
  int polynomial_d : 13;
  uint id : 3;
};

struct cameraObjectInfo
{
  uint32_t object_id;
  class_enum obj_class;
  float position_x;
  float position_y;
  //int32_t position_x;
  //int32_t position_y;
  float velocity;
  float confidence;
  float width;
  uint16_t polygon_x_min;
  uint16_t polygon_x_max;
  uint16_t polygon_y_min;
  uint16_t polygon_y_max;
};

struct camera_ObjectData_state
{
  uint8_t sensor_index;
  uint8_t count;
  uint8_t recv_count;
  uint32_t time;
  bool bReady;
  list<cameraObjectInfo> cameraobjectinfo;
};

struct cameraLanePoint
{
  float x;
  float y;
  float z;
};

struct cameraLaneInfo
{
  uint8_t lane_id;
  float start_point;
  float end_point;
  float distance;
  uint16_t slope;
  float polynomial_a;
  float polynomial_b;
  float polynomial_c;
  float polynomial_d;
};

struct camera_LaneData_state
{
  uint8_t count;
  uint8_t recv_count;
  uint32_t time;
  bool bReady;
  list<cameraLaneInfo> cameralaneinfo;
};

struct cameraData
{
  uint8_t camera_id;
  uint8_t object_num;  // 0-31
  uint8_t lane_num;
  uint32_t time;  // ms
};

// struct cameraLaneDataStr
// {
//   uint8_t camera_id;
//   uint8_t lane_num;
//   uint32_t time;
//   list<cameraLaneInfo> cameralaneinfo;
//   bool bReady = false;
// };

struct canOrder
{
  uint8_t reserve;
};

struct matDataStr
{
  uint32_t height;
  uint32_t width;
  list<uint8_t> data;
  bool bReady = false;
};

// p2
struct p2TimeStr
{
  uint16_t gps_week : 16;
  uint32_t gps_time : 32;
};

struct p2AngRateRawIMUStr
{
  int x : 16;
  int y : 16;
  int z : 16;
};

struct p2AccelIMURawStr
{
  int x : 20;
  int y : 20;
  int z : 20;
};

struct p2InsStatusStr
{
  uint8_t system_status : 8;
  uint8_t gps_num_status : 8;
  uint8_t satellite_status : 8;
};

struct p2LatLonStr
{
  int lat : 32;
  int lon : 32;
};
struct p2AltStr
{
  int alt : 32;
};

struct p2PosSigmaStr
{
  uint16_t e : 16;
  uint16_t n : 16;
  uint16_t u : 16;
};

struct p2VelocityLevelStr
{
  int e : 16;
  int n : 16;
  int u : 16;
  int vel : 16;
};

struct p2VelocityLevelSigmaStr
{
  uint16_t e : 16;
  uint16_t n : 16;
  uint16_t u : 16;
  uint16_t vel : 16;
};

struct p2AccelVehicleStr
{
  int x : 20;
  int y : 20;
  int z : 20;
};

struct p2HeadingPitchRollStr
{
  uint16_t heading : 16;
  int pitch : 16;
  int roll : 16;
};

struct p2HeadingPitchRollSigmaStr
{
  uint16_t heading : 16;
  uint16_t pitch : 16;
  uint16_t roll : 16;
};

struct p2AngRateVehicleStr
{
  int x : 16;
  int y : 16;
  int z : 16;
};

struct p2DataStr
{
  uint32_t gps_week;
  double gps_time;
  float ang_rate_raw_x;
  float ang_rate_raw_y;
  float ang_rate_raw_z;
  float accel_raw_x;
  float accel_raw_y;
  float accel_raw_z;
  uint8_t system_status;
  uint8_t gps_num_status;
  uint8_t satellite_status;
  
  double pos_lat;
  double pos_lon;
  double pos_alt;
  uint16_t pos_e_sigma;
  uint16_t pos_n_sigma;
  uint16_t pos_u_sigma;
  double vel_e;
  double vel_n;
  double vel_u;
  float vel;
  float vel_e_sigma;
  float vel_n_sigma;
  float vel_u_sigma;
  float vel_sigma;
  double accel_vel_x;
  double accel_vel_y;
  double accel_vel_z;
  double heading;
  double pitch;
  double roll;
  float heading_sigma;
  float pitch_sigma;
  float roll_sigma;
  double ang_rate_x;
  double ang_rate_y;
  double ang_rate_z;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  double msec;
};
// p2
/****************vcu309********************/
struct CmdFd_BMS_toVCU_Info
{
  uint8_t Status_BMSModuleApplied;//0 为未投入，1 为投入
  uint8_t Status_BMSModuleAvl;//0 为未投入，1 为投入
};

struct Status_BMSAlarm_toVCU9_Info
{
  uint8_t Alarm_second_BMU : 1;
  uint8_t Alarm_First_BMU  : 1;
  uint8_t Flag_BMS_DischargeAllow  : 1;
  uint8_t Flag_BMS_Charge_RevAllow : 1;
};

struct Status_BMS_toVCU7_Info
{
  uint16_t StatusofSlvBMUDischarRelay1;
  uint16_t StatusofSlvBMUOn1;
  uint16_t StatusofSlvBMUOn2;
};

struct Status_BMS_toVCU1_Info
{
  float CurrentofBMS;
};

struct Cmd_VCUtoMCU_FL_Info
{
  uint8_t Status_EN_Motor : 1;
  uint8_t Mode_Driving_Motor : 5;
  uint8_t Status_PRND : 4;
};

struct TempOfTMS_Info
{
  uint16_t TempOfTMS_;
};

struct TempOfSenor_Info
{
  uint16_t TempOfSensor0;
  uint16_t TempOfSensor1;
  uint16_t TempOfSensor2;
  uint16_t TempOfSensor3;
};

struct FanSpeed_Info
{
  uint8_t FanSpeed1;
  uint8_t FanSpeed2;
  uint8_t FanSpeed3;
  uint8_t FanSpeed4;
};
/****************vcu309********************/

/****************mobileye********************/
// struct LKA_Left_Lane_A_Info
// {
//   uint8_t Model_Degree : 2;
//   uint8_t Quality : 2;
//   uint8_t Lane_Type : 4;
// };

// struct LKA_Left_Lane_A_Info
// {
//   uint8_t Model_Degree : 2;
//   uint8_t Quality : 2;
//   uint8_t Lane_Type : 4;
// };

/****************mobileye********************/

}  // namespace drivers
}  // namespace superg_agv
#endif
