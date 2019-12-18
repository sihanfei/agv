#ifndef CONTROL_UTILS_H_
#define CONTROL_UTILS_H_

#include "ros/ros.h"
#include <math.h>

namespace control
{
struct PIDConf
{
  double kp;
  double ki;
  double kd;
  double kaw;
  double integrator_saturation_level;
  double output_saturation_level;
  bool integrator_enabled;
};

struct positionConf
{
  u_int32_t n_gps_sequence_num;
  double x;
  double y;
  double z;
  double lon;
  double lat;
  double height;
  double velocity_x; // velocity east
  double velocity_y; // velocity north
  double velocity_z; // velocity up
  double heading;
  double pitch;
  double roll;
  double gps_seconds;
  double velocity;
  double dist; //里程，供saveroutepoint模式使用
};

struct PIDTemp
{
  double station_error;
  double speed_error;
  double latteral_offset_preview;
  float gps_seconds;
};

} // end namespace control
#endif