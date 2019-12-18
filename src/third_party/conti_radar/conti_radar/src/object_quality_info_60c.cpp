#include "object_quality_info_60c.h"
#include "const_vars.h"
#include "glog/logging.h"
#include "byte.h"
#include "canbus_consts.h"


ObjectQualityInfo60C::ObjectQualityInfo60C() {}
const uint32_t ObjectQualityInfo60C::ID = 0x60C;

void ObjectQualityInfo60C::Parse(const std::uint8_t* bytes, int32_t length,
                                 ContiRadar& conti_radar) const {
  int obj_id = object_id(bytes, length);

  for (int i = 0; i < conti_radar.conti_radar_obs.size(); ++i) {
    if (conti_radar.conti_radar_obs[i].obstacle_id == obj_id) {
     // auto obs = conti_radar->conti_radar_obs[i];
      conti_radar.conti_radar_obs[i].longitude_dist_rms=LINEAR_RMS[longitude_dist_rms(bytes, length)];
      conti_radar.conti_radar_obs[i].lateral_dist_rms=LINEAR_RMS[lateral_dist_rms(bytes, length)];
      conti_radar.conti_radar_obs[i].longitude_vel_rms=LINEAR_RMS[longitude_vel_rms(bytes, length)];
      conti_radar.conti_radar_obs[i].lateral_vel_rms=LINEAR_RMS[lateral_vel_rms(bytes, length)];
      conti_radar.conti_radar_obs[i].longitude_accel_rms=LINEAR_RMS[longitude_accel_rms(bytes, length)];
      conti_radar.conti_radar_obs[i].lateral_accel_rms=LINEAR_RMS[lateral_accel_rms(bytes, length)];
      conti_radar.conti_radar_obs[i].oritation_angle_rms=ANGLE_RMS[oritation_angle_rms(bytes, length)];
      conti_radar.conti_radar_obs[i].probexist=PROBOFEXIST[probexist(bytes, length)];
      conti_radar.conti_radar_obs[i].meas_state=meas_state(bytes, length);
      break;
    }
  }
}

int ObjectQualityInfo60C::object_id(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int ObjectQualityInfo60C::longitude_dist_rms(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 5);

  int ret = x;
  return ret;
}

int ObjectQualityInfo60C::lateral_dist_rms(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 3);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(6, 2);

  x <<= 2;
  x |= t;

  int ret = x;
  return ret;
}

int ObjectQualityInfo60C::longitude_vel_rms(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(1, 5);

  int ret = x;
  return ret;
}

int ObjectQualityInfo60C::lateral_vel_rms(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 1);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(4, 4);

  x <<= 4;
  x |= t;

  int ret = x;
  return ret;
}

int ObjectQualityInfo60C::longitude_accel_rms(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(7, 1);

  x <<= 1;
  x |= t;

  int ret = x;
  return ret;
}

int ObjectQualityInfo60C::lateral_accel_rms(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 5);

  int ret = x;
  return ret;
}

int ObjectQualityInfo60C::oritation_angle_rms(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 2);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(5, 3);

  x <<= 3;
  x |= t;

  int ret = x;
  return ret;
}

int ObjectQualityInfo60C::probexist(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(5, 3);

  int ret = x;
  return ret;
}

int ObjectQualityInfo60C::meas_state(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(2, 3);

  int ret = x;
  return ret;
}
