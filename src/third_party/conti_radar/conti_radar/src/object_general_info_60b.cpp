#include "object_general_info_60b.h"

#include "glog/logging.h"
//#include "time.h"
#include "byte.h"
#include "canbus_consts.h"
#include "const_vars.h"

ObjectGeneralInfo60B::ObjectGeneralInfo60B() {}
const uint32_t ObjectGeneralInfo60B::ID = 0x60B;

void ObjectGeneralInfo60B::Parse(const std::uint8_t* bytes, int32_t length,
                                 ContiRadar& conti_radar) const {

  struct ContiRadarObs  conti_radar_obj;
  int obj_id = object_id(bytes, length);
  conti_radar_obj.clusterortrack=false;
  conti_radar_obj.obstacle_id=obj_id;
  conti_radar_obj.longitude_dist=longitude_dist(bytes, length);
  conti_radar_obj.lateral_dist=lateral_dist(bytes, length);
  conti_radar_obj.longitude_vel=longitude_vel(bytes, length);
  conti_radar_obj.lateral_vel=lateral_vel(bytes, length);
  conti_radar_obj.rcs=rcs(bytes, length);
  conti_radar_obj.dynprop=dynprop(bytes, length);
  conti_radar.conti_radar_obs.push_back(conti_radar_obj);
  //double timestamp = apollo::common::time::Clock::NowInSeconds();
 // auto header = conti_obs->mutable_header();
 // header->CopyFrom(conti_radar->header());
 // header->set_timestamp_sec(timestamp);
}

int ObjectGeneralInfo60B::object_id(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

double ObjectGeneralInfo60B::longitude_dist(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(3, 5);

  x <<= 5;
  x |= t;

  double ret = x * OBJECT_DIST_RES + OBJECT_DIST_LONG_MIN;
  return ret;
}

double ObjectGeneralInfo60B::lateral_dist(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 3);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);

  x <<= 8;
  x |= t;

  double ret = x * OBJECT_DIST_RES + OBJECT_DIST_LAT_MIN;
  return ret;
}

double ObjectGeneralInfo60B::longitude_vel(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);
  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(6, 2);

  x <<= 2;
  x |= t;
  double ret = x * OBJECT_VREL_RES + OBJECT_VREL_LONG_MIN;
  return ret;
}

double ObjectGeneralInfo60B::lateral_vel(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 6);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(5, 3);

  x <<= 3;
  x |= t;

  double ret = x * OBJECT_VREL_RES + OBJECT_VREL_LAT_MIN;
  return ret;
}

double ObjectGeneralInfo60B::rcs(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * OBJECT_RCS_RES + OBJECT_RCS_MIN;
  return ret;
}

int ObjectGeneralInfo60B::dynprop(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 3);

  int ret = x;
  return ret;
}
