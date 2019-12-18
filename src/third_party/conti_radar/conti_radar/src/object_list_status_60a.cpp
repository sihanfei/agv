#include "object_list_status_60a.h"
#include "glog/logging.h"

#include "byte.h"
#include "canbus_consts.h"

ObjectListStatus60A::ObjectListStatus60A() {}
const uint32_t ObjectListStatus60A::ID = 0x60A;

void ObjectListStatus60A::Parse(const std::uint8_t* bytes, int32_t length,
                                ContiRadar& conti_radar) const {
  conti_radar.object_list_status.nof_objects=num_of_objects(bytes, length);
  conti_radar.object_list_status.meas_counter=meas_counter(bytes, length);
  conti_radar.object_list_status.interface_version=interface_version(bytes, length);
}

int ObjectListStatus60A::num_of_objects(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
int ObjectListStatus60A::meas_counter(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 2);
  uint32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  uint32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

int ObjectListStatus60A::interface_version(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}
