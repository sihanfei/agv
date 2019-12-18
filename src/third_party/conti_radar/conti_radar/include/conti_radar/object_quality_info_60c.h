#ifndef OBJECT_QUALITY_INFO_60C_H_
#define OBJECT_QUALITY_INFO_60C_H_
#include "protocol_data.h"
#include "conti_radar.hpp"

class ObjectQualityInfo60C
    : public ProtocolData<ContiRadar> {
 public:
  static const uint32_t ID;
  ObjectQualityInfo60C();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ContiRadar&conti_radar) const override;

 private:
  int object_id(const std::uint8_t* bytes, int32_t length) const;

  int longitude_dist_rms(const std::uint8_t* bytes, int32_t length) const;

  int lateral_dist_rms(const std::uint8_t* bytes, int32_t length) const;

  int longitude_vel_rms(const std::uint8_t* bytes, int32_t length) const;

  int lateral_vel_rms(const std::uint8_t* bytes, int32_t length) const;

  int longitude_accel_rms(const std::uint8_t* bytes, int32_t length) const;

  int lateral_accel_rms(const std::uint8_t* bytes, int32_t length) const;

  int oritation_angle_rms(const std::uint8_t* bytes, int32_t length) const;

  int probexist(const std::uint8_t* bytes, int32_t length) const;

  int meas_state(const std::uint8_t* bytes, int32_t length) const;
};

#endif  // OBJECT_QUALITY_INFO_60C_H_
