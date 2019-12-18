#ifndef OBJECT_EXTENDED_INFO_60D_H_
#define OBJECT_EXTENDED_INFO_60D_H_
#include "protocol_data.h"
#include "conti_radar.hpp"

class ObjectExtendedInfo60D
    : public ProtocolData<ContiRadar> {
 public:
  static const uint32_t ID;
  ObjectExtendedInfo60D();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ContiRadar& conti_radar) const override;

 private:

  int object_id(const std::uint8_t* bytes, int32_t length) const;

  double longitude_accel(const std::uint8_t* bytes, int32_t length) const;

  double lateral_accel(const std::uint8_t* bytes, int32_t length) const;

  int obstacle_class(const std::uint8_t* bytes, int32_t length) const;

  double oritation_angle(const std::uint8_t* bytes, int32_t length) const;

  double object_length(const std::uint8_t* bytes, int32_t length) const;

  double object_width(const std::uint8_t* bytes, int32_t length) const;
};
#endif  // MODULES_CANBUS_VEHICL_ESR_PROTOCOL_OBJECT_EXTENDED_INFO_60D_H_
