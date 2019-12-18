#ifndef OBJECT_GENERAL_INFO_60B_H_
#define OBJECT_GENERAL_INFO_60B_H_
#include "protocol_data.h"
#include "conti_radar.hpp"

class ObjectGeneralInfo60B
    : public ProtocolData<ContiRadar> {
 public:
  static const uint32_t ID;
  ObjectGeneralInfo60B();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ContiRadar& conti_radar) const override;

 private:
  int object_id(const std::uint8_t* bytes, int32_t length) const;

  double longitude_dist(const std::uint8_t* bytes, int32_t length) const;

  double lateral_dist(const std::uint8_t* bytes, int32_t length) const;

  double longitude_vel(const std::uint8_t* bytes, int32_t length) const;

  double lateral_vel(const std::uint8_t* bytes, int32_t length) const;

  double rcs(const std::uint8_t* bytes, int32_t length) const;

  int dynprop(const std::uint8_t* bytes, int32_t length) const;
};
#endif //OBJECT_GENERAL_INFO_60B_H_
