#ifndef CLUSER_QUALITY_INFO_702_H_
#define CLUSER_QUALITY_INFO_702_H_
#include "protocol_data.h"
#include "conti_radar.hpp"


class ClusterQualityInfo702
    : public ProtocolData<ContiRadar> {
 public:
  static const uint32_t ID;
  ClusterQualityInfo702();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ContiRadar& conti_radar) const override;

 private:
  int target_id(const std::uint8_t* bytes, int32_t length) const;

  int longitude_dist_rms(const std::uint8_t* bytes, int32_t length) const;

  int lateral_dist_rms(const std::uint8_t* bytes, int32_t length) const;

  int longitude_vel_rms(const std::uint8_t* bytes, int32_t length) const;

  int pdh0(const std::uint8_t* bytes, int32_t length) const;

  int ambig_state(const std::uint8_t* bytes, int32_t length) const;

  int invalid_state(const std::uint8_t* bytes, int32_t length) const;

  int lateral_vel_rms(const std::uint8_t* bytes, int32_t length) const;
};
#endif  // CLUSER_QUALITY_INFO_702_H_
