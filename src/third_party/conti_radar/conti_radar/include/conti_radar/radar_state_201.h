#ifndef RADAR_STATE_201_H_
#define RADAR_STATE_201_H_
#include "protocol_data.h"
#include "conti_radar.hpp"

class RadarState201 : public ProtocolData<ContiRadar> {
 public:
  static const uint32_t ID;
  RadarState201();
  void Parse(const std::uint8_t* bytes, int32_t length,
            ContiRadar &conti_radar) const override;
 private:
  int max_dist(const std::uint8_t* bytes, int32_t length) const;

  int radar_power(const std::uint8_t* bytes, int32_t length) const;

  OutputType output_type(const std::uint8_t* bytes, int32_t length) const;

  RcsThreshold rcs_threshold(const std::uint8_t* bytes, int32_t length) const;

  bool send_quality(const std::uint8_t* bytes, int32_t length) const;

  bool send_ext_info(const std::uint8_t* bytes, int32_t length) const;
};

#endif  // RADAR_STATE_201_H_

