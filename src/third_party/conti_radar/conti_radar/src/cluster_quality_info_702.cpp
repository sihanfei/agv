#include "cluster_quality_info_702.h"
#include "const_vars.h"

#include "glog/logging.h"

#include "byte.h"
#include "canbus_consts.h"


ClusterQualityInfo702::ClusterQualityInfo702() {}
const uint32_t ClusterQualityInfo702::ID = 0x702;

void ClusterQualityInfo702::Parse(const std::uint8_t* bytes, int32_t length,
                                  ContiRadar &conti_radar) const
{
  int id = target_id(bytes, length);
  for (int i = 0; i < conti_radar.conti_radar_obs.size(); ++i)
    {
    if (conti_radar.conti_radar_obs[i].obstacle_id == id)
    {
      auto conti_obs = conti_radar.conti_radar_obs[i];
      conti_obs.longitude_dist_rms=LINEAR_RMS[longitude_dist_rms(bytes, length)];
      conti_obs.lateral_dist_rms=LINEAR_RMS[lateral_dist_rms(bytes, length)];
      conti_obs.longitude_vel_rms=LINEAR_RMS[longitude_vel_rms(bytes, length)];
      conti_obs.lateral_vel_rms=LINEAR_RMS[lateral_vel_rms(bytes, length)];
      conti_obs.probexist=PROBOFEXIST[pdh0(bytes, length)];
      switch (invalid_state(bytes, length)) {
        case 0x01:
        case 0x02:
        case 0x03:
        case 0x06:
        case 0x07:
        case 0x0E:
          conti_obs.probexist=PROBOFEXIST[0];
        default:
          break;
      }
      switch (ambig_state(bytes, length)) {
        case 0x00:
        case 0x01:
        case 0x02:
          conti_obs.probexist=PROBOFEXIST[0];
        default:
          break;
      }
    }
  }
}

int ClusterQualityInfo702::target_id(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int ClusterQualityInfo702::longitude_dist_rms(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 5);

  int ret = x;
  return ret;
}

int ClusterQualityInfo702::lateral_dist_rms(const std::uint8_t* bytes,
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

int ClusterQualityInfo702::longitude_vel_rms(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(1, 5);

  int ret = x;
  return ret;
}

int ClusterQualityInfo702::pdh0(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 3);

  int ret = x;
  return ret;
}

int ClusterQualityInfo702::ambig_state(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 3);

  int ret = x;
  return ret;
}

int ClusterQualityInfo702::invalid_state(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(3, 5);

  int ret = x;
  return ret;
}

int ClusterQualityInfo702::lateral_vel_rms(const std::uint8_t* bytes,
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
