#ifndef OBJECT_LIST_STATUS_60A_H_
#define OBJECT_LIST_STATUS_60A_H_
#include "protocol_data.h"
#include "conti_radar.hpp"
class ObjectListStatus60A
  : public ProtocolData<ContiRadar>
{
 public:
  static const uint32_t ID;
  ObjectListStatus60A();
    void Parse(const std::uint8_t* bytes, int32_t length,
             ContiRadar& conti_radar) const override;

 private:
  int num_of_objects(const std::uint8_t* bytes, int32_t length) const;

  int meas_counter(const std::uint8_t* bytes, int32_t length) const;

  int interface_version(const std::uint8_t* bytes, int32_t length) const;
};
#endif  // OBJECT_LIST_STATUS_60A_H_
