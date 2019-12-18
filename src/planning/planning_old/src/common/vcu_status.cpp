
#include "common/vcu_status.h"

namespace planning {

//class VCUStatus
// VCUStatus::VCUStatus(uint8_t VEHMode,uint8_t VEHFlt,uint8_t SOC,bool HVStatus,bool estop_status,uint8_t lift_status,uint8_t rolling_counter)
//     :VEHMode_(VEHMode),VEHFlt_(VEHFlt),SOC_(SOC),HVStatus_(HVStatus),estop_status_(lift_status),rolling_counter_(rolling_counter)
VCUStatus::VCUStatus(uint8_t VEHMode,uint8_t VEHFlt,uint8_t SOC,bool HVStatus,bool estop_status,uint8_t lift_status,uint8_t rolling_counter)
    :VEHMode_(VEHMode),VEHFlt_(VEHFlt),SOC_(SOC),HVStatus_(HVStatus),estop_status_(estop_status),lift_status_(lift_status),rolling_counter_(rolling_counter)
 {

 }

 uint8_t VCUStatus::getVEHMode() const
 {
    return VEHMode_;
 }

 uint8_t VCUStatus::getVEHFlt() const
 {
    return VEHFlt_;
 }

 uint8_t VCUStatus::getSOC() const
 {
    return SOC_;
 }

 bool VCUStatus::getHVStatus() const
 {
    return HVStatus_;
 }

 bool VCUStatus::getEstopStatus() const
 {
    return estop_status_;
 }

 uint8_t VCUStatus::getLiftStatus() const
 {
    return lift_status_;
 }

 uint8_t VCUStatus::getRollingCounter() const
 {
    return rolling_counter_;
 }


}//end namespace
