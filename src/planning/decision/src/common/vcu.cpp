
#include "utils.h"

namespace pnc
{


VCUStatus::VCUStatus(uint8_t VEHMode,uint8_t VEHFlt,uint8_t SOC,bool HVStatus,bool estop_status,uint8_t lift_status,uint8_t rolling_counter)
    :control_mode_(VEHMode),fault_status_(VEHFlt),SOC_(SOC),start_status_(HVStatus),estop_status_(estop_status),lift_status_(lift_status),heat_beat_(rolling_counter)
 {

 }

// 获取VCU状态信息
 uint8_t VCUStatus::getControlMode() const
 {
    return control_mode_;
 }

 uint8_t VCUStatus::getFaultStatus() const
 {
    return fault_status_;
 }

 uint8_t VCUStatus::getSOC() const
 {
    return SOC_;
 }

 bool VCUStatus::getStartStatus() const
 {
    return start_status_;
 }

 bool VCUStatus::getEstopStatus() const
 {
    return estop_status_;
 }

 uint8_t VCUStatus::getLiftStatus() const
 {
    return lift_status_;
 }

 uint8_t VCUStatus::getHeatbeat() const
 {
    return heat_beat_;
 }


}//end namespace

