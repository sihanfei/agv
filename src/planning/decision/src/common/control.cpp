#include "utils.h"

namespace pnc
{


ControlInfo::ControlInfo(uint8_t fault_status, int8_t running_status, double offset_y, double offset_heading, double offset_speed)
    :fault_status_(fault_status),running_status_(running_status),offset_y_(offset_y),offset_heading_(offset_heading),offset_speed_(offset_speed)
{

}

//  uint8_t ControlInfo::getWorkMode() const
//  {
//    return work_mode_;
//  }

//  uint8_t ControlInfo::getLiftStatus() const
//  {
//    return lift_status_;
//  }

//  uint8_t ControlInfo::getEstopStatus() const
//  {
//    return estop_status_;
//  }

//  uint8_t ControlInfo::getStartStatus() const
//  {
//    return start_status_;
//  }

//  uint8_t ControlInfo::getHandBrakeStatus() const
//  {
//    return hand_brake_status_;
//  }

uint8_t ControlInfo::getFaultStatus() const
{
    return fault_status_;
}

int8_t ControlInfo::getRunningStatus() const
{
    return running_status_;
}

double ControlInfo::getOffsetY() const
{
    return offset_y_;
}

double ControlInfo::getOffsetHeading() const
{
    return offset_heading_;
}

double ControlInfo::getOffsetSpeed() const
{
    return offset_speed_;
}

}//end namespace

