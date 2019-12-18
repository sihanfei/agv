#ifndef VCU_STATUS_H
#define VCU_STATUS_H

#include "common/utils.h"

namespace planning {

class VCUStatus
{
public:
    VCUStatus()=default;
    ~VCUStatus()=default;

//    uint8 VEHMode
//    单位：num，描述：AGV当前所在的模式，0：故障，1：调试，2：自动，3：手动（遥控）
//    uint8 VEHFlt
//    单位：num，描述：故障状态，0：无故障，1：一级故障，2：二级故障，3：三级故障
//    uint8 SOC
//    单位：num，描述：动力电池剩余电量，0：满电，1：小于75%，2：小于50%，3：小于25%
//    bool HVStatus
//    单位：num，描述：高压状态，0：非启动，1：启动
//    uint8 LiftStatus
//    单位：num，描述：举升状态，0：未知，1：上升到位，2：下降到位，3：运动中
//    bool EPBStatus
//    单位：num，描述：手刹状态，0：闭合（制动），1：松开
//    uint8 Dir_PRND
//    单位：num，描述：档位状态，1：前进档，2：停车档，3：空档，4：后退档
//    uint8 Rolling_Counter
//    单位：num，描述：vcu反馈的的心跳值

    VCUStatus(uint8_t VEHMode,uint8_t VEHFlt,uint8_t SOC,bool HVStatus,bool estop_status,uint8_t lift_status,uint8_t rolling_counter);

    uint8_t getVEHMode() const;

    uint8_t getVEHFlt() const;

    uint8_t getSOC() const;

    bool getHVStatus() const;
    bool getEstopStatus() const;

    uint8_t getLiftStatus() const;

    uint8_t getRollingCounter() const;

private:

    uint8_t VEHMode_;
    uint8_t VEHFlt_;
    uint8_t SOC_;
    bool HVStatus_;

    bool estop_status_;

    uint8_t lift_status_;

    uint8_t rolling_counter_;

};
}
#endif // VCU_STATUS_H
