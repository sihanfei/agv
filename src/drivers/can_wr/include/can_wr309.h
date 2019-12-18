#ifndef DRIVERS_CAN_WR_INCLUDE_CAN_RW309_H_
#define DRIVERS_CAN_WR_INCLUDE_CAN_RW309_H_

#include "adcuSDK.h"
#include "ros/ros.h"
#include <thread>
#include "device.h"
#include "protocol.h"
// #include <map>

using namespace std;


#define CmdFd_BMS_toVCU 2499740148
#define Status_BMSAlarm_toVCU9 2499742195
#define Status_BMS_toVCU7 2499741683
#define Status_BMS_toVCU1 2499740147
#define Cmd_VCUtoMCU_FL 2364605095
#define FanSpeed 2677603843
#define TempOfTMS 2677603842
#define TempOfSenor 2677603841
#define VECTOR__INDEPENDENT_SIG_MSG 3221225472
#define Rsp_EVC_F 2365460865
#define Rsp_EVC_R 2365461122
#define Cmd_VCU_toEVC_F 2365468934
#define Cmd_VCU_toEVC_R 2365469190
#define Status_MCU_SCSRtoVCU03 2565945072
#define Status_MCU_SCSFtoVCU03 2565944304

#define Status_MCU_LifttoVCU03 2565943536
#define Status_MCU_RRtoVCU03 2565942000
#define Status_MCU_RLtoVCU03 2565941232
#define Status_MCU_FRtoVCU03 2565940464
#define Status_MCU_SCSRtoVCU01 2565944560
#define Status_MCU_SCSFoVCU01 2565943792
#define Status_MCU_LifttoVCU01 2565943024
#define Status_MCU_RRtoVCU01 2565941488
#define Status_MCU_RLtoVCU01 2565940720
#define Status_MCU_FRtoVCU01 2565939952
#define Cmd_VCUtoMCU_SCSR 2364606887
#define Cmd_VCUtoMCU_SCSF 2364606631
#define Cmd_VCUtoMCU_Lift 2364606375
#define Cmd_VCUtoMCU_RL 2364605607
#define Cmd_VCUtoMCU_RR 2364605863
#define Cmd_VCUtoMCU_FR 2364605351

#define Info_CANWifi 2351034374
#define Info_ofDCDC2 2566853368
#define Cmd_VCU_toDCDC2 2349308583
#define Cmd_VCU_toDCDC1 2566857967
#define Info1_DCDC1_toVCU 2566853367
#define CmdRqInfo_VCUtoBMS1 2432631028
#define CmdRsp_BMS_toVCU 2499740146
#define Cmd_VCU_toBMS 2432631026
#define Status_BMS_toVCU8 2499741939
#define Status_BMS_toVCU4 2499740915
#define CmdRqInfo_VCUtoBMS0 2432631027
#define Status_MCU_FLtoVCU03 2565939696
#define Status_MCU_FLtoVCU01 2565939184
#define DTCofTMS         2677603844

namespace superg_agv
{
namespace drivers
{
class CANDrivers309
{
    typedef void (CANDrivers309::*pFun)(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    map< uint32_t, pFun > m_pHandlerMap;
public:
    uint8_t begain_tip[CAN_DATA_SIZE];

private:

public:
    CANDrivers309();
    ~CANDrivers309();
    void InitHandlerMap();
    /********************Parse********************/
    void CmdFd_BMS_toVCUParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Status_BMSAlarm_toVCU9Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Status_BMS_toVCU7Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Status_BMS_toVCU1Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Cmd_VCUtoMCU_FLParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void FanSpeedParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void TempOfTMSParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void TempOfSenorParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void VECTOR__INDEPENDENT_SIG_MSGParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Rsp_EVC_FParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Rsp_EVC_RParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Cmd_VCU_toEVC_FParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Cmd_VCU_toEVC_RParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Status_MCU_SCSRtoVCU03Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Status_MCU_SCSFtoVCU03Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);

    void Status_MCU_LifttoVCU03Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Status_MCU_RRtoVCU03Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Status_MCU_RLtoVCU03Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Status_MCU_FRtoVCU03Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Status_MCU_SCSRtoVCU01Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Status_MCU_SCSFoVCU01Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Status_MCU_LifttoVCU01Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Status_MCU_RRtoVCU01Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Status_MCU_RLtoVCU01Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Status_MCU_FRtoVCU01Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Cmd_VCUtoMCU_SCSRParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Cmd_VCUtoMCU_SCSFParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Cmd_VCUtoMCU_LiftParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Cmd_VCUtoMCU_RLParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Cmd_VCUtoMCU_RRParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Cmd_VCUtoMCU_FRParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    
    void Info_CANWifiParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Info_ofDCDC2Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Cmd_VCU_toDCDC2Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Cmd_VCU_toDCDC1Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Info1_DCDC1_toVCUParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void CmdRqInfo_VCUtoBMS1Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void CmdRsp_BMS_toVCUParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Cmd_VCU_toBMSParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Status_BMS_toVCU8Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Status_BMS_toVCU4Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void CmdRqInfo_VCUtoBMS0Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Status_MCU_FLtoVCU03Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void Status_MCU_FLtoVCU01Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    void DTCofTMSParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type);
    /********************Parse********************/
};
}
}

#endif
