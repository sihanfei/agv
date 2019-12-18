#include "can_wr309.h"

using namespace std;
using namespace superg_agv;
using namespace drivers;

namespace superg_agv
{
namespace drivers
{
CANDrivers309::CANDrivers309()
{
    InitHandlerMap();
}
CANDrivers309::~CANDrivers309()
{

}
void CANDrivers309::InitHandlerMap()
{
    m_pHandlerMap.insert(make_pair(CmdFd_BMS_toVCU, &CANDrivers309::CmdFd_BMS_toVCUParse));
    m_pHandlerMap.insert(make_pair(Status_BMSAlarm_toVCU9, &CANDrivers309::Status_BMSAlarm_toVCU9Parse));
    m_pHandlerMap.insert(make_pair(Status_BMS_toVCU7, &CANDrivers309::Status_BMS_toVCU7Parse));
    m_pHandlerMap.insert(make_pair(Status_BMS_toVCU1, &CANDrivers309::Status_BMS_toVCU1Parse));
    m_pHandlerMap.insert(make_pair(Cmd_VCUtoMCU_FL, &CANDrivers309::Cmd_VCUtoMCU_FLParse));
    m_pHandlerMap.insert(make_pair(FanSpeed, &CANDrivers309::FanSpeedParse));
    m_pHandlerMap.insert(make_pair(TempOfTMS, &CANDrivers309::TempOfTMSParse));
    m_pHandlerMap.insert(make_pair(TempOfSenor, &CANDrivers309::TempOfSenorParse));
    m_pHandlerMap.insert(make_pair(VECTOR__INDEPENDENT_SIG_MSG, &CANDrivers309::VECTOR__INDEPENDENT_SIG_MSGParse));
    m_pHandlerMap.insert(make_pair(Rsp_EVC_F, &CANDrivers309::Rsp_EVC_FParse));
    m_pHandlerMap.insert(make_pair(Rsp_EVC_R, &CANDrivers309::Rsp_EVC_RParse));
    m_pHandlerMap.insert(make_pair(Cmd_VCU_toEVC_F, &CANDrivers309::Cmd_VCU_toEVC_FParse));
    m_pHandlerMap.insert(make_pair(Cmd_VCU_toEVC_R, &CANDrivers309::Cmd_VCU_toEVC_RParse));
    m_pHandlerMap.insert(make_pair(Status_MCU_SCSRtoVCU03, &CANDrivers309::Status_MCU_SCSRtoVCU03Parse));
    m_pHandlerMap.insert(make_pair(Status_MCU_SCSFtoVCU03, &CANDrivers309::Status_MCU_SCSFtoVCU03Parse));

    m_pHandlerMap.insert(make_pair(Status_MCU_LifttoVCU03, &CANDrivers309::Status_MCU_LifttoVCU03Parse));
    m_pHandlerMap.insert(make_pair(Status_MCU_RRtoVCU03, &CANDrivers309::Status_MCU_RRtoVCU03Parse));
    m_pHandlerMap.insert(make_pair(Status_MCU_RLtoVCU03, &CANDrivers309::Status_MCU_RLtoVCU03Parse));
    m_pHandlerMap.insert(make_pair(Status_MCU_FRtoVCU03, &CANDrivers309::Status_MCU_FRtoVCU03Parse));
    m_pHandlerMap.insert(make_pair(Status_MCU_SCSRtoVCU01, &CANDrivers309::Status_MCU_SCSRtoVCU01Parse));
    m_pHandlerMap.insert(make_pair(Status_MCU_SCSFoVCU01, &CANDrivers309::Status_MCU_SCSFoVCU01Parse));
    m_pHandlerMap.insert(make_pair(Status_MCU_LifttoVCU01, &CANDrivers309::Status_MCU_LifttoVCU01Parse));
    m_pHandlerMap.insert(make_pair(Status_MCU_RRtoVCU01, &CANDrivers309::Status_MCU_RRtoVCU01Parse));
    m_pHandlerMap.insert(make_pair(Status_MCU_RLtoVCU01, &CANDrivers309::Status_MCU_RLtoVCU01Parse));
    m_pHandlerMap.insert(make_pair(Status_MCU_FRtoVCU01, &CANDrivers309::Status_MCU_FRtoVCU01Parse));
    m_pHandlerMap.insert(make_pair(Cmd_VCUtoMCU_SCSR, &CANDrivers309::Cmd_VCUtoMCU_SCSRParse));
    m_pHandlerMap.insert(make_pair(Cmd_VCUtoMCU_SCSF, &CANDrivers309::Cmd_VCUtoMCU_SCSFParse));
    m_pHandlerMap.insert(make_pair(Cmd_VCUtoMCU_Lift, &CANDrivers309::Cmd_VCUtoMCU_LiftParse));
    m_pHandlerMap.insert(make_pair(Cmd_VCUtoMCU_RL, &CANDrivers309::Cmd_VCUtoMCU_RLParse));
    m_pHandlerMap.insert(make_pair(Cmd_VCUtoMCU_RR, &CANDrivers309::Cmd_VCUtoMCU_RRParse));
    m_pHandlerMap.insert(make_pair(Cmd_VCUtoMCU_FR, &CANDrivers309::Cmd_VCUtoMCU_FRParse));

    m_pHandlerMap.insert(make_pair(Info_CANWifi, &CANDrivers309::Info_CANWifiParse));
    m_pHandlerMap.insert(make_pair(Info_ofDCDC2, &CANDrivers309::Info_ofDCDC2Parse));
    m_pHandlerMap.insert(make_pair(Cmd_VCU_toDCDC2, &CANDrivers309::Cmd_VCU_toDCDC2Parse));
    m_pHandlerMap.insert(make_pair(Cmd_VCU_toDCDC1, &CANDrivers309::Cmd_VCU_toDCDC1Parse));
    m_pHandlerMap.insert(make_pair(Info1_DCDC1_toVCU, &CANDrivers309::Info1_DCDC1_toVCUParse));
    m_pHandlerMap.insert(make_pair(CmdRqInfo_VCUtoBMS1, &CANDrivers309::CmdRqInfo_VCUtoBMS1Parse));
    m_pHandlerMap.insert(make_pair(CmdRsp_BMS_toVCU, &CANDrivers309::CmdRsp_BMS_toVCUParse));
    m_pHandlerMap.insert(make_pair(Cmd_VCU_toBMS, &CANDrivers309::Cmd_VCU_toBMSParse));
    m_pHandlerMap.insert(make_pair(Status_BMS_toVCU8, &CANDrivers309::Status_BMS_toVCU8Parse));
    m_pHandlerMap.insert(make_pair(Status_BMS_toVCU4, &CANDrivers309::Status_BMS_toVCU4Parse));
    m_pHandlerMap.insert(make_pair(CmdRqInfo_VCUtoBMS0, &CANDrivers309::CmdRqInfo_VCUtoBMS0Parse));
    m_pHandlerMap.insert(make_pair(Status_MCU_FLtoVCU03, &CANDrivers309::Status_MCU_FLtoVCU03Parse));
    m_pHandlerMap.insert(make_pair(Status_MCU_FLtoVCU01, &CANDrivers309::Status_MCU_FLtoVCU01Parse));
    m_pHandlerMap.insert(make_pair(DTCofTMS, &CANDrivers309::DTCofTMSParse));
    map< uint32_t, pFun >::iterator it; 
    for (it = m_pHandlerMap.begin(); it != m_pHandlerMap.end(); ++it)
    {
        ROS_INFO("key %u", it->first);
    }
}

void CANDrivers309::CmdFd_BMS_toVCUParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ROS_INFO("recv id: %d,CmdFd_BMS_toVCUParse",can_id);
    CmdFd_BMS_toVCU_Info CmdFd_BMS_toVCU_Info_;
    CmdFd_BMS_toVCU_Info_.Status_BMSModuleApplied = can_buf.can_data[1];
    CmdFd_BMS_toVCU_Info_.Status_BMSModuleAvl     = can_buf.can_data[3];
    ROS_INFO("Status_BMSModuleApplied: %d,Status_BMSModuleAvl: %d",CmdFd_BMS_toVCU_Info_.Status_BMSModuleApplied,CmdFd_BMS_toVCU_Info_.Status_BMSModuleAvl);
}

void CANDrivers309::Status_BMSAlarm_toVCU9Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ROS_INFO("recv id: %d,Status_BMSAlarm_toVCU9Parse",can_id);
    Status_BMSAlarm_toVCU9_Info Status_BMSAlarm_toVCU9_Info_;
    Status_BMSAlarm_toVCU9_Info_.Alarm_second_BMU         = (can_buf.can_data[0] & 0x20) >> 5;
    Status_BMSAlarm_toVCU9_Info_.Alarm_First_BMU          = (can_buf.can_data[0] & 0x10) >> 4;
    Status_BMSAlarm_toVCU9_Info_.Flag_BMS_DischargeAllow  = (can_buf.can_data[0] & 0x02) >> 1;
    Status_BMSAlarm_toVCU9_Info_.Flag_BMS_Charge_RevAllow = can_buf.can_data[0] & 0x01;
    ROS_INFO("Alarm_second_BMU: %d,Alarm_First_BMU: %d,Flag_BMS_DischargeAllow: %d,Flag_BMS_Charge_RevAllow: %d",Status_BMSAlarm_toVCU9_Info_.Alarm_second_BMU,
        Status_BMSAlarm_toVCU9_Info_.Alarm_First_BMU,Status_BMSAlarm_toVCU9_Info_.Flag_BMS_DischargeAllow,Status_BMSAlarm_toVCU9_Info_.Flag_BMS_Charge_RevAllow);
}

void CANDrivers309::Status_BMS_toVCU7Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ROS_INFO("recv id: %d,Status_BMS_toVCU7Parse",can_id);
    Status_BMS_toVCU7_Info Status_BMS_toVCU7_Info_;
    Status_BMS_toVCU7_Info_.StatusofSlvBMUDischarRelay1 = can_buf.can_data[4] >> 7  | can_buf.can_data[5] << 1  | (can_buf.can_data[6] & 0x7F) << 8;
    Status_BMS_toVCU7_Info_.StatusofSlvBMUOn1           = can_buf.can_data[2] >> 7  | can_buf.can_data[3] << 1  | (can_buf.can_data[4] & 0x7F) << 8;
    Status_BMS_toVCU7_Info_.StatusofSlvBMUOn2           = can_buf.can_data[0] >> 7  | can_buf.can_data[1] << 1  | (can_buf.can_data[2] & 0x7F) << 8;
    ROS_INFO("StatusofSlvBMUDischarRelay1: %d,StatusofSlvBMUOn1: %d,StatusofSlvBMUOn2: %d",Status_BMS_toVCU7_Info_.StatusofSlvBMUDischarRelay1,
             Status_BMS_toVCU7_Info_.StatusofSlvBMUOn1,Status_BMS_toVCU7_Info_.StatusofSlvBMUOn2);
}

void CANDrivers309::Status_BMS_toVCU1Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ROS_INFO("recv id: %d,Status_BMS_toVCU1Parse",can_id);
    Status_BMS_toVCU1_Info Status_BMS_toVCU1_Info_;
    Status_BMS_toVCU1_Info_.CurrentofBMS = (can_buf.can_data[4] >> 7  | can_buf.can_data[5] << 1  | (can_buf.can_data[6] & 0x7F) << 8) * 0.1;
    ROS_INFO("CurrentofBMS: %f",Status_BMS_toVCU1_Info_.CurrentofBMS);
}

void CANDrivers309::Cmd_VCUtoMCU_FLParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ROS_INFO("recv id: %d,Cmd_VCUtoMCU_FLParse",can_id);
    Cmd_VCUtoMCU_FL_Info Cmd_VCUtoMCU_FL_Info_;
    Cmd_VCUtoMCU_FL_Info_.Status_EN_Motor    = (can_buf.can_data[4] & 0x20) >> 5;
    Cmd_VCUtoMCU_FL_Info_.Mode_Driving_Motor = can_buf.can_data[4] & 0x01;
    Cmd_VCUtoMCU_FL_Info_.Status_PRND        = (can_buf.can_data[4] & 0x3C) >> 2;
    ROS_INFO("Mode_Driving_Motor: %d,Status_PRND: %d,Status_EN_Motor: %d",Cmd_VCUtoMCU_FL_Info_.Mode_Driving_Motor,
             Cmd_VCUtoMCU_FL_Info_.Status_PRND,Cmd_VCUtoMCU_FL_Info_.Status_EN_Motor);
}

void CANDrivers309::FanSpeedParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ROS_INFO("recv id: %d,FanSpeedParse",can_id);
    FanSpeed_Info FanSpeed_Info_;
    FanSpeed_Info_.FanSpeed1 = (can_buf.can_data[1] << 8 | can_buf.can_data[0]) * 0.1;
    FanSpeed_Info_.FanSpeed2 = (can_buf.can_data[3] << 8 | can_buf.can_data[2]) * 0.1;
    FanSpeed_Info_.FanSpeed3 = (can_buf.can_data[5] << 8 | can_buf.can_data[4]) * 0.1;
    FanSpeed_Info_.FanSpeed4 = (can_buf.can_data[7] << 8 | can_buf.can_data[6]) * 0.1;
    // ROS_INFO("TempOfSensor0: %d",TempOfTMS_Info_.TempOfTMS);
}

void CANDrivers309::TempOfTMSParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ROS_INFO("recv id: %d,TempOfTMSParse",can_id);
    TempOfTMS_Info TempOfTMS_Info_;
    TempOfTMS_Info_.TempOfTMS_ = can_buf.can_data[5] << 8 | can_buf.can_data[4];
    ROS_INFO("TempOfTMS: %d",TempOfTMS_Info_.TempOfTMS_);
}

void CANDrivers309::TempOfSenorParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ROS_INFO("recv id: %d,TempOfSenorParse",can_id);
    TempOfSenor_Info TempOfSenor_Info_;
    // TempOfSenor_Info_.TempOfSensor0    = (can_buf.can_data[1] << 8 | can_buf.can_data[0]) * 0.1;
    // TempOfSenor_Info_.TempOfSensor1    = (can_buf.can_data[3] << 8 | can_buf.can_data[2]) * 0.1;
    // TempOfSenor_Info_.TempOfSensor2    = (can_buf.can_data[5] << 8 | can_buf.can_data[4]) * 0.1;
    // TempOfSenor_Info_.TempOfSensor3    = (can_buf.can_data[7] << 8 | can_buf.can_data[6]) * 0.1;
    // ROS_INFO("TempOfSensor0: %d",TempOfSenor_Info_.TempOfTMS);
}

void CANDrivers309::VECTOR__INDEPENDENT_SIG_MSGParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::Rsp_EVC_FParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::Rsp_EVC_RParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::Cmd_VCU_toEVC_FParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::Cmd_VCU_toEVC_RParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::Status_MCU_SCSRtoVCU03Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::Status_MCU_SCSFtoVCU03Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}


void CANDrivers309::Status_MCU_LifttoVCU03Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::Status_MCU_RRtoVCU03Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::Status_MCU_RLtoVCU03Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::Status_MCU_FRtoVCU03Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::Status_MCU_SCSRtoVCU01Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::Status_MCU_SCSFoVCU01Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::Status_MCU_LifttoVCU01Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::Status_MCU_RRtoVCU01Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::Status_MCU_RLtoVCU01Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::Status_MCU_FRtoVCU01Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::Cmd_VCUtoMCU_SCSRParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::Cmd_VCUtoMCU_SCSFParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::Cmd_VCUtoMCU_LiftParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::Cmd_VCUtoMCU_RLParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::Cmd_VCUtoMCU_RRParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::Cmd_VCUtoMCU_FRParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
    ;
}

void CANDrivers309::CANDrivers309::Info_CANWifiParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
;
}

void CANDrivers309::Info_ofDCDC2Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
;
}

void CANDrivers309::Cmd_VCU_toDCDC2Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
;
}

void CANDrivers309::Cmd_VCU_toDCDC1Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
;
}

void CANDrivers309::Info1_DCDC1_toVCUParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
;
}

void CANDrivers309::CmdRqInfo_VCUtoBMS1Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
;
}

void CANDrivers309::CmdRsp_BMS_toVCUParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
;
}

void CANDrivers309::Cmd_VCU_toBMSParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
;
}

void CANDrivers309::Status_BMS_toVCU8Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
;
}

void CANDrivers309::Status_BMS_toVCU4Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
;
}

void CANDrivers309::CmdRqInfo_VCUtoBMS0Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
;
}

void CANDrivers309::Status_MCU_FLtoVCU03Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
;
}

void CANDrivers309::Status_MCU_FLtoVCU01Parse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
;
}

void CANDrivers309::DTCofTMSParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &wr_type)
{
;
}
}
}