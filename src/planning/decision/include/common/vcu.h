#ifndef VCU_STATUS_H
#define VCU_STATUS_H



namespace pnc 
{

class VCUStatus
{
public:
    VCUStatus()=default;
    ~VCUStatus()=default;


//    bool EPBStatus
//    单位：num，描述：手刹状态，0：闭合（制动），1：松开
//    uint8 Dir_PRND
//    单位：num，描述：档位状态，1：前进档，2：停车档，3：空档，4：后退档


    VCUStatus(uint8_t VEHMode,uint8_t VEHFlt,uint8_t SOC,bool HVStatus,bool estop_status,uint8_t lift_status,uint8_t rolling_counter);

    uint8_t getControlMode() const;
    uint8_t getFaultStatus() const;
    uint8_t getSOC() const;
    bool getStartStatus() const;
    bool getEstopStatus() const;
    uint8_t getLiftStatus() const;
    uint8_t getHeatbeat() const;

private:

    uint8_t control_mode_;		//AGV当前所在的模式，0：故障，1：调试，2：自动，3：手动（遥控）
    uint8_t fault_status_;		//故障状态，0：无故障，1：一级故障，2：二级故障，3：三级故障
    uint8_t SOC_;							//动力电池剩余电量，0：满电，1：小于75%，2：小于50%，3：小于25%
    bool start_status_;						//高压状态，0：非启动，1：启动
    bool estop_status_;				//
    uint8_t lift_status_;			//举升状态，0：未知，1：上升到位，2：下降到位，3：运动中
    uint8_t heat_beat_;	//vcu反馈的的心跳值

};
}
#endif // VCU_STATUS_H

