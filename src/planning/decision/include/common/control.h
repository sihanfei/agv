#ifndef CONTROL_H
#define CONTROL_H


namespace pnc 
{

class ControlInfo
{
public:

    ControlInfo() = default;
    ~ControlInfo() = default;

    ControlInfo(uint8_t fault_status, int8_t running_status,double offset_y, double offset_heading, double offset_speed);

    //  uint8_t getWorkMode() const;
    //  uint8_t getLiftStatus() const;
    //  uint8_t getEstopStatus() const;
    //  uint8_t getStartStatus() const;
    //  uint8_t getHandBrakeStatus() const;

    uint8_t getFaultStatus() const;
    int8_t getRunningStatus() const;
    double getOffsetY() const;
    double getOffsetHeading() const;
    double getOffsetSpeed() const;


private:

    uint8_t fault_status_;		// 车辆故障状态（0:正常;1:警告;2:一般故障;3:严重故障）
    int8_t running_status_;		// 描述：运动状态（-1:未知；0:静止；1:直行；2:转弯）
    double offset_y_;					// 单位：m     描述：横向路径偏差,车身中心点与路径最近点的横向偏差
    double offset_heading_;		// 单位：deg   描述：航向角偏差
    double offset_speed_;			// 单位：m/s   描述：速度偏差

};
}
#endif // CONTROL_INFO_H

