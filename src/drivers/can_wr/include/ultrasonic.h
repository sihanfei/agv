#ifndef DRIVERS_CAN_WR_INCLUDE_SENSOR_H_
#define DRIVERS_CAN_WR_INCLUDE_SENSOR_H_

#include "can_rw.h"
#include "driver_monitor.h"
#include "logdata.h"
#include <Eigen/Dense>
#include "power_control_msgs/PowerControlCmd.h"

namespace superg_agv
{
namespace drivers
{

#define PI 3.1415926
#define D_DISTANCE 0.05

struct TransformationInfo
{
    double rotate;
    double radian;
    double dx;
    double dy;
};

struct XY
{
    double x;
    double y;
};

class Ultra2Car
{
    public:
        Ultra2Car();
        ~Ultra2Car();
        double angle_to_radian(double degree);
        void Ultra2Car_m(float distance,uint8_t num);
        XY get_xy(uint8_t index);
        uint32_t toCar_num(uint32_t num);
    private:
        
        TransformationInfo transformation_info[32];
        Eigen::Matrix3d rotate_m;
        Eigen::Matrix3d pan_m;
        Eigen::Matrix3d tarns_m[32];
        Eigen::Matrix<double, 1, 3> ultra_m;
        Eigen::Matrix<double, 1, 3> car_m;
        XY xy[2];
};

class ULTRASONIC:private CANDrivers,public Driver_Monitor,public Ultra2Car
{
    // typedef void (SENSOR::*pFun)(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
    typedef void (ULTRASONIC::*pFun)(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
    map< uint32_t, pFun > m_pHandlerMap;

    enum 
    {
        Allstart_Ultrasonic,
        Getstatus_Ultrasonic,
        Waitstatus_Ultrasonic,
        Work_Ultrasonic,
        Error_Ultrasonic
    }Ultrasonic_stat;

    private:
        int devid;
        std::mutex mtx_ultrasonic;
        std::mutex mtx_ultrasonic_data;
        std::condition_variable cond_ultrasonic;
        ultrasonicDataStr ultrasonicData;
        ultrasonicDataStr ultrasonicData_ready;
    public:
        ULTRASONIC();
        ~ULTRASONIC();
        void CANUltrasonic(int &dev_, int &ch_);
        void ultrasonic_Sender();
        void openCanRelay();
        void openCanRelay_();
        void openCanRelay_0();
        void ultrasonicDataParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void ultrasonicControl(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
    protected:
};

}
}

#endif
