#ifndef DRIVERS_CAN_WR_INCLUDE_MOBILEYE_H_
#define DRIVERS_CAN_WR_INCLUDE_MOBILEYE_H_

#include "can_rw.h"
#include "logdata.h"

namespace superg_agv
{
namespace drivers
{

#define VEHICLE     0
#define TRUCK       1
#define BIKE        2
#define PED         3
#define BICYCLE     7

#define MODLE_VEHICLE     4000
#define MODLE_TRUCK       1
#define MODLE_BIKE        1800
#define MODLE_PED         3
#define MODLE_BICYCLE     1800

class MOBILEYE:private CANDrivers
{
    typedef void (MOBILEYE::*pFun)(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
    
    map <uint32_t, pFun> m_pHandlerMap;
    private:
        std::mutex mtx_mobileye;
        std::condition_variable cond_mobileye;
        mobileyeLKALaneStr mobileyeLKALane_;
        // mobileyeObstaclesDataStr mobileyeObstaclesData_;
        mobileyeDataStr mobileyeData;

        enum 
        {
            NONE,
            OBSTACLES,
            LANE
        }MOBILEYE_RECV_TYPE;
    public:
        MOBILEYE();
        ~MOBILEYE();
        void CANMobileye(int &dev_, int &ch_);
        void mobileye_Sender();
        void mobileyeLaneInfoAndMeasureParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void mobileyeSystemWarningParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void mobileyeTSRTypeAndPositionParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void mobileyeTSRVisionDecisionPares(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void mobileyeLightsLocationAndAnglesPares(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void mobileyeLaneInfoMeasureParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void mobileyeNumberOfObstaclesParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void mobileyeObstaclesData_AParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void mobileyeObstaclesData_BParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void mobileyeObstaclesData_CParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void mobileyeSignalsStatusVehicleParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void mobileyeLKALeftLane_AParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void mobileyeLKALeftLane_BParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void mobileyeLKARightLane_AParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void mobileyeLKARightLane_BParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void mobileyeReference_PointsParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void mobileyeNunber_Of_Next_LaneParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
    protected:

}; 

}
}

#endif
