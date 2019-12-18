#ifndef DRIVERS_CAN_WR_INCLUDE_P2_H_
#define DRIVERS_CAN_WR_INCLUDE_P2_H_

#include "can_rw.h"
#include "driver_monitor.h"
#include "logdata.h"

namespace superg_agv
{
namespace drivers
{
class P2:private CANDrivers,public Driver_Monitor
{
    typedef void (P2::*pFun)(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);

    map< uint32_t, pFun > m_pHandlerMap;
  private:
    std::mutex mtx_p2;
    std::mutex mtx_p2_data;
    std::condition_variable cond_p2;
    p2DataStr p2Data;
    p2DataStr p2Data_send;
    ros::Time recvTime;
    struct timeDate
    {
      uint32_t year;
      uint8_t month;
      uint8_t day;
      uint8_t hour;
      uint8_t min;
      uint8_t sec;
      float msec;
    }timeDate;
    uint8_t rate;
    int devid;
  public:
    P2();
    ~P2();
    void CANP2(int &dev_, int &ch_);
    void p2_Sender();

    // int hex2int(uint8_t data[],uint16_t bit_start,uint16_t bit_end);

    void p2TimeParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
    void p2AngRateRawIMUParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
    void p2AccelIMURawParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
    void p2InsStatusParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
    void p2LatitudeLongitudeParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
    void p2AltitudeParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
    void p2PosSigmaParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
    void p2VelocityLevelParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
    void p2VelocityLevelSigmaParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
    void p2AccelVehicleParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
    void p2HeadingPitchRollParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
    void p2HeadingPitchRollSigmaParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
    void p2AngRateVehicleParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);

    void test();
  protected:

};
}
}

#endif