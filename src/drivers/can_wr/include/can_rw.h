#ifndef DRIVERS_CAN_WR_INCLUDE_CAN_RW_H_
#define DRIVERS_CAN_WR_INCLUDE_CAN_RW_H_

#include "adcuSDK.h"
#include "can_drivers.h"
#include "protocol.h"
#include "ros/ros.h"
#include "threadpool.h"
#include <errno.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>
#include <csignal>
#include <sys/time.h>
#include <fstream>

#include <numeric>

#include "spdlog/logger.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/async.h"
#include "spdlog/fmt/bin_to_hex.h"
#include <sstream>

using namespace std;

namespace superg_agv
{
namespace drivers
{

class CANData_Output
{
  public:
    CANData_Output(string &pram1);
    ~CANData_Output();
    // bool Openfile(string &path, string &filename);bool CANData_Output::Openfile(string &pram1, string &pram2, adcuCanData &data)
    bool Openfile(string &pram1, string &pram2, adcuCanData &data, int &ch_);
    bool Closefile(string &path, string &filename);
    bool Isexistence(string &path, string &filename);
    bool Writefile(string &path, string &filename, adcuCanData &data);
  private:
    ofstream logfile;
    string filepath = "/home/higo/work/log/";
    int min;
    string sensor_type;
    time_t systime;
    struct tm *ptminfo;
};


class CANDrivers
{

public:
  typedef void (CANDrivers::*pFun)(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
  struct taskData
  {
    std::mutex mtx_;
    std::mutex mtx_camera_r;
    std::mutex mtx_camera_w;
    std::condition_variable cond_camera_r;
    std::condition_variable cond_camera_w;
    bool is_shutdown_ = false;
    std::queue< std::function< void() > > tasks_;
  }taskData;
  

  vector< vector< adcuCanData > > can_data;
  //写指针
  int read_num[CAN_DATA_SIZE];
  //读指针
  int write_num[CAN_DATA_SIZE];
  //函数map
  map< uint32_t, pFun > m_pHandlerMap;

public:
  uint8_t begain_tip[CAN_DATA_SIZE];

private:

public:

  CANDrivers(/* args */);
  ~CANDrivers();
  void writeCAN(int &dev_);
  int writeCANOnce(int &dev_, uint32_t can_id, uint16_t candata, uint8_t le);
  int readCANOnce(int &dev_, adcuCanData &can_buf);
  int writeCANtoP2(int &dev_, uint32_t can_id, uint8_t *candata, uint8_t le);

  void InitHandlerMap();
};

} // namespace drivers
} // namespace superg_agv

#endif
