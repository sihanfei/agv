#ifndef DRIVERS_MONITOR_H_
#define DRIVERS_MONITOR_H_

#include "ros/ros.h"
#include <string>
#include <list>
#include <queue>
#include <mutex>

using namespace std;

namespace superg_agv
{
namespace drivers
{

enum Sensor_Fre_State
{
  NORMAL,INFO,WRINNING,ERROR
};

struct sensor_info
{
  string sensor_name;
  float frequency;
  list<uint8_t> list_time;
  uint16_t wait_time;
  uint16_t WAIT_TIME_MAX;
  uint8_t count;
  float FREQUENCY;
  Sensor_Fre_State sensor_fre_state;
};

struct Key_Value
{
    uint8_t INT8;
    uint8_t UINT8;
    uint8_t INT16;
    uint8_t UINT16;
    uint8_t INT32;
    uint8_t UINT32;
    uint8_t FLOAT32;
    uint8_t FLOAT64;
    uint8_t STRING;
    string key;
    uint8_t valuetype;
    string value;
};

struct Safety_Status
{
    string message_code;
    uint64_t counter;
    uint16_t hardware_id;
    uint16_t value_num;
    list<Key_Value> KeyValue;
};

class Driver_Monitor
{
  public:
    Driver_Monitor();
    ~Driver_Monitor();
    void frequency_monitor();
    // virtual void state_check();
    // virtual void data_check();
    bool q_sensor_empty();
    int q_sensor_front();
    uint16_t get_q_sensor_size();
    void q_sensor_push(int &sen_num);
    void q_node_malfunction_push(Safety_Status &safety_status);
    Safety_Status q_node_malfunction_front();
    uint16_t get_q_node_malfunction_size();
    void set_rate(uint8_t &rate);
    void sensor_frequency_calculation(sensor_info &frequency_info,bool bRecv);
    void frequency_check(sensor_info &frequency_info);
    void splice_msg_code(string level,string module,string submodule,string type,uint16_t content);
  private:
    static std::mutex mtx_node_malfunction;
    static std::queue<Safety_Status> q_node_malfunction;
    // std::queue<Safety_Status> q_node_malfunction;
    static std::mutex mtx_sensor_type;
    // static std::queue<string> q_sensor_type;
    static list<int> q_sensor_type;
    // std::queue<string> q_sensor_type;
    uint8_t rate;
    sensor_info p2_info;
    sensor_info camera1_info;
    sensor_info camera2_info;
    sensor_info camera3_info;
    sensor_info camera4_info;
    sensor_info camera5_info;
    sensor_info camera6_info;
    sensor_info ultrasonic1_info;
    sensor_info ultrasonic2_info;
    sensor_info ultrasonic3_info;
    sensor_info ultrasonic4_info;
};

}
}


#endif
