#ifndef NODE_STATUS_H
#define NODE_STATUS_H

#include "ros/ros.h"
#include <string>
#include <list>
#include <queue>
#include "driver_monitor.h"
#include "can_rw.h"
#include "status_msgs/NodeStatus.h"


using namespace std;

namespace superg_agv
{
namespace drivers
{

// struct Key_Value
// {
//     uint8_t INT8;
//     uint8_t UINT8;
//     uint8_t INT16;
//     uint8_t UINT16;
//     uint8_t INT32;
//     uint8_t UINT32;
//     uint8_t FLOAT32;
//     uint8_t FLOAT64;
//     uint8_t STRING;
//     string key;
//     uint8_t valuetype;
//     string value;
// };

// struct Safety_Status
// {
//     string message_code;
//     uint64_t counter;
//     uint16_t hardware_id;
//     uint16_t value_num;
//     list<Key_Value> KeyValue;
// };

struct Sensor_Info
{
    string sensor_name;
    int sensor_channel;
    int devid;
};

struct Node_Status_Info
{
    string node_name;
    uint64_t node_pid;
    uint16_t state_num;
    uint64_t status_counter;
    list<Safety_Status> SafetyStatus;
};

// class Driver_Monitor;

class node_status: public Driver_Monitor
{
    // Driver_Monitor *Driver_Monitor_;
    public:
        node_status();
        ~node_status();
        Node_Status_Info* get_Node_Status_Info();
        virtual void node_status_pub();
        void sensor_info_push_back(Sensor_Info &Sensor_Info_);
        void channel_check();
    private:
        Node_Status_Info node_status_info;
        list<Sensor_Info> sensor_info;
};

}
}

#endif
