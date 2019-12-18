#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include "adcuSDK.h"
#include "ros/ros.h"
#include <memory.h>
#include "node_status.h"
#include "power_control_msgs/PowerControlCmd.h"
#include "status_msgs/NodeStatus.h"
#include "status_msgs/SafetyStatus.h"
#include "common_msgs/KeyValue.h"
#include <mutex>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

using namespace std;
namespace superg_agv
{
namespace drivers
{
class Power_Control: public node_status
{
    public:
        Power_Control();
        ~Power_Control();
        void splice_protocol_15(uint8_t sendBuffer15[],uint8_t &addr_,uint8_t &cmd_,uint8_t data[]);
        void splice_protocol_10(uint8_t sendBuffer15[],uint8_t &addr_,uint8_t &cmd_,uint8_t &num,uint8_t &state,uint16_t &time);
        uint8_t calc_CRC(uint8_t msg[],uint8_t &len);
        bool send_485(uint8_t msg[],uint8_t &len);
        int16_t read_485();
        void read_control_state(uint8_t &addr_);
        void write_control_state(uint8_t &addr_,uint8_t msg[]);
        void analysis_control_state(uint8_t msg[],uint8_t &len);
        uint8_t get_control_state(uint8_t index);
        uint8_t get_control_state_size();
        void analysis_ros_cmd(uint8_t data[],uint8_t &len);
        virtual void node_status_pub();
        void power_control_callback(const power_control_msgs::PowerControlCmd::ConstPtr& msg);
        void exception_handling();
        bool send_socket(uint8_t msg[],uint8_t &len);
        int16_t read_socket();
        void loop_socket_read();
    private:
	std::mutex mtx_485;
        int devid;
        uint8_t readBuffer[100];
        uint8_t sendBuffer15[15];
        uint8_t sendBuffer10[15];
        uint8_t control_state[20];
        uint8_t head[2];
        uint8_t addr;
        uint8_t cmd;
        uint8_t data[8];
        uint8_t crc;
        uint8_t tail[2];
        adcuUartOpt opt;
        uint8_t opt_buff[16];

        string ip = "192.168.2.131";
        int port  = 1030;
        unsigned char recvline[15], sendline[15];
        int sockfd;
        struct sockaddr_in servaddr;
};
}
}

#endif
