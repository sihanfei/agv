#include "can_drivers.h"
// #include "p2.h"
// #include "camera.h"
// #include "ultrasonic.h"
// #include "mobileye.h"
// #include "power_control.h"
// #include "driver_monitor.h"
#include "node_status.h"
#include "can_node.h"

using namespace std;
using namespace superg_agv;
using namespace drivers;

// extern std::queue<string> Driver_Monitor::q_sensor_type;
// extern std::queue<Safety_Status> Driver_Monitor::q_node_malfunction;

int main(int argc,char* argv[])
{
    ros::init(argc, argv, "can_rw_camera");
    ros::NodeHandle nh;

    auto cpu_num = thread::hardware_concurrency();
    ROS_INFO("CPU num is : %d", cpu_num);

    Sensor_Info sensor_info_;

    adcuSDKInit();

    usleep(1000 * 2000);

    int channel;
    int devid = 1;
    uint8_t opt_buff[16] = {0};

    CANDrivers can_rw;
    CAN_Node CAN_Node_;

    usleep(1000 * 1000);
    printf("open channel\n");

    adcuDeviceType deviceType;

    deviceType = adcuCAN;

    thread t_camera_send(std::bind(&CAN_Node::camera_Sender, &CAN_Node_));
    t_camera_send.detach();

    channel                 = CHANNEL_CAMERA_1;
    devid                   = adcuDevOpen(deviceType, ( adcuDeviceChannel )channel);
    opt_buff[0]             = channel - 1;
    opt_buff[1]             = 5;
    adcuDevSetOpt(devid,opt_buff,2);
    can_rw.write_num[devid] = channel;
    printf("open %d %d\n", channel, devid);
    thread can_readcamera1(std::bind(&CAN_Node::CANCamera, &CAN_Node_, channel, devid));
    can_readcamera1.detach();
    sensor_info_.sensor_name    = "camera_1";
    sensor_info_.sensor_channel = CHANNEL_CAMERA_1;
    sensor_info_.devid = devid;
    CAN_Node_.sensor_info_push_back(sensor_info_);

    // thread can_writecamera1(std::bind(&CAN_Node::CANCameraWrite, &CAN_Node_, channel, devid));
    // can_writecamera1.detach();


    channel                 = CHANNEL_CAMERA_2;
    devid                   = adcuDevOpen(deviceType, ( adcuDeviceChannel )channel);
    opt_buff[0]             = channel - 1;
    opt_buff[1]             = 5;
    adcuDevSetOpt(devid,opt_buff,2);
    can_rw.write_num[devid] = channel;
    printf("open %d %d\n", channel, devid);
    thread can_readcamera2(std::bind(&CAN_Node::CANCamera, &CAN_Node_, channel, devid));
    can_readcamera2.detach();
    sensor_info_.sensor_name    = "camera_2";
    sensor_info_.sensor_channel = CHANNEL_CAMERA_2;
    sensor_info_.devid = devid;
    CAN_Node_.sensor_info_push_back(sensor_info_);

    while (ros::ok())
    {
        usleep(1000 * 1000);
    }

    usleep(1000 * 2000);
    adcuSDKDeinit();

    for (size_t i = 0; i < 10; i++)
    {
        usleep(1000 * 1000);
        ROS_INFO_STREAM("main will closed after " << (10 - i));
    }

    ROS_INFO("shutting down!\n");
    ros::shutdown();
    return 0;
}
