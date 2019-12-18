#include "power_control.h"
#include "node_status.h"
#include <thread>

using namespace std;
using namespace superg_agv;
using namespace drivers;

// Power_Control power_control;

// std::queue<string> Driver_Monitor::q_sensor_type;
// std::queue<Safety_Status> Driver_Monitor::q_node_malfunction;

// void power_control_callback(const power_control_msgs::PowerControlCmd::ConstPtr& msg)
// {
//     uint8_t msg_data[100];
//     uint8_t len = msg->data.size();
//     for(uint8_t i = 0; i < len;++i)
//     {
//         msg_data[i] = msg->data[i];
//     }
//     power_control.analysis_ros_cmd(msg_data,len);
//     ROS_INFO_STREAM("msg_len:" << msg->length << ",data0;%u" << msg->data[0]);
//     ROS_INFO("power_control_callback");
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "power_control");
    ros::NodeHandle nh;
    Power_Control power_control;
    // ros::Publisher node_state_pub = nh.advertise<status_msgs::NodeStatus>("/node/node_status", 1000, true);
    ros::Subscriber node_state_sub = nh.subscribe("/power_control/power_control_cmd", 1000, &Power_Control::power_control_callback,&power_control);
//    ros::Subscriber node_state_sub = nh.subscribe("/power_control/power_control_cmd", 1000, power_control_callback);
    ros::Rate loop_rate(10);
    // Power_Control power_control;
    // node_status node_status_;

    thread node_status_pub(std::bind(&Power_Control::node_status_pub, &power_control));
    node_status_pub.detach();
    ROS_INFO("node_status_pub.detach");
    thread node_exception_handling(std::bind(&Power_Control::exception_handling, &power_control));
    node_exception_handling.detach();
    ROS_INFO("node_exception_handling.detach");

    while(ros::ok())
    {
	ROS_INFO("power_control_once_loop");
        // status_msgs::NodeStatus node_ststus_msg;
        // status_msgs::SafetyStatus safetystatus_msg;
        // common_msgs::KeyValue keyvalue_msg;
        // node_ststus_msg.header.stamp      = ros::Time::now();
        // node_ststus_msg.header.frame_id   = "base_link";
        // node_ststus_msg.node_name = node_name;//
        // node_ststus_msg.node_pid  = node_pid;//
        // node_ststus_msg.state_num = 1;
        // safetystatus_msg.message_code = "";//
        // safetystatus_msg.counter      = 0;//
        // safetystatus_msg.hardware_id  = 0;//
        // safetystatus_msg.value_num    = power_control.get_control_state_size();
        // for(uint8_t i = 0;i < power_control.get_control_state_size();++i)
        // {
        //     // keyvalue_msg.UINT8 = power_control.get_control_state(i);
        //     safetystatus_msg.values.push_back(keyvalue_msg);
        // }
        // node_ststus_msg.status.push_back(safetystatus_msg);
        // node_state_pub.publish(node_ststus_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
