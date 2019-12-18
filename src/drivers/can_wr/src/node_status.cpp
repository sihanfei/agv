#include "node_status.h"

namespace superg_agv
{
namespace drivers
{

// extern std::queue<string> q_node_malfunction;

node_status::node_status()
{
    node_status_info.node_name = ros::this_node::getName();
    node_status_info.node_pid = getpid();
    // ROS_INFO_STREAM("name:" << node_status_info.node_name << ",pid:" << node_status_info.node_pid);
    node_status_info.state_num = 0;
    node_status_info.status_counter = 0;
}

node_status::~node_status()
{
;
}

Node_Status_Info* node_status::get_Node_Status_Info()
{
    return &node_status_info;
}

void node_status::node_status_pub()
{
    ros::NodeHandle nh;
    ros::Publisher node_status_pub = nh.advertise<status_msgs::NodeStatus>("/node/node_status", 10, true);
    // node_status_info.node_name = ros::this_node::getName();
    // node_status_info.node_pid = getpid();
    ros::Rate rate(10);
    while(ros::ok())
    {
        // string a;
        // if(get_q_node_malfunction_size() != 0)
        // {
        //     a = q_node_malfunction_front();
        // }
        status_msgs::NodeStatus nodestatus;
        status_msgs::SafetyStatus safetystatus_msg;
        common_msgs::KeyValue keyvalue_msg;
        nodestatus.header.stamp = ros::Time::now();
        nodestatus.header.frame_id = "base_link";
        nodestatus.node_name = node_status_info.node_name;
        nodestatus.node_pid  = node_status_info.node_pid;
        nodestatus.state_num = get_q_node_malfunction_size();
        for(uint16_t i = 0;i < nodestatus.state_num;++i)
        {
            safetystatus_msg.message_code = q_node_malfunction_front().message_code;
            safetystatus_msg.counter = node_status_info.status_counter++;
            nodestatus.status.push_back(safetystatus_msg);
        }
        // list<Safety_Status>::iterator ite_SafetyStatus;
        // list<Key_Value>::iterator ite_KeyValue;
        node_status_pub.publish(nodestatus);
        rate.sleep();
    }
}

void node_status::sensor_info_push_back(Sensor_Info &Sensor_Info_)
{
    sensor_info.push_back(Sensor_Info_);
}

void node_status::channel_check()
{
    Safety_Status Safety_Status_;
    ros::Rate rate(10);
    while(ros::ok())
    {
        // list<Sensor_Info>::iterator ite_sensor_info = sensor_info.begin();
        // ROS_INFO_STREAM("sensor_info.size:" << sensor_info.size());
        for(list<Sensor_Info>::iterator ite_sensor_info = sensor_info.begin();
        ite_sensor_info != sensor_info.end();++ite_sensor_info)
        {
            if(adcuDevStatus(ite_sensor_info->devid) == ADCU_DEV_STATUS_ABNORMAL)
            {
                // ROS_INFO_STREAM(ite_sensor_info->sensor_name << "open error");
                if(!ite_sensor_info->sensor_name.compare("p2"))
                {
                    Safety_Status_.message_code = "E05014000";
                }
                else if(!ite_sensor_info->sensor_name.compare("camera_1"))
                {
                    Safety_Status_.message_code = "E05024000";
                }
                else if(!ite_sensor_info->sensor_name.compare("camera_2"))
                {
                    Safety_Status_.message_code = "E05034000";
                }
                else if(!ite_sensor_info->sensor_name.compare("ultrasonic"))
                {
                    Safety_Status_.message_code = "E05084000";
                }
                else if(!ite_sensor_info->sensor_name.compare("mobileye"))
                {
                    Safety_Status_.message_code = "E050134000";
                }
                // else if(!ite_sensor_info->sensor_name.compare("mobileye"))
                // {
                //     Safety_Status_.message_code = "E050134000";
                // }
                q_node_malfunction_push(Safety_Status_);
            }
        }
        rate.sleep();
    }
}


}
}

