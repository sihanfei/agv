#ifndef DECESION_ULTRASONIC_DECESION_H_
#define DECESION_ULTRASONIC_DECESION_H_

#include <ros/ros.h>
#include "control_msgs/AGVStatus.h"
#include "perception_sensor_msgs/UltrasonicInfo.h"
#include <vector>
#include <mutex>
#include <yaml-cpp/yaml.h>

namespace superg_agv
{

using namespace std;

#define pi 3.141592 

struct Ultrasonic_Dicesion_agvstatus
{
    control_msgs::AGVStatus agvstatus;
    bool used;
};
struct Ultrasonic_Dicesion_ultrasonic_info
{
    perception_sensor_msgs::UltrasonicInfo ultrasonic_info;
    bool used;
};

struct point
{
    int index;
    float x;
    float y;
    float safe_distance;
};

class Ultrasonic_Dicesion
{
    public:
        Ultrasonic_Dicesion(ros::NodeHandle &nh);
        ~Ultrasonic_Dicesion();
        void AGV_Status_CB(const control_msgs::AGVStatus &msg);//agv信息回调
        void Ultrasonic_Info_CB(const perception_sensor_msgs::UltrasonicInfo &msg);//超声波数据回调
        // void turnv_compute(const control_msgs::AGVStatus &_agvstatus,double &_turnv);//计算agv角速率
        void Regional_Screening(const control_msgs::AGVStatus &_agvstatus);//根据前后转角筛选出关注区域
        void Dicesion(const perception_sensor_msgs::UltrasonicInfo &_ultrasonic_info ,vector<bool> b_Array_Area);
        void set_Regional_Screening(const control_msgs::AGVStatus &_agvstatus);
        void Ultrasonic_Dicesion_Node();//超声波决策节点
        void set_agvstatus(const control_msgs::AGVStatus &msg);
        void set_ultrasonic_info(const perception_sensor_msgs::UltrasonicInfo &msg);
        bool get_agvstatus_used();
        bool get_ultrasonic_info_used();
        control_msgs::AGVStatus get_agvstatus_value();
        perception_sensor_msgs::UltrasonicInfo get_ultrasonic_info_value();
        void send_stop();
    private:
        double turnv;//AGV转动角速率 deg/s
        vector<bool> b_Array_Area;//前，左，后，右     1为关注区域，0为非关注区域
        vector<uint16_t> i_Array_Ultrasonic_front;
        vector<uint16_t> i_Array_Ultrasonic_tail;
        vector<uint16_t> i_Array_Ultrasonic_right;
        vector<uint16_t> i_Array_Ultrasonic_left;
        vector<uint16_t> i_Array_Ultrasonic;


        float car_length;//车体长度 mm
        float car_width;//车体宽度 mm
        float Lf;//前悬长度 mm
        float Lr;//后悬长度 mm
        float car_slip_angle;//车辆中心滑移角
        float car_w;//车辆中心你角速度
        float car_R;//车辆中心旋转半径 mm
        float Ox;//旋转中心x mm
        float Oy;//旋转中心y mm
        float car_R1;//车体左前方角点的旋转半径 mm
        float car_R2;//车体右前方角点的旋转半径 mm
        float car_R3;//车体左后方角点的旋转半径 mm
        float car_R4;//车体右后方角点的旋转半径 mm
        vector<point> v_ultrantic_left;
        vector<point> v_ultrantic_right;
        vector<point> v_ultrantic_front;
        vector<point> v_ultrantic_tail;
        vector<point> v_ultrantic_regional;

        std::mutex mtx_agvstatus;
        std::mutex mtx_ultrasonic_info;

        Ultrasonic_Dicesion_agvstatus agvstatus;
        // control_msgs::AGVStatus agvstatus;
        // perception_sensor_msgs::UltrasonicInfo ultrasonic_info;
        Ultrasonic_Dicesion_ultrasonic_info ultrasonic_info;

        ros::Subscriber agvstatus_sub;
        ros::Subscriber ultrasonic_sub;
        ros::Publisher agvstatus_pub;
    protected:
};
}
#endif
