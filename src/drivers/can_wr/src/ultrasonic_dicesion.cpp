#include "ultrasonic_dicesion.h"

#define ULTRASONIC_DICESION_YAML_FILE_PATH "/work/superg_agv/src/drivers/can_wr/config/ultrasonic_dicesion.yaml"
// #define ULTRASONIC_DICESION_FILE_NAME "/ultrasonic_dicesion.yaml"

namespace superg_agv
{

Ultrasonic_Dicesion::Ultrasonic_Dicesion(ros::NodeHandle &nh)
{
    // memset(agvstatus,0,sizeof(control_msgs::AGVStatus));
    // memset(ultrasonic_info,0,sizeof(perception_sensor_msgs::UltrasonicInfo));
    b_Array_Area.clear();
    i_Array_Ultrasonic_front.clear();
    i_Array_Ultrasonic_tail.clear();
    i_Array_Ultrasonic_right.clear();
    i_Array_Ultrasonic_left.clear();
    i_Array_Ultrasonic.clear();
    v_ultrantic_left.clear();
    v_ultrantic_right.clear();
    v_ultrantic_front.clear();
    v_ultrantic_tail.clear();
    v_ultrantic_regional.clear();
    agvstatus.used = true;
    ultrasonic_info.used = true;
    string yaml_path = getenv("HOME");
    yaml_path += ULTRASONIC_DICESION_YAML_FILE_PATH;
    ROS_INFO_STREAM(yaml_path);
    YAML::Node yamlConfig = YAML::LoadFile(yaml_path);
    for(uint16_t i = 0;i < 32;++i)
    {
        if(i < 4)
            i_Array_Ultrasonic_front.push_back(i);
        else if(i < 16)
            i_Array_Ultrasonic_left.push_back(i);
        else if(i < 20)
            i_Array_Ultrasonic_tail.push_back(i);
        else
            i_Array_Ultrasonic_right.push_back(i);
    }
    // car_length = 15470;
    // car_width = 2865;
    // Lf = 5508.5;
    // Lr = 5508.5;
    car_length = yamlConfig["car_length"].as<float>();
    car_width = yamlConfig["car_width"].as<float>();
    Lf = yamlConfig["Lf"].as<float>();
    Lr = yamlConfig["Lf"].as<float>();
    car_slip_angle = 0;
    car_w = 0;
    car_R = 0;
    Ox = 0;
    Oy = 0;
    car_R1 = 0;
    car_R2 = 0;
    car_R3 = 0;
    car_R4 = 0;
    point point_;
    for(unsigned j = 0;j < yamlConfig["Ultrasonic_front_list"].size();++j)
    {
        // ROS_INFO_STREAM(yamlConfig["Ultrasonic_front_list"][j]["ultrasonic_info"][0]["index"].as<int>());
        point_.index = yamlConfig["Ultrasonic_front_list"][j]["ultrasonic_info"][0]["index"].as<int>();
        point_.x     = yamlConfig["Ultrasonic_front_list"][j]["ultrasonic_info"][1]["x"].as<float>();
        point_.y     = yamlConfig["Ultrasonic_front_list"][j]["ultrasonic_info"][2]["y"].as<float>();
        ROS_INFO_STREAM("index:" << point_.index << ",point_.x:" << point_.x << ",point_.y:" << point_.y);
        v_ultrantic_front.push_back(point_);
    }
    for(unsigned j = 0;j < yamlConfig["Ultrasonic_left_list"].size();++j)
    {
        point_.index = yamlConfig["Ultrasonic_left_list"][j]["ultrasonic_info"][0]["index"].as<int>();
        point_.x     = yamlConfig["Ultrasonic_left_list"][j]["ultrasonic_info"][1]["x"].as<float>();
        point_.y     = yamlConfig["Ultrasonic_left_list"][j]["ultrasonic_info"][2]["y"].as<float>();
        ROS_INFO_STREAM("index:" << point_.index << ",point_.x:" << point_.x << ",point_.y:" << point_.x);
        v_ultrantic_left.push_back(point_);
    }
    for(unsigned j = 0;j < yamlConfig["Ultrasonic_tail_list"].size();++j)
    {
        point_.index = yamlConfig["Ultrasonic_tail_list"][j]["ultrasonic_info"][0]["index"].as<int>();
        point_.x     = yamlConfig["Ultrasonic_tail_list"][j]["ultrasonic_info"][1]["x"].as<float>();
        point_.y     = yamlConfig["Ultrasonic_tail_list"][j]["ultrasonic_info"][2]["y"].as<float>();
        ROS_INFO_STREAM("index:" << point_.index << ",point_.x:" << point_.x << ",point_.y:" << point_.x);
        v_ultrantic_tail.push_back(point_);
    }
    for(unsigned j = 0;j < yamlConfig["Ultrasonic_right_list"].size();++j)
    {
        point_.index = yamlConfig["Ultrasonic_right_list"][j]["ultrasonic_info"][0]["index"].as<int>();
        point_.x     = yamlConfig["Ultrasonic_right_list"][j]["ultrasonic_info"][1]["x"].as<float>();
        point_.y     = yamlConfig["Ultrasonic_right_list"][j]["ultrasonic_info"][2]["y"].as<float>();
        ROS_INFO_STREAM("index:" << point_.index << ",point_.x:" << point_.x << ",point_.y:" << point_.x);
        v_ultrantic_right.push_back(point_);
    }
    
    // for(uint16_t j = 0;j < 32;++j)
    // {
    //     if(j < 4)
    //     {
    //         point_.index = j;
    //         point_.x     = car_length/2;
    //         point_.y     = 542.4 - 361.6 * j;
    //         v_ultrantic_front.push_back(point_);
    //     }
    //     else if(j < 16)
    //     {
    //         point_.index = j;
    //         point_.x     = 3380.4 - (614.6) * (j - 4);
    //         point_.y     = -car_width/2;
    //         v_ultrantic_left.push_back(point_);
    //     }
    //     else if(j < 20)
    //     {
    //         point_.index = j;
    //         point_.x     = -car_length/2;
    //         point_.y     = -542.4 + 361.6 * (j - 16);
    //         v_ultrantic_tail.push_back(point_);
    //     }
    //     else
    //     {
    //         point_.index = j;
    //         point_.x     = -3380.4 + (614.6) * (j - 20);
    //         point_.y     = car_width/2;
    //         v_ultrantic_right.push_back(point_);
    //     }
    // }
    agvstatus_sub = nh.subscribe("/drivers/com2agv/agv_status",10,&Ultrasonic_Dicesion::AGV_Status_CB,this);
    ultrasonic_sub = nh.subscribe("/drivers/can_wr/sonser_info",10,&Ultrasonic_Dicesion::Ultrasonic_Info_CB,this);
    agvstatus_pub = nh.advertise<control_msgs::AGVStatus>("/control/control_agv",10,true);
    printf("Ultrasonic_Dicesion\n");
}

Ultrasonic_Dicesion::~Ultrasonic_Dicesion()
{

}

void Ultrasonic_Dicesion::AGV_Status_CB(const control_msgs::AGVStatus &msg)
{
    // agvstatus.agvstatus = msg;
    // agvstatus.used = false;
    // printf("AGV_Status_CB\n");
    set_agvstatus(msg);
}

void Ultrasonic_Dicesion::Ultrasonic_Info_CB(const perception_sensor_msgs::UltrasonicInfo &msg)
{
    // ultrasonic_info.ultrasonic_info = msg;
    // ultrasonic_info.used = false;
    // printf("Ultrasonic_Info_CB\n");
    set_ultrasonic_info(msg);
}

// void Ultrasonic_Dicesion::turnv_compute(const control_msgs::AGVStatus &_agvstatus,double &_turnv)
// {
//     if(_agvstatus.ActualAgl_R == 0 && _agvstatus.ActualAgl_R == 0)
//   {
//     _turnv = 0;
//   }
//   else
//   {
//     if(_agvstatus.ActualAgl_R*_agvstatus.ActualAgl_F >= 0)//同向转向
//     {
//       if(_agvstatus.ActualAgl_R == _agvstatus.ActualAgl_F)//同向斜行
//       {
//         _turnv = 0;
//       }
//       else//同向非斜行
//       {
//         _turnv = (180/pi)*_agvstatus.ActualSpd/(5.5085*sqrt((4+(tan(abs(_agvstatus.ActualAgl_F)*pi/180) + tan(abs(_agvstatus.ActualAgl_R)*pi/180))*(tan(abs(_agvstatus.ActualAgl_F)*pi/180) + tan(abs(_agvstatus.ActualAgl_R)*pi/180)))/((tan(abs(_agvstatus.ActualAgl_R)*pi/180) - tan(abs(_agvstatus.ActualAgl_F)*pi/180))*(tan(abs(_agvstatus.ActualAgl_R)*pi/180) - tan(abs(_agvstatus.ActualAgl_F)*pi/180)))));
//       }
//     }
//     else//反向转向
//     {
//       _turnv = (180/pi)*_agvstatus.ActualSpd/(5.5085*sqrt((4+(tan(abs(_agvstatus.ActualAgl_F)*pi/180) - tan(abs(_agvstatus.ActualAgl_R)*pi/180))*(tan(abs(_agvstatus.ActualAgl_F)*pi/180) - tan(abs(_agvstatus.ActualAgl_R)*pi/180)))/((tan(abs(_agvstatus.ActualAgl_R)*pi/180) + tan(abs(_agvstatus.ActualAgl_F)*pi/180))*(tan(abs(_agvstatus.ActualAgl_R)*pi/180) + tan(abs(_agvstatus.ActualAgl_F)*pi/180)))));
//     }
//   }
// }

void Ultrasonic_Dicesion::set_Regional_Screening(const control_msgs::AGVStatus &_agvstatus)
{
    v_ultrantic_regional.clear();
    float temp_y = 0;
    point point_;
    switch(_agvstatus.Dir_PRND)
    {
        case 1://前进挡
            if(_agvstatus.ActualSpd > 0)
            {
                for(vector<point>::iterator ite_v_ultrantic = v_ultrantic_front.begin();ite_v_ultrantic != v_ultrantic_front.end();++ite_v_ultrantic)
                {
                    point_ = *ite_v_ultrantic;
                    point_.safe_distance = 2500;
                    v_ultrantic_regional.push_back(point_);
                }
                ROS_INFO_STREAM("o3x:" << -car_length/2 << ",o3y:" << -sqrt(pow(car_R3,2) - pow((-car_length/2 - Ox),2)) + Oy);
                for(vector<point>::iterator ite_v_ultrantic = v_ultrantic_left.begin();ite_v_ultrantic != v_ultrantic_left.end();++ite_v_ultrantic)
                {
                    temp_y = min((-sqrt(pow(car_R3,2) - pow((ite_v_ultrantic->x - Ox),2)) + Oy),(sqrt(pow(car_R3,2) - pow((ite_v_ultrantic->x - Ox),2)) + Oy));
                        // ROS_INFO("temp_y:%f",temp_y);
                    if(ite_v_ultrantic->y > temp_y)
                    {
                        point_ = *ite_v_ultrantic;
                            // ROS_INFO_STREAM("ult_index:" << point_.index << ",ult_x:" << point_.x << ",ult_y:" << point_.y);
                        point_.safe_distance = fabs(ite_v_ultrantic->y - temp_y);
                        v_ultrantic_regional.push_back(point_);
                            // ROS_INFO_STREAM("ult_index:" << ite_v_ultrantic->index << ",ult_x:" << ite_v_ultrantic->x << ",ult_y:" << ite_v_ultrantic->y);
                    }
                }
                for(vector<point>::iterator ite_v_ultrantic = v_ultrantic_right.begin();ite_v_ultrantic != v_ultrantic_right.end();++ite_v_ultrantic)
                {
                    temp_y = min((-sqrt(pow(car_R4,2) - pow((ite_v_ultrantic->x - Ox),2)) + Oy),(sqrt(pow(car_R4,2) - pow((ite_v_ultrantic->x - Ox),2)) + Oy));
                    if(ite_v_ultrantic->y < temp_y)
                    {
                        point_ = *ite_v_ultrantic;
                            // ROS_INFO_STREAM("ult_index:" << point_.index << ",ult_x:" << point_.x << ",ult_y:" << point_.y);
                        point_.safe_distance = fabs(ite_v_ultrantic->y - temp_y);
                        v_ultrantic_regional.push_back(point_);
                            // ROS_INFO_STREAM("ult_index:" << ite_v_ultrantic->index << ",ult_x:" << ite_v_ultrantic->x << ",ult_y:" << ite_v_ultrantic->y);
                    }
                }
            }
            break;
        case 4://后退挡
            // ROS_INFO_STREAM("o3x:" << -car_length/2 << ",o3y:" << -sqrt(pow(car_R3,2) - pow((-car_length/2 - Ox),2)) + Oy);
            for(vector<point>::iterator ite_v_ultrantic = v_ultrantic_tail.begin();ite_v_ultrantic != v_ultrantic_tail.end();++ite_v_ultrantic)
            {
                point_ = *ite_v_ultrantic;
                point_.safe_distance = 2500;
                v_ultrantic_regional.push_back(point_);
            }
            for(vector<point>::iterator ite_v_ultrantic = v_ultrantic_left.begin();ite_v_ultrantic != v_ultrantic_left.end();++ite_v_ultrantic)
            {
                // temp_y = -sqrt(pow(R1,2) - pow((7735 - ite_v_ultrantic->x),2)) - (2865/2);
                temp_y = min((-sqrt(pow(car_R1,2) - pow((ite_v_ultrantic->x - Ox),2)) + Oy),(sqrt(pow(car_R1,2) - pow((ite_v_ultrantic->x - Ox),2)) + Oy));
                // ROS_INFO("temp_y:%f",temp_y);
                if(ite_v_ultrantic->y > temp_y)
                {
                    point_ = *ite_v_ultrantic;
                    point_.safe_distance = fabs(ite_v_ultrantic->y - temp_y);
                    v_ultrantic_regional.push_back(point_);
                    // ROS_INFO_STREAM("ult_index:" << ite_v_ultrantic->index << ",ult_x:" << ite_v_ultrantic->x << ",ult_y:" << ite_v_ultrantic->y);
                }
            }
            for(vector<point>::iterator ite_v_ultrantic = v_ultrantic_right.begin();ite_v_ultrantic != v_ultrantic_right.end();++ite_v_ultrantic)
            {
                // temp_y = -sqrt(pow(R1,2) - pow((7735 - ite_v_ultrantic->x),2)) - (2865/2);
                temp_y = min((-sqrt(pow(car_R2,2) - pow((ite_v_ultrantic->x - Ox),2)) + Oy),(sqrt(pow(car_R2,2) - pow((ite_v_ultrantic->x - Ox),2)) + Oy));
                // ROS_INFO("temp_y:%f",temp_y);
                if(ite_v_ultrantic->y < temp_y)
                {
                    point_ = *ite_v_ultrantic;
                    point_.safe_distance = fabs(ite_v_ultrantic->y - temp_y);
                    v_ultrantic_regional.push_back(point_);
                    // ROS_INFO_STREAM("ult_index:" << ite_v_ultrantic->index << ",ult_x:" << ite_v_ultrantic->x << ",ult_y:" << ite_v_ultrantic->y);
                }
            }
            break;
        default:
            break;    
    }
}

void Ultrasonic_Dicesion::Regional_Screening(const control_msgs::AGVStatus &_agvstatus)
{
    ROS_INFO_STREAM("Dir_PRND:" << _agvstatus.Dir_PRND);
    b_Array_Area.clear();
    switch(_agvstatus.Dir_PRND)
    {
        case 1://前进挡
            ROS_INFO_STREAM("a:" << _agvstatus.ActualAgl_R * _agvstatus.ActualAgl_F);
            if((_agvstatus.ActualAgl_R * _agvstatus.ActualAgl_F) > 0)//斜行
            {
                if(_agvstatus.ActualAgl_R > 0)//右斜行，前，右
                {
                    b_Array_Area.push_back(true);
                    b_Array_Area.push_back(false);
                    b_Array_Area.push_back(false);
                    b_Array_Area.push_back(true);
                }
                else//左斜行，前，左
                {
                    b_Array_Area.push_back(true);
                    b_Array_Area.push_back(true);
                    b_Array_Area.push_back(false);
                    b_Array_Area.push_back(false);
                }
            }
            else if((_agvstatus.ActualAgl_R * _agvstatus.ActualAgl_F) < 0)//大角度转向,全区域
            {
                b_Array_Area.push_back(true);
                b_Array_Area.push_back(true);
                b_Array_Area.push_back(true);
                b_Array_Area.push_back(true);
            }
            else//
            {
                if(_agvstatus.ActualAgl_R == _agvstatus.ActualAgl_F)//直行，前，左，右
                {
                    b_Array_Area.push_back(true);
                    b_Array_Area.push_back(true);
                    b_Array_Area.push_back(false);
                    b_Array_Area.push_back(true);
                }
                else if((_agvstatus.ActualAgl_R + _agvstatus.ActualAgl_F) > 0)//前，右
                {
                    b_Array_Area.push_back(true);
                    b_Array_Area.push_back(false);
                    b_Array_Area.push_back(false);
                    b_Array_Area.push_back(true);
                }
                else//前，左
                {
                    b_Array_Area.push_back(true);
                    b_Array_Area.push_back(true);
                    b_Array_Area.push_back(false);
                    b_Array_Area.push_back(false);
                }
            }
            for(vector<bool>::iterator ite_Array_Area = b_Array_Area.begin();ite_Array_Area != b_Array_Area.end();++ite_Array_Area)
            {
                ROS_INFO_STREAM("b_Array_Area:" << *ite_Array_Area);
            }
            break;
        case 4://后退挡
            if((_agvstatus.ActualAgl_R * _agvstatus.ActualAgl_F) > 0)//斜行
            {
                if(_agvstatus.ActualAgl_R > 0)//右斜行，前，右
                {
                    b_Array_Area.push_back(false);
                    b_Array_Area.push_back(true);
                    b_Array_Area.push_back(true);
                    b_Array_Area.push_back(false);
                }
                else//左斜行，前，左
                {
                    b_Array_Area.push_back(false);
                    b_Array_Area.push_back(false);
                    b_Array_Area.push_back(true);
                    b_Array_Area.push_back(true);
                }
            }
            else if((_agvstatus.ActualAgl_R * _agvstatus.ActualAgl_F) < 0)//大角度转向,全区域
            {
                b_Array_Area.push_back(true);
                b_Array_Area.push_back(true);
                b_Array_Area.push_back(true);
                b_Array_Area.push_back(true);
            }
            else//
            {
                if(_agvstatus.ActualAgl_R == _agvstatus.ActualAgl_F)//直行，前，左，右
                {
                    b_Array_Area.push_back(false);
                    b_Array_Area.push_back(true);
                    b_Array_Area.push_back(true);
                    b_Array_Area.push_back(true);
                }
                else if((_agvstatus.ActualAgl_R + _agvstatus.ActualAgl_F) > 0)//前，右
                {
                    b_Array_Area.push_back(false);
                    b_Array_Area.push_back(true);
                    b_Array_Area.push_back(true);
                    b_Array_Area.push_back(false);
                }
                else//前，左
                {
                    b_Array_Area.push_back(false);
                    b_Array_Area.push_back(false);
                    b_Array_Area.push_back(true);
                    b_Array_Area.push_back(true);
                }
            }
            for(vector<bool>::iterator ite_Array_Area = b_Array_Area.begin();ite_Array_Area != b_Array_Area.end();++ite_Array_Area)
            {
                ROS_INFO_STREAM("b_Array_Area:" << *ite_Array_Area);
            }
            break;
        default:
            b_Array_Area.push_back(false);
            b_Array_Area.push_back(false);
            b_Array_Area.push_back(false);
            b_Array_Area.push_back(false);
            break;
    }
}

void Ultrasonic_Dicesion::Dicesion(const perception_sensor_msgs::UltrasonicInfo &ultrasonic_info_ ,vector<bool> b_Array_Area)
{
    // uint8_t i = 0;
    // if(!ultrasonic_info.ult_obstacle.size())
    // {
    //     for(vector<bool>::iterator ite_Area = b_Array_Area.begin();ite_Area != b_Array_Area.end();++ite_Area)
    //     {
    //         if(ite_Area)
    //         {
    //             for(vector<uint16_t>::iterator ite_Ultrasonic = )
    //         }
    //     }
    // }
    vector<uint16_t>::iterator it;
    uint32_t ultrasonic_id = 32;
    if(ultrasonic_info_.ult_obstacle.size())
    {
        printf("!ultrasonic_info_.ult_obstacle.size()\n");
        for(uint8_t j = 0;j < ultrasonic_info_.ult_obstacle.size();++j)
        {
            ultrasonic_id = ultrasonic_info_.ult_obstacle.at(j).id;
            ROS_INFO_STREAM("ultrasonic_id:" << ultrasonic_id);
            for(uint8_t i = 0;i < b_Array_Area.size();++i)
            {
                if(b_Array_Area[i] && 0 == i)
                {
                    it = find(i_Array_Ultrasonic_front.begin(),i_Array_Ultrasonic_front.end(),ultrasonic_id);
                    if(it != i_Array_Ultrasonic_front.end())
                    {
                        printf("front ultransonic warrning\n");
                        //存在
                    }
                    else
                    {
                        printf("front ultransonic is clear\n");
                        //不存在
                    }
                }
                if(b_Array_Area[i] && 1 == i)
                {
                    it = find(i_Array_Ultrasonic_left.begin(),i_Array_Ultrasonic_left.end(),ultrasonic_id);
                    if(it != i_Array_Ultrasonic_left.end())
                    {
                        printf("left ultransonic warrning\n");
                        //存在
                    }
                    else
                    {
                        printf("left ultransonic is clear\n");
                        //不存在
                    }
                }
                if(b_Array_Area[i] && 2 == i)
                {
                    it = find(i_Array_Ultrasonic_tail.begin(),i_Array_Ultrasonic_tail.end(),ultrasonic_id);
                    if(it != i_Array_Ultrasonic_tail.end())
                    {
                        printf("tail ultransonic warrning\n");
                        //存在
                    }
                    else
                    {
                        printf("tail ultransonic is clear\n");
                        //不存在
                    }
                }
                if(b_Array_Area[i] && 3 == i)
                {
                    it = find(i_Array_Ultrasonic_right.begin(),i_Array_Ultrasonic_right.end(),ultrasonic_id);
                    if(it != i_Array_Ultrasonic_right.end())
                    {
                        printf("right ultransonic warrning\n");
                        //存在
                    }
                    else
                    {
                        printf("right ultransonic is clear\n");
                        //不存在
                    }
                }
            }
        }
        // ultrasonic_info.clear();
    }
}

void Ultrasonic_Dicesion::set_agvstatus(const control_msgs::AGVStatus &msg)
{
    mtx_agvstatus.lock();
    agvstatus.agvstatus = msg;
    agvstatus.used = false;
    mtx_agvstatus.unlock();
}

void Ultrasonic_Dicesion::set_ultrasonic_info(const perception_sensor_msgs::UltrasonicInfo &msg)
{
    mtx_ultrasonic_info.lock();
    ultrasonic_info.ultrasonic_info = msg;
    ultrasonic_info.used = false;
    mtx_ultrasonic_info.unlock();
}

bool Ultrasonic_Dicesion::get_agvstatus_used()
{
    bool used_;
    mtx_agvstatus.lock();
    used_ = agvstatus.used;
    mtx_agvstatus.unlock();
    return used_;
}

bool Ultrasonic_Dicesion::get_ultrasonic_info_used()
{
    bool used_;
    mtx_ultrasonic_info.lock();
    used_ = ultrasonic_info.used;
    mtx_ultrasonic_info.unlock();
    return used_;
}

control_msgs::AGVStatus Ultrasonic_Dicesion::get_agvstatus_value()
{
    control_msgs::AGVStatus agvstatus_;
    mtx_ultrasonic_info.lock();
    agvstatus_ = agvstatus.agvstatus;
    mtx_ultrasonic_info.unlock();
    return agvstatus_;
}

perception_sensor_msgs::UltrasonicInfo Ultrasonic_Dicesion::get_ultrasonic_info_value()
{
    perception_sensor_msgs::UltrasonicInfo ultrasonic_info_;
    mtx_ultrasonic_info.lock();
    ultrasonic_info_ = ultrasonic_info.ultrasonic_info;
    ultrasonic_info.used = true;
    mtx_ultrasonic_info.unlock();
    return ultrasonic_info_;
}

void Ultrasonic_Dicesion::send_stop()
{
    
}

void Ultrasonic_Dicesion::Ultrasonic_Dicesion_Node()
{
    perception_sensor_msgs::UltrasonicInfo ultrasonic_info_;
    control_msgs::AGVStatus agvstatus_;
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        // printf("thread Ultrasonic_Dicesion_Node runnning~!\n");
        if(!get_ultrasonic_info_used())//收到agv消息
        {
            printf("!get_ultrasonic_info_used()\n");
            ultrasonic_info_ = get_ultrasonic_info_value();
            ROS_INFO_STREAM("size:" << ultrasonic_info_.ult_obstacle.size());
            if(0 != ultrasonic_info_.ult_obstacle.size())//有超声波信息
            {
                if(!get_agvstatus_used())
                {
                    printf("!get_agvstatus_used()\n");
                    agvstatus_ = get_agvstatus_value();
                    if(2 == agvstatus_.VEHMode)
                    {
                        Regional_Screening(agvstatus_);
                        Dicesion(ultrasonic_info_,b_Array_Area);
                    }
                }
            }
            // turnv_compute(agvstatus,turnv);
            // Regional_Screening(agvstatus);//根据车辆信息计算出关注区域
            // Dicesion(ultrasonic_info,b_Array_Area);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

}
