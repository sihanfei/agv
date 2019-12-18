#include "conti_radar_can.hpp"
#include "radar_config_200.h"
#include "radar_state_201.h"
using namespace std ;

struct can_frame frame;


ContiRadarCan::ContiRadarCan()
{
    is_conti_radar_configed=false;
    conti_radar_.object_list_status.interface_version=0;
    conti_radar_.object_list_status.nof_objects=0;
    conti_radar_.object_list_status.meas_counter=0;
    conti_radar_.radar_state.max_distance=240;
    conti_radar_.radar_state.output_type=OUTPUT_TYPE_NONE;
    conti_radar_.radar_state.send_quality=false;
    conti_radar_.radar_state.send_ext_info=false;
    conti_radar_.radar_state.rcs_threshold=RCS_THRESHOLD_STANDARD;
    conti_radar_.radar_state.radar_power=0;
}

ContiRadarCan::~ContiRadarCan()
{
   close(s);
}

void ContiRadarCan::conti_radar_can_init(string can_serial)
{
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW); //创建套接字
    cout<<"can_serial :"<<can_serial.c_str()<<endl;
    strcpy(ifr.ifr_name, can_serial.c_str());
    ioctl(s, SIOCGIFINDEX, &ifr); //指定 can1 设备
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr)); //将套接字与 can1 绑定
    cout<<"The can receiver is successfully initialized"<<endl;
}

void ContiRadarCan::conti_radar_can_close()
{
    close(s);
}
void ContiRadarCan::conti_radar_can_Recv()
{
   nbytes = read(s, &frame, sizeof(frame)); //接收报文
   if(nbytes > 0)
    {
     ObjectListStatus60A   cluster_list_status60a;
     ObjectGeneralInfo60B  object_general_info60b;
     ObjectQualityInfo60C  object_general_info60c;
     ObjectExtendedInfo60D object_extended_info60d;
     RadarState201  radar_state_201;
     if (!is_conti_radar_configed && frame.can_id!= RadarState201::ID)
     {
        return ;
     }
     if(RadarState201::ID==frame.can_id)
     {
        radar_state_201.Parse(frame.data,8,conti_radar_);
        if (conti_radar_.radar_state.send_quality==true&&
        conti_radar_.radar_state.send_ext_info ==true &&
        conti_radar_.radar_state.max_distance ==250&&
        conti_radar_.radar_state.output_type ==OUTPUT_TYPE_OBJECTS &&
        conti_radar_.radar_state.rcs_threshold ==RCS_THRESHOLD_STANDARD&&
        conti_radar_.radar_state.radar_power==0)
        {
          is_conti_radar_configed=true;
        }
        else
        {
          cout<<"send configuration to radar! "<<endl;
          conti_radar_can_send();
          cout<<"configuration successfully "<<endl;
        }
     }
     switch(frame.can_id)
     {
        case 0x60A:
        {
            //接收到的消息
            conti_can_bus_info_to_sensor();
            conti_radar_.object_list_status.nof_objects=0;
            conti_radar_.object_list_status.meas_counter=0;
            conti_radar_.object_list_status.interface_version=0;
            conti_radar_.conti_radar_obs.clear();
            cluster_list_status60a.Parse(frame.data,8,conti_radar_);
            break;
        }
        case 0x60B:
        {
             object_general_info60b.Parse(frame.data,8,conti_radar_);
             break;
        }
        case 0x60C:
        {
            object_general_info60c.Parse(frame.data,8,conti_radar_);
            break;
        }
        case 0x60D:
        {
            object_extended_info60d.Parse(frame.data,8,conti_radar_);
            break;
        }

     }

   }
}

void ContiRadarCan::conti_radar_can_send()
{
    RadarConfig200 radar_config_200;
 /*
    int s, nbytes;
    static int8_t send_switch = 0;
    struct sockaddr_can addr;
    struct ifreq ifr;
    s= socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字
    strcpy(ifr.ifr_name, "can0" );
    ioctl(s, SIOCGIFINDEX, &ifr); //指定 can0 设备
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr));//将套接字与 can0 绑定
    */
    // 禁用过滤规则，本进程不接收报文，只负责发送
    // 如果应用程序不需要接收报文，可以禁用过滤规则。这样的话，原始套接字就会忽略所有接收到的报文。在这种仅仅发送数据的应用中，可以在内核中省略接收队列，以此减少 CPU 资源的消耗。
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    struct can_frame can_send_msg;
    can_send_msg.can_id = 0x200;
    can_send_msg.can_dlc = 8;
    radar_config_200.UpdateData(can_send_msg.data);
    nbytes = write(s, &can_send_msg, sizeof(can_send_msg));
    if (nbytes != sizeof(can_send_msg))
    {
        printf("Send Error can_send_msg\n!");
    }
}


void ContiRadarCan::conti_can_bus_info_to_sensor()
{
    if(conti_radar_.conti_radar_obs.size()<=conti_radar_.object_list_status.nof_objects)
    {
        contiRadarCb_(conti_radar_);
        /*send_conti_radar_objs.num_of_conti_radar=conti_radar_.object_list_status.nof_objects;
        send_conti_radar_objs.meas_counter=conti_radar_.object_list_status.meas_counter;
        cout<<" num_of_objects: "<< send_conti_radar_objs.num_of_conti_radar<<endl;
        for (int i=0; i<conti_radar_.conti_radar_obs.size();i++)
        {
            sensor_lcm::conti_radar_obj conti_obj_info;
            conti_obj_info.obstacle_id=conti_radar_.conti_radar_obs[i].obstacle_id;
            conti_obj_info.dynprop=conti_radar_.conti_radar_obs[i].dynprop;
            conti_obj_info.lateral_accel=conti_radar_.conti_radar_obs[i].lateral_accel;
            conti_obj_info.lateral_accel_rms=conti_radar_.conti_radar_obs[i].lateral_accel_rms;
            conti_obj_info.lateral_dist=conti_radar_.conti_radar_obs[i].lateral_dist;
            conti_obj_info.lateral_dist_rms=conti_radar_.conti_radar_obs[i].lateral_dist_rms;
            conti_obj_info.lateral_vel=conti_radar_.conti_radar_obs[i].lateral_vel;
            conti_obj_info.lateral_vel_rms=conti_radar_.conti_radar_obs[i].lateral_vel_rms;
            conti_obj_info.length=conti_radar_.conti_radar_obs[i].length;
            conti_obj_info.longitude_accel=conti_radar_.conti_radar_obs[i].longitude_accel;
            conti_obj_info.longitude_accel_rms=conti_radar_.conti_radar_obs[i].longitude_accel_rms;
            conti_obj_info.longitude_dist=conti_radar_.conti_radar_obs[i].longitude_dist;
            conti_obj_info.longitude_dist_rms=conti_radar_.conti_radar_obs[i].longitude_dist_rms;
            conti_obj_info.longitude_vel=conti_radar_.conti_radar_obs[i].longitude_vel;
            conti_obj_info.longitude_vel_rms=conti_radar_.conti_radar_obs[i].longitude_vel_rms;
            conti_obj_info.meas_state=conti_radar_.conti_radar_obs[i].meas_state;
            conti_obj_info.obstacle_class=conti_radar_.conti_radar_obs[i].obstacle_class;
            conti_obj_info.oritation_angle=conti_radar_.conti_radar_obs[i].oritation_angle;
            conti_obj_info.oritation_angle_rms=conti_radar_.conti_radar_obs[i].oritation_angle_rms;
            conti_obj_info.probexist=conti_radar_.conti_radar_obs[i].probexist;
            conti_obj_info.rcs=conti_radar_.conti_radar_obs[i].rcs;
            conti_obj_info.width=conti_radar_.conti_radar_obs[i].width;

            send_conti_radar_objs.conti_radar_objs.push_back(conti_obj_info);

        }

        g_conti_radar_can_lcm->publish("conti_radar_obj_list",&send_conti_radar_objs);
        send_conti_radar_objs.conti_radar_objs.clear();
        send_conti_radar_objs.num_of_conti_radar=0;
        send_conti_radar_objs.meas_counter=0;*/
    }
}

void ContiRadarCan::setContiRadarRecvCallback(ContiRadarCan::ContiRadarCb cb)
{
    contiRadarCb_ = cb;
}