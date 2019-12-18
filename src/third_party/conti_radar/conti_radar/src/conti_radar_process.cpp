#include "conti_radar_process.hpp"
#include<iomanip>
#include <sys/time.h>
#include <signal.h>

ContiRadarProcess::ContiRadarProcess(const ros::NodeHandle& nh_private,const ros::NodeHandle nh_node)
    :nh_private_(nh_private)
    ,nh_node_(nh_node)
{
    nh_private_.getParam("config_file", config_file_);
    nh_private_.getParam("frame_id", frame_id_);

    ROS_INFO("config_file: %s",config_file_.c_str());
    ROS_INFO("frame_id: %s",frame_id_.c_str());

    Config configSettings(config_file_);
    can_channel_=configSettings.Read("can_channel",can_channel_);
   
    ROS_INFO("can_channel: %s",can_channel_.c_str());

    pubContiRadarMsg_ = nh_node_.advertise<conti_radar_msgs::ContiRadarObjList>("conti_radar",1);
}

ContiRadarProcess::~ContiRadarProcess()
{


}

int ContiRadarProcess::conti_radar_start()
{
   //新建一个线程，用于接收radar的消息
   boost::thread captureImageThread(boost::bind(&ContiRadarProcess::onContiRadarRecv,this));	
   //pthread_create(&(conti_radar_th),NULL,&(Thread_conti_radar_can_recv_Function),NULL); // can 数据接收线程
   return 0;
}

void ContiRadarProcess::conti_radar_stop()
{

}

//回调函数
int ContiRadarProcess::onRadarRecvCallback(const ContiRadar& conti_radar_)
{
    //将该值赋值给conti_radar_msgs::ContiRadarObjList，然后发布
    contiRadarObjList_.num_of_conti_radar=conti_radar_.object_list_status.nof_objects;
    contiRadarObjList_.meas_counter=conti_radar_.object_list_status.meas_counter;
    contiRadarObjList_.header.stamp = ros::Time::now();
    contiRadarObjList_.header.frame_id = frame_id_;

    cout<<" num_of_objects: "<< contiRadarObjList_.num_of_conti_radar<<endl;
    for (int i=0; i<conti_radar_.conti_radar_obs.size();i++)
    {
        conti_radar_msgs::ContiRadarObj conti_obj_info;
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

        contiRadarObjList_.conti_radar_objs.push_back(conti_obj_info);

    }
    pubContiRadarMsg_.publish(contiRadarObjList_);
    contiRadarObjList_.conti_radar_objs.clear();
    contiRadarObjList_.num_of_conti_radar=0;
    contiRadarObjList_.meas_counter=0;
}


void ContiRadarProcess::onContiRadarRecv()
{
    conti_radar_can_.conti_radar_can_init(can_channel_);
    conti_radar_can_.setContiRadarRecvCallback(boost::bind(&ContiRadarProcess::onRadarRecvCallback, this, _1));
    while(ros::ok())
    {
        auto start =system_clock::now();
        conti_radar_can_.conti_radar_can_Recv();
        auto end_=system_clock::now();
        auto duration=duration_cast<microseconds>(end_-start);
        double delta=(double)duration.count()* microseconds::period::num / microseconds::period::den;
    }
}

int ContiRadarProcess::init()
{
    
}






