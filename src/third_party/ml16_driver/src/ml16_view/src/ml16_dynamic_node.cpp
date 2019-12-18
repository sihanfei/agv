#include<ros/ros.h> //generic C++ stuff
#include <dynamic_reconfigure/server.h>
#include <ml16_view/dynamicConfig.h>
#include "ml16_view/TanwayML.h"

TanwayML ml16;
void dynamic_callback(ml16_view::dynamicConfig &config)
{
    ml16.StartAngle = config.StartAngle;
    ml16.EndAngle = config.EndAngle;
    ml16.StaticQuantityFirst[0] = config.StaticQuantityFirst_1;
    ml16.StaticQuantityFirst[1] = config.StaticQuantityFirst_2;
    ml16.StaticQuantityFirst[2] = config.StaticQuantityFirst_3;
    ml16.StaticQuantityFirst[3] = config.StaticQuantityFirst_4;
    ml16.StaticQuantityFirst[4] = config.StaticQuantityFirst_5;
    ml16.StaticQuantityFirst[5] = config.StaticQuantityFirst_6;
    ml16.StaticQuantityFirst[6] = config.StaticQuantityFirst_7;
    ml16.StaticQuantityFirst[7] = config.StaticQuantityFirst_8;
    ml16.StaticQuantityFirst[8] = config.StaticQuantityFirst_9;
    ml16.StaticQuantityFirst[9] = config.StaticQuantityFirst_10;
    ml16.StaticQuantityFirst[10] = config.StaticQuantityFirst_11;
    ml16.StaticQuantityFirst[11] = config.StaticQuantityFirst_12;
    ml16.StaticQuantityFirst[12] = config.StaticQuantityFirst_13;
    ml16.StaticQuantityFirst[13] = config.StaticQuantityFirst_14;
    ml16.StaticQuantityFirst[14] = config.StaticQuantityFirst_15;
    ml16.StaticQuantityFirst[15] = config.StaticQuantityFirst_16;
    ml16.transformCloud_Status = config.transformCloud_Status;
    ml16.trans_x = config.trans_x;
    ml16.trans_y = config.trans_y;
    ml16.trans_z = config.trans_z;
    ml16.rotate_theta_xy = config.rotate_theta_xy;
    ml16.rotate_theta_xz = config.rotate_theta_xz;
    ml16.rotate_theta_yz = config.rotate_theta_yz;

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ml16node"); //node name
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::Rate r(100);
    ml16.getParam(nh,nh_private);
    dynamic_reconfigure::Server<ml16_view::dynamicConfig> server;
    dynamic_reconfigure::Server<ml16_view::dynamicConfig>::CallbackType callback;
 
    callback = boost::bind(&dynamic_callback, _1);
    server.setCallback(callback);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_clr_ptr_n1(new pcl::PointCloud<pcl::PointXYZRGB>);

    ROS_INFO( "ML16_VIEWER_TEST for ROS" );
    ROS_INFO( "Test Version 2.7" );
    ROS_INFO( "Update Date: 2019/03/04\n" );

    ROS_INFO( "View in rviz;");
    ROS_INFO( "Choose: topic= ml16_cloud and fixed frame= TanwayML16");

    while (ros::ok()) {
	ml16.publishCloud(point_cloud_clr_ptr_n1);
   	ros::spinOnce();
	}
    return 0;
}
