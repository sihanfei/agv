#include <ros/ros.h> //generic C++ stuff
#include <stdlib.h>
#include <math.h>
#include <ros/console.h>
#include "ml16_view/TanwayML.h"



int main(int argc, char** argv) {
    ros::init(argc, argv, "ml16node"); //node name
    ros::NodeHandle nh;
    ros::Rate r(10);

    ros::NodeHandle nh_private("~");

    ROS_INFO( "ML16_VIEWER_TEST for ROS" );
    ROS_INFO( "Test Version 2.6" );
    ROS_INFO( "Update Date: 2019/02/13\n" );

    ROS_INFO( "View in rviz;");
    ROS_INFO( "Choose: topic= ml16_cloud and fixed frame= TanwayML16");


    int PublishCount = 0;
    int pre_PublishCount = 0;
    int min_count=1;

    TanwayML ml16;
    ml16.getParam(nh,nh_private);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_clr_ptr_n1(new pcl::PointCloud<pcl::PointXYZRGB>);

    while (ros::ok()) {
	ml16.publishCloud(point_cloud_clr_ptr_n1);
	}
    return 0;
}
