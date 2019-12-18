#include <ros/ros.h>
#include "perception_fusion.h"
#include "associate/base_association.h"
#include "associate/max_association.h"
#include "associate/hungarian_association.cpp"


ros::Publisher pub;

int main(int argc, char **argv)
{

    cout << "start perception_fusion main: " << endl;

    ros::init(argc, argv, "perception_fusion");
    ros::NodeHandle nh;
    ros::NodeHandle private_node("~");
    pub = nh.advertise<sensor_msgs::PointCloud2>("/perception_fusion/rviz/pub_global", 1);

    // BaseAssociation* base_association = new MaxAssociation(0.5);
    BaseAssociation* base_association = new HungarianAssociation(0.05);

    float velocity_threshold = 0.3;
    // bool is_draw = true;
    int is_draw = 0, global_rviz = 1;
    private_node.param<int>("is_draw", is_draw, 0);
    private_node.param<int>("global_rviz", global_rviz, 1);
    double range_xmax = 5.0, range_xmin = -5.0, range_ymax = 15.0, range_ymin = -15.0;
    private_node.param<double>("range_xmax", range_xmax, 5.0);
    private_node.param<double>("range_xmin", range_xmin, -5.0);
    private_node.param<double>("range_ymax", range_ymax, 15.0);
    private_node.param<double>("range_ymin", range_ymin, -15.0);
    double agv_xmax = 1.8, agv_xmin = -1.8, agv_ymax = 8.0, agv_ymin = -8.0;
    private_node.param<double>("agv_xmax", agv_xmax, 1.8);
    private_node.param<double>("agv_xmin", agv_xmin, -1.8);
    private_node.param<double>("agv_ymax", agv_ymax, 8.0);
    private_node.param<double>("agv_ymin", agv_ymin, -8.0);
    // cout << "1. global_rviz = " << global_rviz << endl;
    // if (argc > 1 && strcmp(argv[1], "draw_bounding_box") == 0)
    // {
    //     is_draw = true;
    // }  

    PerceptionFusion perception_fusion(base_association, velocity_threshold, is_draw, global_rviz,
    range_xmax, range_xmin, range_ymax, range_ymin, agv_xmax, agv_xmin, agv_ymax, agv_ymin);

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}