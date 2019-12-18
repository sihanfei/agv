#include <ros/ros.h>
#include "perception_camera.h"
#include "associate/base_association.h"
#include "associate/max_association.h"
#include "associate/hungarian_association.cpp"

#include <vector>

#define cam_index 1
#define lidar_index 1

int main(int argc, char **argv)
{

    cout << "start perception_camera main: " << endl;

    ros::init(argc, argv, "perception_camera");

    string config_path = "";
    ros::param::get("config_path", config_path);
    string cam_name = "";
    ros::param::get("cam_name", cam_name);
    string lidar_name = "";
    ros::param::get("lidar_name", lidar_name);
    config_path = config_path + lidar_name + "_to_" + cam_name + ".yaml";

    string image_obstacle_topic = "";
    ros::param::get("image_obstacle_topic", image_obstacle_topic);
    string lidar_topic = "";
    ros::param::get("lidar_topic", lidar_topic);
    string pub_obstacle_info_topic = "";
    ros::param::get("pub_obstacle_info_topic", pub_obstacle_info_topic);
    string pub_rviz_bounding_box_topic = "";
    ros::param::get("pub_rviz_bounding_box_topic", pub_rviz_bounding_box_topic);
    string pub_rviz_bounding_box_info_topic = "";
    ros::param::get("pub_rviz_bounding_box_info_topic", pub_rviz_bounding_box_info_topic);
    string pub_rviz_split_pointcloud_with_camera_fov_topic = "";
    ros::param::get("pub_rviz_split_pointcloud_with_camera_fov_topic", pub_rviz_split_pointcloud_with_camera_fov_topic);
    string pub_rviz_split_pointcloud_with_image_obstacle_info_topic = "";
    ros::param::get("pub_rviz_split_pointcloud_with_image_obstacle_info_topic", pub_rviz_split_pointcloud_with_image_obstacle_info_topic);

    // // BaseAssociation* base_association = new MaxAssociation(0.5);
    sensor_camera::BaseAssociation* base_association = new sensor_camera::HungarianAssociation(0.2);

    bool is_draw = true;

    sensor_camera::PerceptionCamera perception_camera(
        base_association, 
        config_path,
        image_obstacle_topic,
        lidar_topic,
        pub_obstacle_info_topic,
        pub_rviz_bounding_box_topic,
        pub_rviz_bounding_box_info_topic,
        pub_rviz_split_pointcloud_with_camera_fov_topic,
        pub_rviz_split_pointcloud_with_image_obstacle_info_topic,
        is_draw
    );

    while (ros::ok())
    {
        ros::spinOnce();
    }
    
    return 0;
}