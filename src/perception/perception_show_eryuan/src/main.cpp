#include <ros/ros.h>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <common_msgs/ObstacleInfo.h>
#include <perception_sensor_msgs/ObjectList.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace Eigen;

ros::Publisher pub_rviz_bounding_box;

void drawLine(float x1, float y1, float z1, float x2, float y2, float z2, pcl::PointCloud<pcl::PointXYZRGB>&  pointCloudBoundingBox)
{
    float r = (x2-x1) * (x2-x1) + (y2-y1) * (y2-y1) + (z2-z1) * (z2-z1);
    for (float i = 0; i <= r; i += 0.2)
    {
        uint8_t red = 255, green = 0, blue = 255;
        uint32_t rgb = ((uint32_t)red << 16 | (uint32_t)green << 8 | (uint32_t)blue);
        pcl::PointXYZRGB point;
        point.x = x1 + i / r * (x2 - x1);
        point.y = y1 + i / r * (y2 - y1);
        point.z = z1 + i / r * (z2 - z1);
        point.rgb = *reinterpret_cast<float*>(&rgb);
        pointCloudBoundingBox.points.push_back(point);
    }
}

void showResultInRviz(const perception_sensor_msgs::ObjectList& object_list, pcl::PointCloud<pcl::PointXYZRGB>& show_point, sensor_msgs::PointCloud2& msg_point)
{
    for(int i = 0; i < object_list.object_list.size(); i++)
    {
        drawLine(object_list.object_list[i].peek[0].x, object_list.object_list[i].peek[0].y, 0, object_list.object_list[i].peek[1].x, object_list.object_list[i].peek[1].y, 0, show_point);
        drawLine(object_list.object_list[i].peek[1].x, object_list.object_list[i].peek[1].y, 0, object_list.object_list[i].peek[2].x, object_list.object_list[i].peek[2].y, 0, show_point);
        drawLine(object_list.object_list[i].peek[2].x, object_list.object_list[i].peek[2].y, 0, object_list.object_list[i].peek[3].x, object_list.object_list[i].peek[3].y, 0, show_point);
        drawLine(object_list.object_list[i].peek[3].x, object_list.object_list[i].peek[3].y, 0, object_list.object_list[i].peek[0].x, object_list.object_list[i].peek[0].y, 0, show_point);
    }

    pcl::toROSMsg(show_point, msg_point);
    msg_point.header.frame_id = "/velodyne";
    msg_point.header.stamp = ros::Time::now();
}

void callback(const perception_sensor_msgs::ObjectList::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZRGB> show_point;
    sensor_msgs::PointCloud2 msg_point;
    showResultInRviz(*msg, show_point, msg_point);
    pub_rviz_bounding_box.publish(msg_point);
}

int main(int argc, char **argv)
{
    cout << "start perception_show_eryuan main: " << endl;

    ros::init(argc, argv, "perception_show_eryuan");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/drivers/perception/camera_obstacle_info", 1, callback);

    pub_rviz_bounding_box = nh.advertise<sensor_msgs::PointCloud2>("/perception/show_eryuan", 1);

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
