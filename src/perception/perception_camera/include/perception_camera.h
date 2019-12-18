#ifndef _PERCEPTION_FUSION_H_
#define _PERCEPTION_FUSION_H_

#include <map>
#include <omp.h>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <common_msgs/ObstacleInfo.h>
#include <common_msgs/DetectionInfo.h>
#include <perception_sensor_msgs/ObjectList.h>
#include <perception_msgs/FusionDataInfo.h>
#include <perception_camera/CameraObstacle.h>

#include "utils/tools.h"
#include "sensor_object/base_object.h"
#include "associate/base_association.h"

using namespace std;

// #define DEBUG_PERCEPTION_FUSION

namespace sensor_camera
{
    class PerceptionCamera
    {
    public:
        explicit PerceptionCamera(
            BaseAssociation* base_associatio, 
            string calibration_file,
            string image_obstacle_topic,
            string lidar_topic,
            string pub_obstacle_info_topic,
            string pub_rviz_bounding_box_topic,
            string pub_rviz_bounding_box_info_topic,
            string pub_rviz_split_pointcloud_with_camera_fov_topic,
            string pub_rviz_split_pointcloud_with_image_obstacle_info_topic,
            bool is_draw
        );

        ~PerceptionCamera();    

    private:
        void callbackcamera(const perception_camera::CameraObstacle::ConstPtr& camera_obstacle_msg, const sensor_msgs::PointCloud2::ConstPtr& msg_lidar);

        void updateNewObject(vector<sensor_camera::BaseObject*>& obj_list);
        void updateAssociatedObject(vector<sensor_camera::BaseObject*>& new_obj, Eigen::MatrixXd& matrix);
        void updateUnassociatedObject(vector<sensor_camera::BaseObject*>& new_obj, Eigen::MatrixXd& matrix);
        void publishFusionObject(ros::Time pub_time, bool is_draw);

        void projectLidarPoints(const sensor_msgs::PointCloud2& msg_lidar);
        void splitPointCloudWithImageObstacleInfo(vector<BaseObject*>& base_object_list);

    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_obstacle_info_;
        ros::Publisher pub_rviz_bounding_box_;
        ros::Publisher pub_rviz_bounding_box_info_;
        ros::Publisher pub_rviz_split_pointcloud_with_camera_fov_;
        ros::Publisher pub_rviz_split_pointcloud_with_image_obstacle_info_;

        typedef message_filters::sync_policies::ApproximateTime<perception_camera::CameraObstacle, sensor_msgs::PointCloud2> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;

        boost::shared_ptr<Sync> sync_camera_;
        boost::shared_ptr<Sync> sync_lidar_;
        message_filters::Subscriber<perception_camera::CameraObstacle> sub_image_obstacle_info_;
        message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pointcloud_;

        BaseAssociation* base_association_;
        map<uint32_t, sensor_camera::BaseObject*> global_object_;

        uint32_t global_id_ = 0;
        bool is_draw_ = false;

        cv::Mat* intrinsic_;
        cv::Mat* distCoeffs_;
        cv::Mat* rvec_;
        cv::Mat* tvec_;
        float focus_ = 0.0;
        float fdx_ = 0.0;
        float fdy_ = 0.0;

        float alpha_ = -0 * 3.1415926 / 180;
        float fov_ = 60.0;
        const int camera_width_ = 1280.0;
        const int camera_height_ = 720.0;
        const int resize_width_ = 640.0;
        const int resize_height_ = 360.0;

        float* dist_image_ = new float[resize_height_*resize_width_*3];
    };
}

#endif // _PERCEPTION_FUSION_H_