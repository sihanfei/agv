#include <math.h>
#include <iostream>
#include "utils/tools.h"
#include "sensor_object/base_object.h"
#include "sensor_object/lidar_object.h"
#include "filter/normal_kalman_filter.h"

using namespace std;

// transform "ObjectList" to "BaseObject"
void sensor_lidar::inputTypeTransform(list<sensor_lidar::obstacleFeature>& obstacleFeatureList, vector<sensor_lidar::BaseObject*> &sensor_obj_list, const ros::Time& sub_time)
{
#ifdef DEBUG_TOOLS
    cout << "inputTypeTransform: start" << endl;
#endif

    for(std::list<sensor_lidar::obstacleFeature>::iterator it = obstacleFeatureList.begin(); it != obstacleFeatureList.end(); it++)
    {   
        boost::array<float, NUM_STATE> state = {it->xMin_, it->yMin_, it->xMax_, it->yMax_};
        boost::array<float, NUM_STATE*NUM_STATE> measurement_cov = {
            10000, 0, 0, 0, 0, 0, 
            0, 10000, 0, 0, 0, 0, 
            0, 0, 10000, 0, 0, 0,
            0, 0, 0, 10000, 0, 0,
            0, 0, 0, 0, 20000, 0,
            0, 0, 0, 0, 0, 20000
        };

        float x1 = it->x1_;
        float y1 = it->y1_;
        float x2 = it->x2_;
        float y2 = it->y2_;
        float x3 = it->x3_;
        float y3 = it->y3_;
        float x4 = it->x4_;
        float y4 = it->y4_;

        sensor_lidar::BaseFilter* filter = new sensor_lidar::NormalKalmanFilter(state, measurement_cov);
        sensor_lidar::LidarObject* lidar_object = new sensor_lidar::LidarObject(sub_time, 0, 1.0, 0.0, filter, x1, y1, x2, y2, x3, y3, x4, y4);
        sensor_obj_list.push_back(lidar_object);
        // delete lidar_object;
    }

#ifdef DEBUG_TOOLS
    cout << "inputTypeTransform: end" << endl;
#endif
}

// transform "BaseObject" to "FusionDataInfo"
void sensor_lidar::outputTypeTransform(const map<uint32_t, sensor_lidar::BaseObject*> &g_map, perception_sensor_msgs::ObjectList& pub_obj_list, ros::Time pub_time)
{
#ifdef DEBUG_TOOLS
    cout << "outputTypeTransform: start" << endl;
#endif

    map<uint32_t, BaseObject*>::const_iterator it = g_map.begin();

    pub_obj_list.header.stamp = pub_time;
    pub_obj_list.sensor_type = 1;
    pub_obj_list.obstacle_num = g_map.size();

    for(it; it != g_map.end(); it++)
    {
        common_msgs::DetectionInfo obstacle;       
        obstacle.id = it->first;
        obstacle.obj_class = 0;
        obstacle.confidence = 1.0;
        Eigen::VectorXf state = it->second->getState();
        float xmin = state(0);
        float ymin = state(1);
        float xmax = state(2);
        float ymax = state(3);
        obstacle.state[0] = state(0);
        obstacle.state[1] = state(1);
        obstacle.state[2] = state(2);
        obstacle.state[3] = state(3);
        obstacle.state[4] = state(4);
        obstacle.state[5] = state(5);

        obstacle.measurement_cov[0] = 10000;
        obstacle.measurement_cov[1] = 0;
        obstacle.measurement_cov[2] = 0;
        obstacle.measurement_cov[3] = 0;
        obstacle.measurement_cov[4] = 0;
        obstacle.measurement_cov[5] = 0;

        obstacle.measurement_cov[6]  = 0;
        obstacle.measurement_cov[7]  = 10000;
        obstacle.measurement_cov[8]  = 0;
        obstacle.measurement_cov[9]  = 0;
        obstacle.measurement_cov[10] = 0;
        obstacle.measurement_cov[11] = 0;

        obstacle.measurement_cov[12] = 0;
        obstacle.measurement_cov[13] = 0;
        obstacle.measurement_cov[14] = 10000;
        obstacle.measurement_cov[15] = 0;
        obstacle.measurement_cov[16] = 0;
        obstacle.measurement_cov[17] = 0;

        obstacle.measurement_cov[18] = 0;
        obstacle.measurement_cov[19] = 0;
        obstacle.measurement_cov[20] = 0;
        obstacle.measurement_cov[21] = 10000;
        obstacle.measurement_cov[22] = 0;
        obstacle.measurement_cov[23] = 0;

        obstacle.measurement_cov[24] = 0;
        obstacle.measurement_cov[25] = 0;
        obstacle.measurement_cov[26] = 0;
        obstacle.measurement_cov[27] = 0;
        obstacle.measurement_cov[28] = 20000;
        obstacle.measurement_cov[29] = 0;

        obstacle.measurement_cov[30] = 0;
        obstacle.measurement_cov[31] = 0;
        obstacle.measurement_cov[32] = 0;
        obstacle.measurement_cov[33] = 0;
        obstacle.measurement_cov[34] = 0;
        obstacle.measurement_cov[35] = 20000;

        obstacle.peek[0].x = it->second->x1_;
        obstacle.peek[0].y = it->second->y1_;

        obstacle.peek[1].x = it->second->x2_;
        obstacle.peek[1].y = it->second->y2_;

        obstacle.peek[2].x = it->second->x3_;
        obstacle.peek[2].y = it->second->y3_;

        obstacle.peek[3].x = it->second->x4_;
        obstacle.peek[3].y = it->second->y4_;
        
        pub_obj_list.object_list.push_back(obstacle);
    }

#ifdef DEBUG_TOOLS
    cout << "outputTypeTransform: end" << endl;
#endif
}

void sensor_lidar::showResultInRviz(const map<uint32_t, sensor_lidar::BaseObject*>& g_map, pcl::PointCloud<pcl::PointXYZRGB>& show_point, sensor_msgs::PointCloud2& msg_point)
{
    map<uint32_t, BaseObject*>::const_iterator it;
    for(it = g_map.begin(); it != g_map.end(); it++)
    {
        // Eigen::VectorXf state = it->second->getState();
        // float xmin = state(0);
        // float ymin = state(1);
        // float xmax = state(2);
        // float ymax = state(3);
        // float xmin = it->second->x3_;
        // float ymin = it->second->y3_;
        // float xmax = it->second->x1_;
        // float ymax = it->second->y1_;

        // cout << "xmin = " << xmin << endl;
        // cout << "ymin = " << ymin << endl;
        // cout << "xmax = " << xmax << endl;
        // cout << "ymax = " << ymax << endl;

        sensor_lidar::drawLine(it->second->x1_, it->second->y1_, 0, it->second->x2_, it->second->y2_, 0, show_point);
        sensor_lidar::drawLine(it->second->x2_, it->second->y2_, 0, it->second->x3_, it->second->y3_, 0, show_point);
        sensor_lidar::drawLine(it->second->x3_, it->second->y3_, 0, it->second->x4_, it->second->y4_, 0, show_point);
        sensor_lidar::drawLine(it->second->x4_, it->second->y4_, 0, it->second->x1_, it->second->y1_, 0, show_point);
    }

    pcl::toROSMsg(show_point, msg_point);
    msg_point.header.frame_id = "/odom";
    msg_point.header.stamp = ros::Time::now();
}

void sensor_lidar::drawLine(float x1, float y1, float z1, float x2, float y2, float z2, pcl::PointCloud<pcl::PointXYZRGB>&  pointCloudBoundingBox)
{
    float r = (x2-x1) * (x2-x1) + (y2-y1) * (y2-y1) + (z2-z1) * (z2-z1);
    for (float i = 0; i <= r; i += 0.2)
    {
        uint8_t red = 255, green = 0, blue = 0;
        uint32_t rgb = ((uint32_t)red << 16 | (uint32_t)green << 8 | (uint32_t)blue);
        pcl::PointXYZRGB point;
        point.x = x1 + i / r * (x2 - x1);
        point.y = y1 + i / r * (y2 - y1);
        point.z = z1 + i / r * (z2 - z1);
        point.rgb = *reinterpret_cast<float*>(&rgb);
        pointCloudBoundingBox.points.push_back(point);
    }
}
