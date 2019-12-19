#include <math.h>
#include <iostream>
#include "utils/tools.h"
#include "sensor_object/base_object.h"
#include "sensor_object/camera_object.h"
#include "filter/normal_kalman_filter.h"

using namespace std;

// transform "ObjectList" to "BaseObject"
void sensor_camera::inputTypeTransform(const perception_camera::CameraObstacle& camera_obstacle_msg, vector<sensor_camera::BaseObject*> &sensor_obj_list, ros::Time sub_time)
{
#ifdef DEBUG_TOOLS
    cout << "inputTypeTransform: start" << endl;
#endif
    
    for(int i = 0; i < camera_obstacle_msg.confidence.size(); i++)
    {   
        if(camera_obstacle_msg.xmin[i] < 0.01)
        {
            continue;
        }
        if(camera_obstacle_msg.object_class[i] != 2 && camera_obstacle_msg.object_class[i] != 4 && camera_obstacle_msg.object_class[i] != 5)
        {
            continue;
        }

        boost::array<float, NUM_STATE> pix_state = {
            camera_obstacle_msg.xmin[i] < 0 ? 0 : camera_obstacle_msg.xmin[i],
            camera_obstacle_msg.ymin[i] < 0 ? 0 : camera_obstacle_msg.ymin[i],
            camera_obstacle_msg.xmax[i] > 1279 ? 1280 : camera_obstacle_msg.xmax[i],
            camera_obstacle_msg.ymax[i] > 719 ? 719 : camera_obstacle_msg.ymax[i]
        };
        boost::array<float, NUM_STATE*NUM_STATE> pix_measurement_cov = {
            1, 0, 0, 0, 
            0, 1, 0, 0, 
            0, 0, 1, 0,
            0, 0, 0, 1
        };
        sensor_camera::BaseFilter* pix_filter = new sensor_camera::NormalKalmanFilter(pix_state, pix_measurement_cov);

        float vx = 0.0;
        float vy = 0.0;
        boost::array<float, NUM_STATE> world_state = {
            camera_obstacle_msg.x[i], 
            camera_obstacle_msg.y[i],
            vx,
            vy
        };
        boost::array<float, NUM_STATE*NUM_STATE> world_measurement_cov = {
            10000, 0, 0, 0,
            0, 10000, 0, 0,
            0, 0, 20000, 0,
            0, 0, 0, 20000
        };
        sensor_camera::BaseFilter* world_filter = new sensor_camera::NormalKalmanFilter(world_state, world_measurement_cov);
        
        float state_3d[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        getWorldBbox(
            camera_obstacle_msg.x[i], 
            camera_obstacle_msg.y[i], 
            camera_obstacle_msg.w[i], 
            camera_obstacle_msg.h[i],
            camera_obstacle_msg.object_class[i],
            state_3d
        );

        sensor_obj_list.push_back(
            new sensor_camera::CameraObject(
                sub_time, 
                camera_obstacle_msg.object_class[i], 
                camera_obstacle_msg.confidence[i], 
                camera_obstacle_msg.confidence[i], 
                world_filter,
                pix_filter,
                state_3d
            )
        );
    }

#ifdef DEBUG_TOOLS
    cout << "inputTypeTransform: end" << endl;
#endif
}

// transform "BaseObject" to "FusionDataInfo"
void sensor_camera::outputTypeTransform(const map<uint32_t, sensor_camera::BaseObject*> &g_map, perception_sensor_msgs::ObjectList& pub_obj_list, ros::Time pub_time)
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
        Eigen::VectorXf state_3d = it->second->getState3d();
        Eigen::VectorXf world_state = it->second->getWorldState();
        float xmin = state_3d(0);
        float ymin = state_3d(1);
        float xmax = state_3d(2);
        float ymax = state_3d(3);
        obstacle.state[0] = state_3d(0);
        obstacle.state[1] = state_3d(1);
        obstacle.state[2] = state_3d(2);
        obstacle.state[3] = state_3d(3);
        obstacle.state[4] = world_state(2);
        obstacle.state[5] = world_state(3);

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

        obstacle.peek[0].x = xmax;
        obstacle.peek[0].y = ymax;

        obstacle.peek[1].x = xmax;
        obstacle.peek[1].y = ymin;

        obstacle.peek[2].x = xmin;
        obstacle.peek[2].y = ymin;

        obstacle.peek[3].x = xmin;
        obstacle.peek[3].y = ymax;
        
        pub_obj_list.object_list.push_back(obstacle);
    }

#ifdef DEBUG_TOOLS
    cout << "outputTypeTransform: end" << endl;
#endif
}

void sensor_camera::getWorldBbox(float x, float y, float w, float h, uint8_t classes, float state_3d[6])
{
    if(classes == 2)
    {
        state_3d[0] = x;
        state_3d[1] = y - w/2;
        state_3d[2] = x + 0.8;
        state_3d[3] = y + w/2;
    }
    else if(classes == 4)
    {
        state_3d[0] = x;
        state_3d[1] = y - w/2;
        state_3d[2] = x + 4.0;
        state_3d[3] = y + w/2;
    }
    else if(classes == 5)
    {
        state_3d[0] = x;
        state_3d[1] = y - w/2;
        state_3d[2] = x + 1.5;
        state_3d[3] = y + w/2;
    }
    state_3d[4] = w;
    state_3d[5] = h;
}

void sensor_camera::showResultInRviz(const map<uint32_t, sensor_camera::BaseObject*>& g_map, pcl::PointCloud<pcl::PointXYZRGB>& show_point, sensor_msgs::PointCloud2& msg_point)
{
    map<uint32_t, BaseObject*>::const_iterator it;
    for(it = g_map.begin(); it != g_map.end(); it++)
    {
        Eigen::VectorXf state_3d = it->second->getState3d();
        float xmin = state_3d(0);
        float ymin = state_3d(1);
        float xmax = state_3d(2);
        float ymax = state_3d(3);

        sensor_camera::drawLine(xmax, ymax, 0, xmax, ymin, 0, show_point);
        sensor_camera::drawLine(xmax, ymin, 0, xmin, ymin, 0, show_point);
        sensor_camera::drawLine(xmin, ymin, 0, xmin, ymax, 0, show_point);
        sensor_camera::drawLine(xmin, ymax, 0, xmax, ymax, 0, show_point);
    }

    pcl::toROSMsg(show_point, msg_point);
    msg_point.header.frame_id = "/velodyne";
    msg_point.header.stamp = ros::Time::now();
}

void sensor_camera::drawLine(float x1, float y1, float z1, float x2, float y2, float z2, pcl::PointCloud<pcl::PointXYZRGB>&  pointCloudBoundingBox)
{
    float r = (x2-x1) * (x2-x1) + (y2-y1) * (y2-y1) + (z2-z1) * (z2-z1);
    for (float i = 0; i <= r; i += 0.2)
    {
        uint8_t red = 0, green = 0, blue = 255;
        uint32_t rgb = ((uint32_t)red << 16 | (uint32_t)green << 8 | (uint32_t)blue);
        pcl::PointXYZRGB point;
        point.x = x1 + i / r * (x2 - x1);
        point.y = y1 + i / r * (y2 - y1);
        point.z = z1 + i / r * (z2 - z1);
        point.rgb = *reinterpret_cast<float*>(&rgb);
        pointCloudBoundingBox.points.push_back(point);
    }
}