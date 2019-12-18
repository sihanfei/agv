#include <math.h>
#include <iostream>
#include "utils/tools.h"
#include "sensor_object/base_object.h"
#include "sensor_object/camera_object.h"
#include "sensor_object/lidar_object.h"
#include "filter/normal_kalman_filter.h"

using namespace std;

// transform "ObjectList" to "BaseObject"
void inputTypeTransform(const perception_sensor_msgs::ObjectList &obj_list, vector<BaseObject*> &sensor_obj_list, uint8_t sensor_type, float (&att)[3], Vector3d &veh_llh)
{
#ifdef DEBUG_TOOLS
    cout << "inputTypeTransform: start" << endl;
#endif

    for(int i = 0; i < obj_list.object_list.size(); i++)
    {
        BaseFilter* filter = new NormalKalmanFilter(obj_list.object_list[i].state, obj_list.object_list[i].measurement_cov);
        
        if(sensor_type == 0)
        {   
            float point4[8];
            point4[0] = obj_list.object_list[i].peek[0].x;
            point4[1] = obj_list.object_list[i].peek[0].y;
            point4[2] = obj_list.object_list[i].peek[1].x;
            point4[3] = obj_list.object_list[i].peek[1].y;
            point4[4] = obj_list.object_list[i].peek[2].x;
            point4[5] = obj_list.object_list[i].peek[2].y;
            point4[6] = obj_list.object_list[i].peek[3].x;
            point4[7] = obj_list.object_list[i].peek[3].y;
            sensor_obj_list.push_back(
                new CameraObject(
                    obj_list.object_list[i].id + 10000000,
                    obj_list.header.stamp,
                    obj_list.object_list[i].obj_class,
                    obj_list.object_list[i].confidence, 1.0, 
                    att, veh_llh,
                    filter, point4
                )
            );
        }
        else if(sensor_type == 1) 
        {
            float point4[8];
            point4[0] = obj_list.object_list[i].peek[0].x;
            point4[1] = obj_list.object_list[i].peek[0].y;
            point4[2] = obj_list.object_list[i].peek[1].x;
            point4[3] = obj_list.object_list[i].peek[1].y;
            point4[4] = obj_list.object_list[i].peek[2].x;
            point4[5] = obj_list.object_list[i].peek[2].y;
            point4[6] = obj_list.object_list[i].peek[3].x;
            point4[7] = obj_list.object_list[i].peek[3].y;
            sensor_obj_list.push_back(
                new LidarObject(
                    0,
                    obj_list.header.stamp,
                    obj_list.object_list[i].obj_class,
                    obj_list.object_list[i].confidence, 1.0, 
                    att, veh_llh,
                    filter, point4
                )
            );
        }
    }

#ifdef DEBUG_TOOLS
    cout << "inputTypeTransform: end" << endl;
#endif
}

extern ros::Publisher pub;

// transform "BaseObject" to "FusionDataInfo"
// void outputTypeTransform(const map<uint32_t, BaseObject*> &g_map, perception_msgs::FusionDataInfo &pub_obj_list, ros::Time pub_time, float veh_vxvy[2], float att[3])
// {
// #ifdef DEBUG_TOOLS
//     cout << "outputTypeTransform: start" << endl;
// #endif

//     map<uint32_t, BaseObject*>::const_iterator it = g_map.begin();

//     pub_obj_list.header.stamp = pub_time;
//     pub_obj_list.obstacle_num = g_map.size();

//     for(it; it != g_map.end(); it++)
//     {
//         common_msgs::ObstacleInfo obstacle;
//         obstacle.id = it->first;
//         float gx = it->second->veh_llh_(0);
//         float gy = it->second->veh_llh_(1);
//         float yaw = att[0];
//         // std::cout << "*****id = " << obstacle.id << std::endl;
//         // std::cout << "*****yaw(degree) = " << yaw << std::endl;
//         yaw = -yaw * M_PI / 180;
//         // std::cout << "*****gx = " << gx << std::endl;
//         // std::cout << "*****gy = " << gy << std::endl;
//         // std::cout << "*****yaw = " << yaw << std::endl;
//         Eigen::VectorXf state = it->second->getState();
//         // state(0) --> xmin
//         // state(1) --> ymin
//         // state(2) --> xmax
//         // state(3) --> ymax
//         // float x1 = state(0);
//         // float y1 = state(3);
//         // float x2 = state(2);
//         // float y2 = state(3);
//         // float x3 = state(2);
//         // float y3 = state(1);
//         // float x4 = state(0);
//         // float y4 = state(1);

//         float x1 = it->second->point4_[0];
//         float y1 = it->second->point4_[1];
//         float x2 = it->second->point4_[2];
//         float y2 = it->second->point4_[3];
//         float x3 = it->second->point4_[4];
//         float y3 = it->second->point4_[5];
//         float x4 = it->second->point4_[6];
//         float y4 = it->second->point4_[7];

//         obstacle.local_peak[0].x = x1;
//         obstacle.local_peak[0].y = y1;
//         obstacle.local_peak[1].x = x2;
//         obstacle.local_peak[1].y = y2;
//         obstacle.local_peak[2].x = x3;
//         obstacle.local_peak[2].y = y3;
//         obstacle.local_peak[3].x = x4;
//         obstacle.local_peak[3].y = y4;

//         float global_x1 = x1*cos(yaw)-y1*sin(yaw);
//         float global_y1 = y1*cos(yaw)+x1*sin(yaw);
//         float global_x2 = x2*cos(yaw)-y2*sin(yaw);
//         float global_y2 = y2*cos(yaw)+x2*sin(yaw);
//         float global_x3 = x3*cos(yaw)-y3*sin(yaw);
//         float global_y3 = y3*cos(yaw)+x3*sin(yaw);
//         float global_x4 = x4*cos(yaw)-y4*sin(yaw);
//         float global_y4 = y4*cos(yaw)+x4*sin(yaw);
//         global_x1 = global_x1 + gx;
//         global_y1 = global_y1 + gy;
//         global_x2 = global_x2 + gx;
//         global_y2 = global_y2 + gy;
//         global_x3 = global_x3 + gx;
//         global_y3 = global_y3 + gy;
//         global_x4 = global_x4 + gx;
//         global_y4 = global_y4 + gy;

//         // std::cout << "*****global_x1 = " << global_x1 << std::endl;
//         // std::cout << "*****global_y1 = " << global_y1 << std::endl;
//         // std::cout << "*****global_x2 = " << global_x2 << std::endl;
//         // std::cout << "*****global_y2 = " << global_y2 << std::endl;
//         // std::cout << "*****global_x3 = " << global_x3 << std::endl;
//         // std::cout << "*****global_y3 = " << global_y3 << std::endl;
//         // std::cout << "*****global_x4 = " << global_x4 << std::endl;
//         // std::cout << "*****global_y4 = " << global_y4 << std::endl;
//         obstacle.peak[0].x = global_x1;
//         obstacle.peak[0].y = global_y1;
//         obstacle.peak[1].x = global_x2;
//         obstacle.peak[1].y = global_y2;
//         obstacle.peak[2].x = global_x3;
//         obstacle.peak[2].y = global_y3;
//         obstacle.peak[3].x = global_x4;
//         obstacle.peak[3].y = global_y4;
        
//         // state(4) --> vx
//         // state(5) --> vy
//         // float vx = state(4)*cos(yaw)-state(5)*sin(yaw);
//         // float vy = state(5)*cos(yaw)+state(4)*sin(yaw);
//         // vx = vx + veh_vxvy[0];
//         // vy = vy + veh_vxvy[1];
//         float vx = state(4);
//         float vy = state(5);
//         // std::cout << "gvx = " << veh_vxvy[0] << std::endl;
//         // std::cout << "gvy = " << veh_vxvy[1] << std::endl;
//         // std::cout << "*****global_vx = " << vx << std::endl;
//         // std::cout << "*****global_vy = " << vy << std::endl;
//         float v = sqrt(pow(vx, 2) + pow(vy, 2));
//         obstacle.velocity = v > 0.2 ? v : 0.0;

//         if(vy == 0)
//         {
//             obstacle.theta = 0;
//         }
//         else
//         {
//             obstacle.theta = atan(vx / vy) * 180.0 / 3.14159;
//         }

//         pub_obj_list.obstacles.push_back(obstacle);
//     }

//     pcl::PointCloud<pcl::PointXYZRGB> show_point;
//     sensor_msgs::PointCloud2 msg_point;
//     showResultInRviz2(pub_obj_list ,show_point, msg_point);
//     pub.publish(msg_point);


// #ifdef DEBUG_TOOLS
//     cout << "outputTypeTransform: end" << endl;
// #endif
// }

// void showResultInRviz(const map<uint32_t, BaseObject*>& g_map, pcl::PointCloud<pcl::PointXYZRGB>& show_point, sensor_msgs::PointCloud2& msg_point, float att[3])
// {
//     bool gobal_rviz = true;
//     map<uint32_t, BaseObject*>::const_iterator it;
//     for(it = g_map.begin(); it != g_map.end(); it++)
//     {
//         float gx = it->second->veh_llh_(0);
//         float gy = it->second->veh_llh_(1);
//         float yaw = att[0];
//         yaw = -yaw * M_PI / 180;
//         Eigen::VectorXf state = it->second->getState();
//         // float xmin = state(0);
//         // float ymin = state(1);
//         // float xmax = state(2);
//         // float ymax = state(3);

//         // float x1 = state(0);
//         // float y1 = state(3);
//         // float x2 = state(2);
//         // float y2 = state(3);
//         // float x3 = state(2);
//         // float y3 = state(1);
//         // float x4 = state(0);
//         // float y4 = state(1);

//         float x1 = it->second->point4_[0];
//         float y1 = it->second->point4_[1];
//         float x2 = it->second->point4_[2];
//         float y2 = it->second->point4_[3];
//         float x3 = it->second->point4_[4];
//         float y3 = it->second->point4_[5];
//         float x4 = it->second->point4_[6];
//         float y4 = it->second->point4_[7];

//         float global_x1, global_y1, global_x2, global_y2, global_x3, global_y3, global_x4, global_y4;
//         if (gobal_rviz)
//         {
//             global_x1 = x1*cos(yaw)-y1*sin(yaw);
//             global_y1 = y1*cos(yaw)+x1*sin(yaw);
//             global_x2 = x2*cos(yaw)-y2*sin(yaw);
//             global_y2 = y2*cos(yaw)+x2*sin(yaw);
//             global_x3 = x3*cos(yaw)-y3*sin(yaw);
//             global_y3 = y3*cos(yaw)+x3*sin(yaw);
//             global_x4 = x4*cos(yaw)-y4*sin(yaw);
//             global_y4 = y4*cos(yaw)+x4*sin(yaw);
//             global_x1 = global_x1 + gx;
//             global_y1 = global_y1 + gy;
//             global_x2 = global_x2 + gx;
//             global_y2 = global_y2 + gy;
//             global_x3 = global_x3 + gx;
//             global_y3 = global_y3 + gy;
//             global_x4 = global_x4 + gx;
//             global_y4 = global_y4 + gy;
//         }
//         else
//         {
//             global_x1 = x1;
//             global_y1 = y1;
//             global_x2 = x2;
//             global_y2 = y2;
//             global_x3 = x3;
//             global_y3 = y3;
//             global_x4 = x4;
//             global_y4 = y4;
//         }
        
//         drawLine(global_x1, global_y1, 0, global_x2, global_y2, 0, show_point);
//         drawLine(global_x2, global_y2, 0, global_x3, global_y3, 0, show_point);
//         drawLine(global_x3, global_y3, 0, global_x4, global_y4, 0, show_point);
//         drawLine(global_x4, global_y4, 0, global_x1, global_y1, 0, show_point);
//     }

//     pcl::toROSMsg(show_point, msg_point);
//     msg_point.header.frame_id = "/odom";
//     msg_point.header.stamp = ros::Time::now();
// }

// void showResultInRviz2(perception_msgs::FusionDataInfo &pub_obj_list, pcl::PointCloud<pcl::PointXYZRGB>& show_point, sensor_msgs::PointCloud2& msg_point)
// {
//     for(int i = 0; i < pub_obj_list.obstacles.size(); i++)
//     {

//         drawLine(pub_obj_list.obstacles[i].peak[0].x, pub_obj_list.obstacles[i].peak[0].y, 0, pub_obj_list.obstacles[i].peak[1].x, pub_obj_list.obstacles[i].peak[1].y, 0, show_point);
//         drawLine(pub_obj_list.obstacles[i].peak[1].x, pub_obj_list.obstacles[i].peak[1].y, 0, pub_obj_list.obstacles[i].peak[2].x, pub_obj_list.obstacles[i].peak[2].y, 0, show_point);
//         drawLine(pub_obj_list.obstacles[i].peak[2].x, pub_obj_list.obstacles[i].peak[2].y, 0, pub_obj_list.obstacles[i].peak[3].x, pub_obj_list.obstacles[i].peak[3].y, 0, show_point);
//         drawLine(pub_obj_list.obstacles[i].peak[3].x, pub_obj_list.obstacles[i].peak[3].y, 0, pub_obj_list.obstacles[i].peak[0].x, pub_obj_list.obstacles[i].peak[0].y, 0, show_point);
//     }

//     pcl::toROSMsg(show_point, msg_point);
//     msg_point.header.frame_id = "/odom";
//     msg_point.header.stamp = ros::Time::now();
// }

void drawLine(float x1, float y1, float z1, float x2, float y2, float z2, pcl::PointCloud<pcl::PointXYZRGB>&  pointCloudBoundingBox)
{
    float r = (x2-x1) * (x2-x1) + (y2-y1) * (y2-y1) + (z2-z1) * (z2-z1);
    for (float i = 0; i <= r; i += 0.01)
    {
        uint8_t red = 0, green = 255, blue = 0;
        uint32_t rgb = ((uint32_t)red << 16 | (uint32_t)green << 8 | (uint32_t)blue);
        pcl::PointXYZRGB point;
        point.x = x1 + i / r * (x2 - x1);
        point.y = y1 + i / r * (y2 - y1);
        point.z = z1 + i / r * (z2 - z1);
        point.rgb = *reinterpret_cast<float*>(&rgb);
        pointCloudBoundingBox.points.push_back(point);
    }
}

void drawLineCamere(float x1, float y1, float z1, float x2, float y2, float z2, pcl::PointCloud<pcl::PointXYZRGB>&  pointCloudBoundingBox)
{
    float r = (x2-x1) * (x2-x1) + (y2-y1) * (y2-y1) + (z2-z1) * (z2-z1);
    for (float i = 0; i <= r; i += 0.01)
    {
        uint8_t red = 255, green = 127, blue = 255;
        uint32_t rgb = ((uint32_t)red << 16 | (uint32_t)green << 8 | (uint32_t)blue);
        pcl::PointXYZRGB point;
        point.x = x1 + i / r * (x2 - x1);
        point.y = y1 + i / r * (y2 - y1);
        point.z = z1 + i / r * (z2 - z1);
        point.rgb = *reinterpret_cast<float*>(&rgb);
        pointCloudBoundingBox.points.push_back(point);
    }
}

void removeObject(const ros::Time lidar_time, const ros::Time eryuan_time, map<uint32_t, BaseObject*> &g_map, map<uint32_t, BaseObject*> eryuan_object_list[], const int eryuan_number, map<uint32_t, BaseObject*> &publish_map)
{
    for (map<uint32_t, BaseObject*>::iterator it = g_map.begin();
    it != g_map.end(); it++)
    {
        if (fabs(lidar_time.toSec() - (it->second)->timestamp_.toSec()) > 0.15)
        {
            delete it->second;
            g_map.erase(it);
        }
    }
    for (int i = 0; i < eryuan_number; i++)
    {
        for (map<uint32_t, BaseObject*>::iterator it = eryuan_object_list[i].begin();
        it != eryuan_object_list[i].end(); it++)
        {
            if (fabs(eryuan_time.toSec() - (it->second)->timestamp_.toSec()) > 0.15)
            {
                delete it->second;
                eryuan_object_list[i].erase(it);
            }
        }
    }

    publish_map = g_map;
    for (int i = 0; i < eryuan_number; i++)
    {
        for (map<uint32_t, BaseObject*>::const_iterator it = eryuan_object_list[i].begin();
        it != eryuan_object_list[i].end(); it++)
        {
            publish_map[it->first] = it->second;
        }
    }
    for (map<uint32_t, BaseObject*>::const_iterator it1 = publish_map.begin();
    it1 != publish_map.end(); it1++)
    {
        for (map<uint32_t, BaseObject*>::const_iterator it2 = publish_map.begin();
        it2 != publish_map.end(); it2++)
        {
            if (it1 == it2)
            {
                continue;
            }
            float x1 = it1->second->point4_[0];
            float y1 = it1->second->point4_[1];
            float x2 = it1->second->point4_[2];
            float y2 = it1->second->point4_[3];
            float x3 = it1->second->point4_[4];
            float y3 = it1->second->point4_[5];
            float x4 = it1->second->point4_[6];
            float y4 = it1->second->point4_[7];
            float A1 = y2 - y1, B1 = x1 - x2, C1 = x2*y1-x1*y2, C2 = x4*(y1-y2)+y4*(x2-x1);
            float A2 = y4 - y1, B2 = x1 - x4, D1 = x4*y1-x1*y4, D2 = x2*(y1-y4)+y2*(x4-x1);
            bool in_rectangle = true;
            for (int i = 0; i < 4; i++)
            {
                float x = it2->second->point4_[2*i];
                float y = it2->second->point4_[2*i+1];
                // in_rectangle = in_rectangle & ( ((y2-y1)*(x-x1)-(x2-x1)*(y-y1))*((y2-y1)*(x-x4)-(x2-x1)*(y-y4)) < 0 );
                // in_rectangle = in_rectangle & ( ((y4-y1)*(x-x1)-(x4-x1)*(y-y1))*((y4-y1)*(x-x2)-(x4-x1)*(y-y2)) < 0 );
                in_rectangle = ( (fabs(A1*x+B1*y+C1) <= fabs(C1-C2) ) && (fabs(A1*x+B1*y+C2) <= fabs(C1-C2)) );
                in_rectangle = in_rectangle & ( ( fabs(A2*x+B2*y+D1) <= fabs(D1-D2) ) && ( fabs(A2*x+B2*y+D2) <= fabs(D1-D2) ) );
            }
            if (in_rectangle)
            {
                publish_map.erase(it2);
            }
        }
    }


    // for (map<uint32_t, BaseObject*>::const_iterator it1 = g_map.begin();
    // it1 != g_map.end(); it1++)
    // {
    //     bool jump = false;
    //     for (map<uint32_t, BaseObject*>::const_iterator it2 = g_map.begin();
    //     it2 != g_map.end(); it2++)
    //     {
    //         if (it1->first == it2->first)
    //         {
    //             continue;
    //         }
    //         // float x1 = it2->second->point4_[0];
    //         // float y1 = it2->second->point4_[1];
    //         // float x2 = it2->second->point4_[2];
    //         // float y2 = it2->second->point4_[3];
    //         // float x3 = it2->second->point4_[4];
    //         // float y3 = it2->second->point4_[5];
    //         // float x4 = it2->second->point4_[6];
    //         // float y4 = it2->second->point4_[7];
    //         bool in_rectangle = true;
    //         for (int i = 0; i < 4; i++)
    //         {
    //            in_rectangle = in_rectangle & ( ((it2->second->point4_[3]-it2->second->point4_[1])*(it1->second->point4_[2*i]-it2->second->point4_[0])-(it2->second->point4_[2]-it2->second->point4_[0])*(it1->second->point4_[2*i+1]-it2->second->point4_[1])) * ((it2->second->point4_[7]-it2->second->point4_[5])*(it1->second->point4_[2*i]-it2->second->point4_[4])-(it2->second->point4_[6]-it2->second->point4_[4])*(it1->second->point4_[2*i+1]-it2->second->point4_[5])) < 0 );
    //            in_rectangle = in_rectangle & ( ((it2->second->point4_[7]-it2->second->point4_[1])*(it1->second->point4_[2*i]-it2->second->point4_[0])-(it2->second->point4_[6]-it2->second->point4_[0])*(it1->second->point4_[2*i+1]-it2->second->point4_[1])) * ((it2->second->point4_[5]-it2->second->point4_[3])*(it1->second->point4_[2*i]-it2->second->point4_[2])-(it2->second->point4_[4]-it2->second->point4_[2])*(it1->second->point4_[2*i+1]-it2->second->point4_[3])) < 0 );
    //         }
    //         if (in_rectangle)
    //         {
    //             g_map.erase(it1);
    //             jump = true;
    //             break;
    //         }
    //     }
    //     if (jump)
    //     {
    //       continue;
    //     }
    //     for (int j = 0; j < eryuan_number; j++)
    //     {
    //         for (map<uint32_t, BaseObject*>::const_iterator it2 = eryuan_object_list[j].begin();
    //         it2 != eryuan_object_list[j].end(); it2++)
    //         {
    //             bool in_rectangle = true;
    //             for (int i = 0; i < 4; i++)
    //             {
    //                 in_rectangle = in_rectangle & ( ((it2->second->point4_[3]-it2->second->point4_[1])*(it1->second->point4_[2*i]-it2->second->point4_[0])-(it2->second->point4_[2]-it2->second->point4_[0])*(it1->second->point4_[2*i+1]-it2->second->point4_[1])) * ((it2->second->point4_[7]-it2->second->point4_[5])*(it1->second->point4_[2*i]-it2->second->point4_[4])-(it2->second->point4_[6]-it2->second->point4_[4])*(it1->second->point4_[2*i+1]-it2->second->point4_[5])) < 0 );
    //                 in_rectangle = in_rectangle & ( ((it2->second->point4_[7]-it2->second->point4_[1])*(it1->second->point4_[2*i]-it2->second->point4_[0])-(it2->second->point4_[6]-it2->second->point4_[0])*(it1->second->point4_[2*i+1]-it2->second->point4_[1])) * ((it2->second->point4_[5]-it2->second->point4_[3])*(it1->second->point4_[2*i]-it2->second->point4_[2])-(it2->second->point4_[4]-it2->second->point4_[2])*(it1->second->point4_[2*i+1]-it2->second->point4_[3])) < 0 );
    //             }
    //             if (in_rectangle)
    //             {
    //                 g_map.erase(it1);
    //                 jump = true;
    //                 break;
    //             }
    //         }
    //         if (jump)
    //         {
    //             break;
    //         }
    //     }
    // }
    // for (int k = 0; k < eryuan_number; k++)
    // {
    //     for (map<uint32_t, BaseObject*>::const_iterator it1 = eryuan_object_list[k].begin();
    //         it1 != eryuan_object_list[k].end(); it1++)
    //     {
    //         bool jump = false;
    //         for (map<uint32_t, BaseObject*>::const_iterator it2 = g_map.begin();
    //         it2 != g_map.end(); it2++)
    //         {
    //             bool in_rectangle = true;
    //             for (int i = 0; i < 4; i++)
    //             {
    //             in_rectangle = in_rectangle & ( ((it2->second->point4_[3]-it2->second->point4_[1])*(it1->second->point4_[2*i]-it2->second->point4_[0])-(it2->second->point4_[2]-it2->second->point4_[0])*(it1->second->point4_[2*i+1]-it2->second->point4_[1])) * ((it2->second->point4_[7]-it2->second->point4_[5])*(it1->second->point4_[2*i]-it2->second->point4_[4])-(it2->second->point4_[6]-it2->second->point4_[4])*(it1->second->point4_[2*i+1]-it2->second->point4_[5])) < 0 );
    //             in_rectangle = in_rectangle & ( ((it2->second->point4_[7]-it2->second->point4_[1])*(it1->second->point4_[2*i]-it2->second->point4_[0])-(it2->second->point4_[6]-it2->second->point4_[0])*(it1->second->point4_[2*i+1]-it2->second->point4_[1])) * ((it2->second->point4_[5]-it2->second->point4_[3])*(it1->second->point4_[2*i]-it2->second->point4_[2])-(it2->second->point4_[4]-it2->second->point4_[2])*(it1->second->point4_[2*i+1]-it2->second->point4_[3])) < 0 );
    //             }
    //             if (in_rectangle)
    //             {
    //                 g_map.erase(it1);
    //                 jump = true;
    //                 break;
    //             }
    //         }
    //         if (jump)
    //         {
    //             continue;
    //         }
    //         for (int j = 0; j < eryuan_number; j++)
    //         {
    //             for (map<uint32_t, BaseObject*>::const_iterator it2 = eryuan_object_list[j].begin();
    //             it2 != eryuan_object_list[j].end(); it2++)
    //             {
    //                 if (it1->first == it2->first)
    //                 {
    //                     continue;
    //                 }
    //                 bool in_rectangle = true;
    //                 for (int i = 0; i < 4; i++)
    //                 {
    //                     in_rectangle = in_rectangle & ( ((it2->second->point4_[3]-it2->second->point4_[1])*(it1->second->point4_[2*i]-it2->second->point4_[0])-(it2->second->point4_[2]-it2->second->point4_[0])*(it1->second->point4_[2*i+1]-it2->second->point4_[1])) * ((it2->second->point4_[7]-it2->second->point4_[5])*(it1->second->point4_[2*i]-it2->second->point4_[4])-(it2->second->point4_[6]-it2->second->point4_[4])*(it1->second->point4_[2*i+1]-it2->second->point4_[5])) < 0 );
    //                     in_rectangle = in_rectangle & ( ((it2->second->point4_[7]-it2->second->point4_[1])*(it1->second->point4_[2*i]-it2->second->point4_[0])-(it2->second->point4_[6]-it2->second->point4_[0])*(it1->second->point4_[2*i+1]-it2->second->point4_[1])) * ((it2->second->point4_[5]-it2->second->point4_[3])*(it1->second->point4_[2*i]-it2->second->point4_[2])-(it2->second->point4_[4]-it2->second->point4_[2])*(it1->second->point4_[2*i+1]-it2->second->point4_[3])) < 0 );
    //                 }
    //                 if (in_rectangle)
    //                 {
    //                     g_map.erase(it1);
    //                     jump = true;
    //                     break;
    //                 }
    //             }
    //             if (jump)
    //             {
    //                 break;
    //             }
    //         }
    //     }
    // }
}

void outputTypeTransformCamera(const map<uint32_t, BaseObject*> &publish_map, perception_msgs::FusionDataInfo &pub_obj_list, const ros::Time pub_time, const float veh_vxvy[2], const float att[3])
{
    #ifdef DEBUG_TOOLS
        cout << "outputTypeTransform: start" << endl;
    #endif
    pub_obj_list.header.stamp = pub_time;
    pub_obj_list.obstacle_num = publish_map.size();
    for (map<uint32_t, BaseObject*>::const_iterator it = publish_map.begin();
    it != publish_map.end(); it++)
    {
        common_msgs::ObstacleInfo obstacle;
        obstacle.id = it->first;
        float gx = it->second->veh_llh_(0);
        float gy = it->second->veh_llh_(1);
        float yaw = att[0];
        yaw = -yaw * M_PI / 180;
        Eigen::VectorXf state = it->second->getState();
        float x1 = it->second->point4_[0];
        float y1 = it->second->point4_[1];
        float x2 = it->second->point4_[2];
        float y2 = it->second->point4_[3];
        float x3 = it->second->point4_[4];
        float y3 = it->second->point4_[5];
        float x4 = it->second->point4_[6];
        float y4 = it->second->point4_[7];

        obstacle.local_peak[0].x = x1;
        obstacle.local_peak[0].y = y1;
        obstacle.local_peak[1].x = x2;
        obstacle.local_peak[1].y = y2;
        obstacle.local_peak[2].x = x3;
        obstacle.local_peak[2].y = y3;
        obstacle.local_peak[3].x = x4;
        obstacle.local_peak[3].y = y4;

        float global_x1 = x1*cos(yaw)-y1*sin(yaw);
        float global_y1 = y1*cos(yaw)+x1*sin(yaw);
        float global_x2 = x2*cos(yaw)-y2*sin(yaw);
        float global_y2 = y2*cos(yaw)+x2*sin(yaw);
        float global_x3 = x3*cos(yaw)-y3*sin(yaw);
        float global_y3 = y3*cos(yaw)+x3*sin(yaw);
        float global_x4 = x4*cos(yaw)-y4*sin(yaw);
        float global_y4 = y4*cos(yaw)+x4*sin(yaw);
        global_x1 = global_x1 + gx;
        global_y1 = global_y1 + gy;
        global_x2 = global_x2 + gx;
        global_y2 = global_y2 + gy;
        global_x3 = global_x3 + gx;
        global_y3 = global_y3 + gy;
        global_x4 = global_x4 + gx;
        global_y4 = global_y4 + gy;
        obstacle.peak[0].x = global_x1;
        obstacle.peak[0].y = global_y1;
        obstacle.peak[1].x = global_x2;
        obstacle.peak[1].y = global_y2;
        obstacle.peak[2].x = global_x3;
        obstacle.peak[2].y = global_y3;
        obstacle.peak[3].x = global_x4;
        obstacle.peak[3].y = global_y4;
        // state(4) --> vx
        // state(5) --> vy
        float vx = state(4)*cos(yaw)-state(5)*sin(yaw);
        float vy = state(5)*cos(yaw)+state(4)*sin(yaw);
        vx = vx + veh_vxvy[0];
        vy = vy + veh_vxvy[1];
        float v = sqrt(pow(vx, 2) + pow(vy, 2));
        obstacle.velocity = v > 0.2 ? v : 0.0;

        if(vy == 0)
        {
        obstacle.theta = 0;
        }
        else
        {
        obstacle.theta = atan(vx / vy) * 180.0 / 3.14159;
        }
        pub_obj_list.obstacles.push_back(obstacle);
    }
}

// void outputTypeTransformCamera(const map<uint32_t, BaseObject*> &g_map, perception_msgs::FusionDataInfo &pub_obj_list, ros::Time pub_time, float veh_vxvy[2], float att[3], map<uint32_t, BaseObject*> eryuan_object_list[], int eryuan_number)
// {
//   #ifdef DEBUG_TOOLS
//     cout << "outputTypeTransform: start" << endl;
//   #endif
//   pub_obj_list.header.stamp = pub_time;
//   pub_obj_list.obstacle_num = g_map.size();
//   for (map<uint32_t, BaseObject*>::const_iterator it = g_map.begin();
//       it != g_map.end(); it++)
//   {
//     common_msgs::ObstacleInfo obstacle;
//     obstacle.id = it->first;
//     float gx = it->second->veh_llh_(0);
//     float gy = it->second->veh_llh_(1);
//     float yaw = att[0];
//     yaw = -yaw * M_PI / 180;
//     Eigen::VectorXf state = it->second->getState();
//     float x1 = it->second->point4_[0];
//     float y1 = it->second->point4_[1];
//     float x2 = it->second->point4_[2];
//     float y2 = it->second->point4_[3];
//     float x3 = it->second->point4_[4];
//     float y3 = it->second->point4_[5];
//     float x4 = it->second->point4_[6];
//     float y4 = it->second->point4_[7];

//     float global_x1 = x1*cos(yaw)-y1*sin(yaw);
//     float global_y1 = y1*cos(yaw)+x1*sin(yaw);
//     float global_x2 = x2*cos(yaw)-y2*sin(yaw);
//     float global_y2 = y2*cos(yaw)+x2*sin(yaw);
//     float global_x3 = x3*cos(yaw)-y3*sin(yaw);
//     float global_y3 = y3*cos(yaw)+x3*sin(yaw);
//     float global_x4 = x4*cos(yaw)-y4*sin(yaw);
//     float global_y4 = y4*cos(yaw)+x4*sin(yaw);
//     global_x1 = global_x1 + gx;
//     global_y1 = global_y1 + gy;
//     global_x2 = global_x2 + gx;
//     global_y2 = global_y2 + gy;
//     global_x3 = global_x3 + gx;
//     global_y3 = global_y3 + gy;
//     global_x4 = global_x4 + gx;
//     global_y4 = global_y4 + gy;
//     obstacle.peak[0].x = global_x1;
//     obstacle.peak[0].y = global_y1;
//     obstacle.peak[1].x = global_x2;
//     obstacle.peak[1].y = global_y2;
//     obstacle.peak[2].x = global_x3;
//     obstacle.peak[2].y = global_y3;
//     obstacle.peak[3].x = global_x4;
//     obstacle.peak[3].y = global_y4;
//     // state(4) --> vx
//     // state(5) --> vy
//     float vx = state(4)*cos(yaw)-state(5)*sin(yaw);
//     float vy = state(5)*cos(yaw)+state(4)*sin(yaw);
//     vx = vx + veh_vxvy[0];
//     vy = vy + veh_vxvy[1];
//     float v = sqrt(pow(vx, 2) + pow(vy, 2));
//     obstacle.velocity = v > 0.2 ? v : 0.0;

//     if(vy == 0)
//     {
//       obstacle.theta = 0;
//     }
//     else
//     {
//       obstacle.theta = atan(vx / vy) * 180.0 / 3.14159;
//     }
//     pub_obj_list.obstacles.push_back(obstacle);
//   }

//   // ------------------------ camera ------------------------
//   for (int i = 0; i < eryuan_number; i++)
//   {
//     pub_obj_list.obstacle_num += eryuan_object_list[i].size();
//     for(map<uint32_t, BaseObject*>::const_iterator it = eryuan_object_list[i].begin();
//     it != eryuan_object_list[i].end(); it++)
//     {
//       common_msgs::ObstacleInfo obstacle;
//       obstacle.id = it->first;
//       float gx = it->second->veh_llh_(0);
//       float gy = it->second->veh_llh_(1);
//       float yaw = att[0];
//       yaw = -yaw * M_PI / 180;
//       Eigen::VectorXf state = it->second->getState();
//       float x1 = it->second->point4_[0];
//       float y1 = it->second->point4_[1];
//       float x2 = it->second->point4_[2];
//       float y2 = it->second->point4_[3];
//       float x3 = it->second->point4_[4];
//       float y3 = it->second->point4_[5];
//       float x4 = it->second->point4_[6];
//       float y4 = it->second->point4_[7];

//       float global_x1 = x1*cos(yaw)-y1*sin(yaw);
//       float global_y1 = y1*cos(yaw)+x1*sin(yaw);
//       float global_x2 = x2*cos(yaw)-y2*sin(yaw);
//       float global_y2 = y2*cos(yaw)+x2*sin(yaw);
//       float global_x3 = x3*cos(yaw)-y3*sin(yaw);
//       float global_y3 = y3*cos(yaw)+x3*sin(yaw);
//       float global_x4 = x4*cos(yaw)-y4*sin(yaw);
//       float global_y4 = y4*cos(yaw)+x4*sin(yaw);
//       global_x1 = global_x1 + gx;
//       global_y1 = global_y1 + gy;
//       global_x2 = global_x2 + gx;
//       global_y2 = global_y2 + gy;
//       global_x3 = global_x3 + gx;
//       global_y3 = global_y3 + gy;
//       global_x4 = global_x4 + gx;
//       global_y4 = global_y4 + gy;

//       obstacle.peak[0].x = global_x1;
//       obstacle.peak[0].y = global_y1;
//       obstacle.peak[1].x = global_x2;
//       obstacle.peak[1].y = global_y2;
//       obstacle.peak[2].x = global_x3;
//       obstacle.peak[2].y = global_y3;
//       obstacle.peak[3].x = global_x4;
//       obstacle.peak[3].y = global_y4;
//       // state(4) --> vx
//       // state(5) --> vy
//       float vx = state(4)*cos(yaw)-state(5)*sin(yaw);
//       float vy = state(5)*cos(yaw)+state(4)*sin(yaw);
//       vx = vx + veh_vxvy[0];
//       vy = vy + veh_vxvy[1];
//       float v = sqrt(pow(vx, 2) + pow(vy, 2));
//       obstacle.velocity = v > 0.2 ? v : 0.0;
//       if(vy == 0)
//       {
//         obstacle.theta = 0;
//       }
//       else
//       {
//         obstacle.theta = atan(vx / vy) * 180.0 / 3.14159;
//       }
//       pub_obj_list.obstacles.push_back(obstacle);
//     }
//   }
// }

// void outputTypeTransformCamera(const map<uint32_t, BaseObject*> &g_map, perception_msgs::FusionDataInfo &pub_obj_list, ros::Time pub_time, float veh_vxvy[2], float att[3], vector<BaseObject*> base_object_list)
// {
// #ifdef DEBUG_TOOLS
//     cout << "outputTypeTransform: start" << endl;
// #endif

//     map<uint32_t, BaseObject*>::const_iterator it = g_map.begin();

//     pub_obj_list.header.stamp = pub_time;
//     pub_obj_list.obstacle_num = g_map.size();

//     for(it; it != g_map.end(); it++)
//     {
//         common_msgs::ObstacleInfo obstacle;
//         obstacle.id = it->first;
//         float gx = it->second->veh_llh_(0);
//         float gy = it->second->veh_llh_(1);
//         float yaw = att[0];
//         // std::cout << "*****id = " << obstacle.id << std::endl;
//         // std::cout << "*****yaw(degree) = " << yaw << std::endl;
//         yaw = -yaw * M_PI / 180;
//         // std::cout << "*****gx = " << gx << std::endl;
//         // std::cout << "*****gy = " << gy << std::endl;
//         // std::cout << "*****yaw = " << yaw << std::endl;
//         Eigen::VectorXf state = it->second->getState();
//         // state(0) --> xmin
//         // state(1) --> ymin
//         // state(2) --> xmax
//         // state(3) --> ymax
//         // float x1 = state(0);
//         // float y1 = state(3);
//         // float x2 = state(2);
//         // float y2 = state(3);
//         // float x3 = state(2);
//         // float y3 = state(1);
//         // float x4 = state(0);
//         // float y4 = state(1);

//         float x1 = it->second->point4_[0];
//         float y1 = it->second->point4_[1];
//         float x2 = it->second->point4_[2];
//         float y2 = it->second->point4_[3];
//         float x3 = it->second->point4_[4];
//         float y3 = it->second->point4_[5];
//         float x4 = it->second->point4_[6];
//         float y4 = it->second->point4_[7];

//         float global_x1 = x1*cos(yaw)-y1*sin(yaw);
//         float global_y1 = y1*cos(yaw)+x1*sin(yaw);
//         float global_x2 = x2*cos(yaw)-y2*sin(yaw);
//         float global_y2 = y2*cos(yaw)+x2*sin(yaw);
//         float global_x3 = x3*cos(yaw)-y3*sin(yaw);
//         float global_y3 = y3*cos(yaw)+x3*sin(yaw);
//         float global_x4 = x4*cos(yaw)-y4*sin(yaw);
//         float global_y4 = y4*cos(yaw)+x4*sin(yaw);
//         global_x1 = global_x1 + gx;
//         global_y1 = global_y1 + gy;
//         global_x2 = global_x2 + gx;
//         global_y2 = global_y2 + gy;
//         global_x3 = global_x3 + gx;
//         global_y3 = global_y3 + gy;
//         global_x4 = global_x4 + gx;
//         global_y4 = global_y4 + gy;

//         // std::cout << "*****global_x1 = " << global_x1 << std::endl;
//         // std::cout << "*****global_y1 = " << global_y1 << std::endl;
//         // std::cout << "*****global_x2 = " << global_x2 << std::endl;
//         // std::cout << "*****global_y2 = " << global_y2 << std::endl;
//         // std::cout << "*****global_x3 = " << global_x3 << std::endl;
//         // std::cout << "*****global_y3 = " << global_y3 << std::endl;
//         // std::cout << "*****global_x4 = " << global_x4 << std::endl;
//         // std::cout << "*****global_y4 = " << global_y4 << std::endl;
//         obstacle.peak[0].x = global_x1;
//         obstacle.peak[0].y = global_y1;
//         obstacle.peak[1].x = global_x2;
//         obstacle.peak[1].y = global_y2;
//         obstacle.peak[2].x = global_x3;
//         obstacle.peak[2].y = global_y3;
//         obstacle.peak[3].x = global_x4;
//         obstacle.peak[3].y = global_y4;
        
//         // state(4) --> vx
//         // state(5) --> vy
//         float vx = state(4)*cos(yaw)-state(5)*sin(yaw);
//         float vy = state(5)*cos(yaw)+state(4)*sin(yaw);
//         vx = vx + veh_vxvy[0];
//         vy = vy + veh_vxvy[1];
//         // std::cout << "*****global_vx = " << vx << std::endl;
//         // std::cout << "*****global_vy = " << vy << std::endl;
//         float v = sqrt(pow(vx, 2) + pow(vy, 2));
//         obstacle.velocity = v > 0.2 ? v : 0.0;

//         if(vy == 0)
//         {
//             obstacle.theta = 0;
//         }
//         else
//         {
//             obstacle.theta = atan(vx / vy) * 180.0 / 3.14159;
//         }

//         pub_obj_list.obstacles.push_back(obstacle);
//     }

//     // ------------------------ camera ------------------------
//     for(int i = 0; i < base_object_list.size(); i++)
//     {
//         common_msgs::ObstacleInfo obstacle;
//         obstacle.id = base_object_list[i]->id;
//         float gx = base_object_list[i]->veh_llh_(0);
//         float gy = base_object_list[i]->veh_llh_(1);
//         float yaw = att[0];
//         // std::cout << "*****id = " << obstacle.id << std::endl;
//         // std::cout << "*****yaw(degree) = " << yaw << std::endl;
//         yaw = -yaw * M_PI / 180;
//         // std::cout << "*****gx = " << gx << std::endl;
//         // std::cout << "*****gy = " << gy << std::endl;
//         // std::cout << "*****yaw = " << yaw << std::endl;
//         Eigen::VectorXf state = base_object_list[i]->getState();
//         // state(0) --> xmin
//         // state(1) --> ymin
//         // state(2) --> xmax
//         // state(3) --> ymax
//         // float x1 = state(0);
//         // float y1 = state(3);
//         // float x2 = state(2);
//         // float y2 = state(3);
//         // float x3 = state(2);
//         // float y3 = state(1);
//         // float x4 = state(0);
//         // float y4 = state(1);

//         float x1 = base_object_list[i]->point4_[0];
//         float y1 = base_object_list[i]->point4_[1];
//         float x2 = base_object_list[i]->point4_[2];
//         float y2 = base_object_list[i]->point4_[3];
//         float x3 = base_object_list[i]->point4_[4];
//         float y3 = base_object_list[i]->point4_[5];
//         float x4 = base_object_list[i]->point4_[6];
//         float y4 = base_object_list[i]->point4_[7];

//         float global_x1 = x1*cos(yaw)-y1*sin(yaw);
//         float global_y1 = y1*cos(yaw)+x1*sin(yaw);
//         float global_x2 = x2*cos(yaw)-y2*sin(yaw);
//         float global_y2 = y2*cos(yaw)+x2*sin(yaw);
//         float global_x3 = x3*cos(yaw)-y3*sin(yaw);
//         float global_y3 = y3*cos(yaw)+x3*sin(yaw);
//         float global_x4 = x4*cos(yaw)-y4*sin(yaw);
//         float global_y4 = y4*cos(yaw)+x4*sin(yaw);
//         global_x1 = global_x1 + gx;
//         global_y1 = global_y1 + gy;
//         global_x2 = global_x2 + gx;
//         global_y2 = global_y2 + gy;
//         global_x3 = global_x3 + gx;
//         global_y3 = global_y3 + gy;
//         global_x4 = global_x4 + gx;
//         global_y4 = global_y4 + gy;

//         // std::cout << "*****global_x1 = " << global_x1 << std::endl;
//         // std::cout << "*****global_y1 = " << global_y1 << std::endl;
//         // std::cout << "*****global_x2 = " << global_x2 << std::endl;
//         // std::cout << "*****global_y2 = " << global_y2 << std::endl;
//         // std::cout << "*****global_x3 = " << global_x3 << std::endl;
//         // std::cout << "*****global_y3 = " << global_y3 << std::endl;
//         // std::cout << "*****global_x4 = " << global_x4 << std::endl;
//         // std::cout << "*****global_y4 = " << global_y4 << std::endl;
//         obstacle.peak[0].x = global_x1;
//         obstacle.peak[0].y = global_y1;
//         obstacle.peak[1].x = global_x2;
//         obstacle.peak[1].y = global_y2;
//         obstacle.peak[2].x = global_x3;
//         obstacle.peak[2].y = global_y3;
//         obstacle.peak[3].x = global_x4;
//         obstacle.peak[3].y = global_y4;
        
//         // state(4) --> vx
//         // state(5) --> vy
//         float vx = state(4)*cos(yaw)-state(5)*sin(yaw);
//         float vy = state(5)*cos(yaw)+state(4)*sin(yaw);
//         vx = vx + veh_vxvy[0];
//         vy = vy + veh_vxvy[1];
//         // std::cout << "*****global_vx = " << vx << std::endl;
//         // std::cout << "*****global_vy = " << vy << std::endl;
//         float v = sqrt(pow(vx, 2) + pow(vy, 2));
//         obstacle.velocity = v > 0.2 ? v : 0.0;

//         if(vy == 0)
//         {
//             obstacle.theta = 0;
//         }
//         else
//         {
//             obstacle.theta = atan(vx / vy) * 180.0 / 3.14159;
//         }

//         pub_obj_list.obstacles.push_back(obstacle);
//     }

//     pcl::PointCloud<pcl::PointXYZRGB> show_point;
//     sensor_msgs::PointCloud2 msg_point;
//     showResultInRviz2(pub_obj_list ,show_point, msg_point);
//     pub.publish(msg_point);


// #ifdef DEBUG_TOOLS
//     cout << "outputTypeTransform: end" << endl;
// #endif
// }

void showResultInRvizWithCamera(const map<uint32_t, BaseObject*> &publish_map, pcl::PointCloud<pcl::PointXYZRGB>& show_point, sensor_msgs::PointCloud2& msg_point, const float att[3], const bool global_rviz)
{
    for(map<uint32_t, BaseObject*>::const_iterator it = publish_map.begin(); it != publish_map.end(); it++)
    {
        float gx = it->second->veh_llh_(0);
        float gy = it->second->veh_llh_(1);
        float yaw = att[0];
        yaw = -yaw * M_PI / 180;
        Eigen::VectorXf state = it->second->getState();
        float x1 = it->second->point4_[0];
        float y1 = it->second->point4_[1];
        float x2 = it->second->point4_[2];
        float y2 = it->second->point4_[3];
        float x3 = it->second->point4_[4];
        float y3 = it->second->point4_[5];
        float x4 = it->second->point4_[6];
        float y4 = it->second->point4_[7];

        float global_x1, global_y1, global_x2, global_y2, global_x3, global_y3, global_x4, global_y4;
        if (global_rviz)
        {
            global_x1 = x1*cos(yaw)-y1*sin(yaw);
            global_y1 = y1*cos(yaw)+x1*sin(yaw);
            global_x2 = x2*cos(yaw)-y2*sin(yaw);
            global_y2 = y2*cos(yaw)+x2*sin(yaw);
            global_x3 = x3*cos(yaw)-y3*sin(yaw);
            global_y3 = y3*cos(yaw)+x3*sin(yaw);
            global_x4 = x4*cos(yaw)-y4*sin(yaw);
            global_y4 = y4*cos(yaw)+x4*sin(yaw);
            global_x1 = global_x1 + gx;
            global_y1 = global_y1 + gy;
            global_x2 = global_x2 + gx;
            global_y2 = global_y2 + gy;
            global_x3 = global_x3 + gx;
            global_y3 = global_y3 + gy;
            global_x4 = global_x4 + gx;
            global_y4 = global_y4 + gy;
        }
        else
        {
            global_x1 = x1;
            global_y1 = y1;
            global_x2 = x2;
            global_y2 = y2;
            global_x3 = x3;
            global_y3 = y3;
            global_x4 = x4;
            global_y4 = y4;
        }
        if (it->first < 10000000)
        {
            drawLine(global_x1, global_y1, 0, global_x2, global_y2, 0, show_point);
            drawLine(global_x2, global_y2, 0, global_x3, global_y3, 0, show_point);
            drawLine(global_x3, global_y3, 0, global_x4, global_y4, 0, show_point);
            drawLine(global_x4, global_y4, 0, global_x1, global_y1, 0, show_point);
        }
        else
        {
            drawLineCamere(global_x1, global_y1, 0, global_x2, global_y2, 0, show_point);
            drawLineCamere(global_x2, global_y2, 0, global_x3, global_y3, 0, show_point);
            drawLineCamere(global_x3, global_y3, 0, global_x4, global_y4, 0, show_point);
            drawLineCamere(global_x4, global_y4, 0, global_x1, global_y1, 0, show_point);
        }
    }
    pcl::toROSMsg(show_point, msg_point);
    msg_point.header.frame_id = "/odom";
    msg_point.header.stamp = ros::Time::now();
}

// void showResultInRvizWithCamera(const map<uint32_t, BaseObject*>& g_map, pcl::PointCloud<pcl::PointXYZRGB>& show_point, sensor_msgs::PointCloud2& msg_point, float att[3], map<uint32_t, BaseObject*> eryuan_object_list[], int eryuan_number, bool global_rviz)
// {
//     // bool gobal_rviz = true;
//     for(map<uint32_t, BaseObject*>::const_iterator it = g_map.begin(); it != g_map.end(); it++)
//     {
//         float gx = it->second->veh_llh_(0);
//         float gy = it->second->veh_llh_(1);
//         float yaw = att[0];
//         yaw = -yaw * M_PI / 180;
//         Eigen::VectorXf state = it->second->getState();
//         float x1 = it->second->point4_[0];
//         float y1 = it->second->point4_[1];
//         float x2 = it->second->point4_[2];
//         float y2 = it->second->point4_[3];
//         float x3 = it->second->point4_[4];
//         float y3 = it->second->point4_[5];
//         float x4 = it->second->point4_[6];
//         float y4 = it->second->point4_[7];

//         float global_x1, global_y1, global_x2, global_y2, global_x3, global_y3, global_x4, global_y4;
//         if (global_rviz)
//         {
//             global_x1 = x1*cos(yaw)-y1*sin(yaw);
//             global_y1 = y1*cos(yaw)+x1*sin(yaw);
//             global_x2 = x2*cos(yaw)-y2*sin(yaw);
//             global_y2 = y2*cos(yaw)+x2*sin(yaw);
//             global_x3 = x3*cos(yaw)-y3*sin(yaw);
//             global_y3 = y3*cos(yaw)+x3*sin(yaw);
//             global_x4 = x4*cos(yaw)-y4*sin(yaw);
//             global_y4 = y4*cos(yaw)+x4*sin(yaw);
//             global_x1 = global_x1 + gx;
//             global_y1 = global_y1 + gy;
//             global_x2 = global_x2 + gx;
//             global_y2 = global_y2 + gy;
//             global_x3 = global_x3 + gx;
//             global_y3 = global_y3 + gy;
//             global_x4 = global_x4 + gx;
//             global_y4 = global_y4 + gy;
//         }
//         else
//         {
//             global_x1 = x1;
//             global_y1 = y1;
//             global_x2 = x2;
//             global_y2 = y2;
//             global_x3 = x3;
//             global_y3 = y3;
//             global_x4 = x4;
//             global_y4 = y4;
//         }
//         drawLine(global_x1, global_y1, 0, global_x2, global_y2, 0, show_point);
//         drawLine(global_x2, global_y2, 0, global_x3, global_y3, 0, show_point);
//         drawLine(global_x3, global_y3, 0, global_x4, global_y4, 0, show_point);
//         drawLine(global_x4, global_y4, 0, global_x1, global_y1, 0, show_point);
//     }
//     for (int i = 0; i < eryuan_number; i++)
//     {
//         for (map<uint32_t, BaseObject*>::const_iterator it = eryuan_object_list[i].begin();
//         it != eryuan_object_list[i].end(); it++)
//         {
//             float gx = it->second->veh_llh_(0);
//             float gy = it->second->veh_llh_(1);
//             float yaw = att[0];
//             yaw = -yaw * M_PI / 180;
//             Eigen::VectorXf state = it->second->getState();
//             float x1 = it->second->point4_[0];
//             float y1 = it->second->point4_[1];
//             float x2 = it->second->point4_[2];
//             float y2 = it->second->point4_[3];
//             float x3 = it->second->point4_[4];
//             float y3 = it->second->point4_[5];
//             float x4 = it->second->point4_[6];
//             float y4 = it->second->point4_[7];

//             float global_x1, global_y1, global_x2, global_y2, global_x3, global_y3, global_x4, global_y4;
//             if (global_rviz)
//             {
//                 global_x1 = x1*cos(yaw)-y1*sin(yaw);
//                 global_y1 = y1*cos(yaw)+x1*sin(yaw);
//                 global_x2 = x2*cos(yaw)-y2*sin(yaw);
//                 global_y2 = y2*cos(yaw)+x2*sin(yaw);
//                 global_x3 = x3*cos(yaw)-y3*sin(yaw);
//                 global_y3 = y3*cos(yaw)+x3*sin(yaw);
//                 global_x4 = x4*cos(yaw)-y4*sin(yaw);
//                 global_y4 = y4*cos(yaw)+x4*sin(yaw);
//                 global_x1 = global_x1 + gx;
//                 global_y1 = global_y1 + gy;
//                 global_x2 = global_x2 + gx;
//                 global_y2 = global_y2 + gy;
//                 global_x3 = global_x3 + gx;
//                 global_y3 = global_y3 + gy;
//                 global_x4 = global_x4 + gx;
//                 global_y4 = global_y4 + gy;
//             }
//             else
//             {
//                 global_x1 = x1;
//                 global_y1 = y1;
//                 global_x2 = x2;
//                 global_y2 = y2;
//                 global_x3 = x3;
//                 global_y3 = y3;
//                 global_x4 = x4;
//                 global_y4 = y4;
//             }
//             drawLineCamere(global_x1, global_y1, 0, global_x2, global_y2, 0, show_point);
//             drawLineCamere(global_x2, global_y2, 0, global_x3, global_y3, 0, show_point);
//             drawLineCamere(global_x3, global_y3, 0, global_x4, global_y4, 0, show_point);
//             drawLineCamere(global_x4, global_y4, 0, global_x1, global_y1, 0, show_point);
//         }
//     }
//     pcl::toROSMsg(show_point, msg_point);
//     msg_point.header.frame_id = "/odom";
//     msg_point.header.stamp = ros::Time::now();
// }


// void showResultInRvizWithCamera(const map<uint32_t, BaseObject*>& g_map, pcl::PointCloud<pcl::PointXYZRGB>& show_point, sensor_msgs::PointCloud2& msg_point, float att[3], vector<BaseObject*> base_object_list)
// {
//     bool gobal_rviz = true;
//     map<uint32_t, BaseObject*>::const_iterator it;
//     for(it = g_map.begin(); it != g_map.end(); it++)
//     {
//         float gx = it->second->veh_llh_(0);
//         float gy = it->second->veh_llh_(1);
//         float yaw = att[0];
//         yaw = -yaw * M_PI / 180;
//         Eigen::VectorXf state = it->second->getState();
//         // float xmin = state(0);
//         // float ymin = state(1);
//         // float xmax = state(2);
//         // float ymax = state(3);

//         // float x1 = state(0);
//         // float y1 = state(3);
//         // float x2 = state(2);
//         // float y2 = state(3);
//         // float x3 = state(2);
//         // float y3 = state(1);
//         // float x4 = state(0);
//         // float y4 = state(1);

//         float x1 = it->second->point4_[0];
//         float y1 = it->second->point4_[1];
//         float x2 = it->second->point4_[2];
//         float y2 = it->second->point4_[3];
//         float x3 = it->second->point4_[4];
//         float y3 = it->second->point4_[5];
//         float x4 = it->second->point4_[6];
//         float y4 = it->second->point4_[7];

//         float global_x1, global_y1, global_x2, global_y2, global_x3, global_y3, global_x4, global_y4;
//         if (gobal_rviz)
//         {
//             global_x1 = x1*cos(yaw)-y1*sin(yaw);
//             global_y1 = y1*cos(yaw)+x1*sin(yaw);
//             global_x2 = x2*cos(yaw)-y2*sin(yaw);
//             global_y2 = y2*cos(yaw)+x2*sin(yaw);
//             global_x3 = x3*cos(yaw)-y3*sin(yaw);
//             global_y3 = y3*cos(yaw)+x3*sin(yaw);
//             global_x4 = x4*cos(yaw)-y4*sin(yaw);
//             global_y4 = y4*cos(yaw)+x4*sin(yaw);
//             global_x1 = global_x1 + gx;
//             global_y1 = global_y1 + gy;
//             global_x2 = global_x2 + gx;
//             global_y2 = global_y2 + gy;
//             global_x3 = global_x3 + gx;
//             global_y3 = global_y3 + gy;
//             global_x4 = global_x4 + gx;
//             global_y4 = global_y4 + gy;
//         }
//         else
//         {
//             global_x1 = x1;
//             global_y1 = y1;
//             global_x2 = x2;
//             global_y2 = y2;
//             global_x3 = x3;
//             global_y3 = y3;
//             global_x4 = x4;
//             global_y4 = y4;
//         }
        
//         drawLine(global_x1, global_y1, 0, global_x2, global_y2, 0, show_point);
//         drawLine(global_x2, global_y2, 0, global_x3, global_y3, 0, show_point);
//         drawLine(global_x3, global_y3, 0, global_x4, global_y4, 0, show_point);
//         drawLine(global_x4, global_y4, 0, global_x1, global_y1, 0, show_point);
//     }

//     for(int i = 0; i < base_object_list.size(); i++)
//     {
//         float gx = base_object_list[0]->veh_llh_(0);
//         float gy = base_object_list[0]->veh_llh_(1);
//         float yaw = att[0];
//         yaw = -yaw * M_PI / 180;
//         Eigen::VectorXf state = base_object_list[0]->getState();
//         // float xmin = state(0);
//         // float ymin = state(1);
//         // float xmax = state(2);
//         // float ymax = state(3);

//         // float x1 = state(0);
//         // float y1 = state(3);
//         // float x2 = state(2);
//         // float y2 = state(3);
//         // float x3 = state(2);
//         // float y3 = state(1);
//         // float x4 = state(0);
//         // float y4 = state(1);

//         float x1 = base_object_list[0]->point4_[0];
//         float y1 = base_object_list[0]->point4_[1];
//         float x2 = base_object_list[0]->point4_[2];
//         float y2 = base_object_list[0]->point4_[3];
//         float x3 = base_object_list[0]->point4_[4];
//         float y3 = base_object_list[0]->point4_[5];
//         float x4 = base_object_list[0]->point4_[6];
//         float y4 = base_object_list[0]->point4_[7];

//         float global_x1, global_y1, global_x2, global_y2, global_x3, global_y3, global_x4, global_y4;
//         if (gobal_rviz)
//         {
//             global_x1 = x1*cos(yaw)-y1*sin(yaw);
//             global_y1 = y1*cos(yaw)+x1*sin(yaw);
//             global_x2 = x2*cos(yaw)-y2*sin(yaw);
//             global_y2 = y2*cos(yaw)+x2*sin(yaw);
//             global_x3 = x3*cos(yaw)-y3*sin(yaw);
//             global_y3 = y3*cos(yaw)+x3*sin(yaw);
//             global_x4 = x4*cos(yaw)-y4*sin(yaw);
//             global_y4 = y4*cos(yaw)+x4*sin(yaw);
//             global_x1 = global_x1 + gx;
//             global_y1 = global_y1 + gy;
//             global_x2 = global_x2 + gx;
//             global_y2 = global_y2 + gy;
//             global_x3 = global_x3 + gx;
//             global_y3 = global_y3 + gy;
//             global_x4 = global_x4 + gx;
//             global_y4 = global_y4 + gy;
//         }
//         else
//         {
//             global_x1 = x1;
//             global_y1 = y1;
//             global_x2 = x2;
//             global_y2 = y2;
//             global_x3 = x3;
//             global_y3 = y3;
//             global_x4 = x4;
//             global_y4 = y4;
//         }
//         std::cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << std::endl;
//         drawLineCamere(global_x1, global_y1, 0, global_x2, global_y2, 0, show_point);
//         drawLineCamere(global_x2, global_y2, 0, global_x3, global_y3, 0, show_point);
//         drawLineCamere(global_x3, global_y3, 0, global_x4, global_y4, 0, show_point);
//         drawLineCamere(global_x4, global_y4, 0, global_x1, global_y1, 0, show_point);
//     }

//     pcl::toROSMsg(show_point, msg_point);
//     msg_point.header.frame_id = "/odom";
//     msg_point.header.stamp = ros::Time::now();
// }

// void filterEryuanObject(map<uint32_t, BaseObject*> eryuan_object_list[], int sensor_index)
// {
//     sensor_index--;
//     for (map<uint32_t, BaseObject*>::const_iterator it = eryuan_object_list[sensor_index]->begin();
//     it != eryuan_object_list[sensor_index]->end(); it++)
//     {

//     }
// }

void updateEryuanObject(map<uint32_t, BaseObject*> eryuan_object_list[], const perception_sensor_msgs::ObjectList msg_object_list, float att[], Vector3d &veh_llh,
float range_xmax, float range_xmin, float range_ymax, float range_ymin,
float agv_xmax, float agv_xmin, float agv_ymax, float agv_ymin)
{
    int index = msg_object_list.sensor_index - 1;
    for (map<uint32_t, BaseObject*>::const_iterator it = eryuan_object_list[index].begin();
    it != eryuan_object_list[index].end(); it++)
    {
        delete it->second;
    }
    eryuan_object_list[index].clear();
    for(int i = 0; i < msg_object_list.object_list.size(); i++)
    {
        // bool jump = false;
        // for (int j = 0; j < 4; j ++)
        // {
        //     if (msg_object_list.object_list[i].peek[j].x > range_xmax ||
        //     msg_object_list.object_list[i].peek[j].x < range_xmin ||
        //     msg_object_list.object_list[i].peek[j].y > range_ymax ||
        //     msg_object_list.object_list[i].peek[j].y < range_ymin)
        //     {
        //         jump = true;
        //         break;
        //     }
        // }
        if (msg_object_list.object_list[i].peek[0].x < range_xmin ||
        msg_object_list.object_list[i].peek[2].x > range_xmax ||
        msg_object_list.object_list[i].peek[0].y < range_ymin ||
        msg_object_list.object_list[i].peek[1].y > range_ymax)
        {
            continue;
        }
        float xmax = msg_object_list.object_list[i].peek[0].x > agv_xmax ? agv_xmax : msg_object_list.object_list[i].peek[0].x;
        float xmin = msg_object_list.object_list[i].peek[2].x > agv_xmin ? msg_object_list.object_list[i].peek[2].x : agv_xmin;
        float ymax = msg_object_list.object_list[i].peek[0].y > agv_ymax ? agv_ymax : msg_object_list.object_list[i].peek[0].y;
        float ymin = msg_object_list.object_list[i].peek[1].y > agv_ymin ? msg_object_list.object_list[i].peek[1].y : agv_ymin;
        // std::cout << "peek[0].x = " << msg_object_list.object_list[i].peek[0].x << std::endl;
        // std::cout << "peek[0].y = " << msg_object_list.object_list[i].peek[0].y << std::endl;
        // std::cout << "peek[1].x = " << msg_object_list.object_list[i].peek[1].x << std::endl;
        // std::cout << "peek[1].y = " << msg_object_list.object_list[i].peek[1].y << std::endl;
        // std::cout << "peek[2].x = " << msg_object_list.object_list[i].peek[2].x << std::endl;
        // std::cout << "peek[2].y = " << msg_object_list.object_list[i].peek[2].y << std::endl;
        // std::cout << "peek[3].x = " << msg_object_list.object_list[i].peek[3].x << std::endl;
        // std::cout << "peek[3].y = " << msg_object_list.object_list[i].peek[3].y << std::endl;
        // std::cout << "xmax = " << xmax << std::endl;
        // std::cout << "xmin = " << xmin << std::endl;
        // std::cout << "ymax = " << ymax << std::endl;
        // std::cout << "ymin = " << ymin << std::endl;
        if (xmax > xmin && ymax > ymin)
        {
            continue;
        }
        // 1. car  2. person  3. trunk  4. cone
        if (msg_object_list.object_list[i].obj_class != 2 &&
        msg_object_list.object_list[i].obj_class != 4)
        {
            continue;
        }
        BaseFilter* filter = new NormalKalmanFilter(msg_object_list.object_list[i].state, msg_object_list.object_list[i].measurement_cov);
        float point4[8];
        point4[0] = msg_object_list.object_list[i].peek[0].x;
        point4[1] = msg_object_list.object_list[i].peek[0].y;
        point4[2] = msg_object_list.object_list[i].peek[1].x;
        point4[3] = msg_object_list.object_list[i].peek[1].y;
        point4[4] = msg_object_list.object_list[i].peek[2].x;
        point4[5] = msg_object_list.object_list[i].peek[2].y;
        point4[6] = msg_object_list.object_list[i].peek[3].x;
        point4[7] = msg_object_list.object_list[i].peek[3].y;
        
        uint32_t id = msg_object_list.object_list[i].id + 10000000 * msg_object_list.sensor_index;
        eryuan_object_list[index][id] = new CameraObject(
                        id,
                        msg_object_list.header.stamp,
                        msg_object_list.object_list[i].obj_class,
                        msg_object_list.object_list[i].confidence, 1.0, 
                        att, veh_llh,
                        filter, point4);
    }
}

