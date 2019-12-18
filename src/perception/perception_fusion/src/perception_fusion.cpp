#include "perception_fusion.h"

PerceptionFusion::PerceptionFusion(BaseAssociation* base_association, float velocity_threshold, bool is_draw, bool global_rviz,
float range_xmax, float range_xmin, float range_ymax, float range_ymin,
float agv_xmax, float agv_xmin, float agv_ymax, float agv_ymin):
base_association_(base_association),
velocity_threshold_(velocity_threshold),
is_draw_(is_draw),
global_rviz_(global_rviz),
range_xmax_(range_xmax),
range_xmin_(range_xmin),
range_ymax_(range_ymax),
range_ymin_(range_ymin),
agv_xmax_(agv_xmax),
agv_xmin_(agv_xmin),
agv_ymax_(agv_ymax),
agv_ymin_(agv_ymin)
{
#ifdef DEBUG_PERCEPTION_FUSION
    cout << "PerceptionFusion ctor start" << endl;
#endif

    // std::cout << "range_xmax = " << range_xmax_ << std::endl;
    pub_obstacle_info_ = nh_.advertise<perception_msgs::FusionDataInfo>("/perception/obstacle_info", 1);
    pub_rviz_bounding_box_ = nh_.advertise<sensor_msgs::PointCloud2>("/perception_fusion/rviz/make_bounding_box", 1);
    pub_rviz_bounding_box_info_ = nh_.advertise<visualization_msgs::Marker>("/perception_fusion/rviz/make_bounding_box_info", 1);

    // sub_camera_.subscribe(nh_, "/drivers/perception/camera_obstacle_info", 100);
    // sub_location_.subscribe(nh_, "/localization/fusion_msg", 100);
    // sync_camera_location_.reset(new Sync(MySyncPolicy(5000), sub_camera_, sub_location_));
    // sync_camera_location_->registerCallback(boost::bind(&PerceptionFusion::callbackFusion, this, _1, _2));

    sub_lidar_no_sync_ = nh_.subscribe("/perception/detection_lidar", 1, &PerceptionFusion::callbackLidarFusion, this);

#ifdef DEBUG_PERCEPTION_FUSION
    cout << "PerceptionFusion ctor endl" << endl;
#endif
}

PerceptionFusion::~PerceptionFusion()
{
    delete base_association_;
}

// 传感器类型 0：相机, 1: 激光雷达, 2: 毫米板雷达
void PerceptionFusion::callbackLidarFusion(const perception_sensor_msgs::ObjectList::ConstPtr& sensor_msg)
{
    cout << "====================== callbackLidarFusion: start ======================" << endl;
    std::cout << "fusion sensor time = " << std::setprecision(15) << sensor_msg->header.stamp << std::endl;
    std::cout << "fusion start time = " << std::setprecision(15) << ros::Time::now().toSec() << std::endl;
    clock_t start, end;
    float elapsedTime;
    start = clock();

    perception_sensor_msgs::ObjectList msg_object_list = *sensor_msg;

    veh_vxvy_[0] = sensor_msg->velocity.linear.x;
    veh_vxvy_[1] = sensor_msg->velocity.linear.y;

    lidar_time = sensor_msg->header.stamp;
    
    // 关联矩阵
    Eigen::MatrixXd incidence_matrix;

    // 将目标信息转为BaseObject类型
    vector<BaseObject*> base_object_list;
    float att[3] = {sensor_msg->yaw, sensor_msg->pitch, sensor_msg->roll};
    Vector3d veh_llh(sensor_msg->pose.x, sensor_msg->pose.y, sensor_msg->pose.z);
    cout << "sensor_msg->sensor_type: " << to_string(sensor_msg->sensor_type) << endl;
    inputTypeTransform(msg_object_list, base_object_list, sensor_msg->sensor_type, att, veh_llh);   
            cout << "base_object_list.size: " << base_object_list.size() << endl;

    // if(sensor_msg->sensor_type == 1)
    // {
    //     // 处理第一帧数据
    //     if(global_object_.empty())
    //     {
    //         updateNewObject(base_object_list);
    //     }
    //     else
    //     {
    //         cout << "base_object_list.size: " << base_object_list.size() << endl;

    //         if(0 != base_object_list.size())
    //         {
    //             clock_t start2, end2;
    //             float elapsedTime2;
    //             start2 = clock();
    //             // 对疑似新目标和未匹配全局目标做数据关联
    //             base_association_->getIncidenceMatrix(global_object_, base_object_list, incidence_matrix);

    //             // 更新关联上的全局目标
    //             updateAssociatedObject(base_object_list, incidence_matrix);
    //             // 更新新目标
    //             updateUnassociatedObject(base_object_list, incidence_matrix);
    //             end2 = clock();
    //             elapsedTime2 = (float)(end2 - start2) / CLOCKS_PER_SEC;
    //             cout << "Completed2 in: " << elapsedTime2 << " s" << endl;

    //         }
    //     }
    //     publishFusionObject(sensor_msg->header.stamp, is_draw_, att, sensor_msg->sensor_type);
    // }
    // else if(sensor_msg->sensor_type == 0)
    // {
    //     publishFusionObjectWithCamera(sensor_msg->header.stamp, is_draw_, att, sensor_msg->sensor_type, base_object_list);
    // }

    // 处理第一帧数据
    if(global_object_.empty())
    {
        updateNewObject(base_object_list);
    }
    else
    {
        cout << "base_object_list.size: " << base_object_list.size() << endl;

        if(0 != base_object_list.size())
        {
            clock_t start2, end2;
            float elapsedTime2;
            start2 = clock();
            // 对疑似新目标和未匹配全局目标做数据关联
            base_association_->getIncidenceMatrix(global_object_, base_object_list, incidence_matrix);

            // 更新关联上的全局目标
            updateAssociatedObject(base_object_list, incidence_matrix);
            // 更新新目标
            updateUnassociatedObject(base_object_list, incidence_matrix);
            end2 = clock();
            elapsedTime2 = (float)(end2 - start2) / CLOCKS_PER_SEC;
            cout << "Completed2 in: " << elapsedTime2 << " s" << endl;
        }
    }
    map<uint32_t, BaseObject*> publish_map;
    removeObject(lidar_time, eryuan_time, global_object_, eryuan_object_list, eryuan_number, publish_map);
    publishFusionObjectWithCamera(sensor_msg->header.stamp, is_draw_, att, publish_map);

    end = clock();
    std::cout << "fusion end time = " << std::setprecision(15) << ros::Time::now().toSec() << std::endl;
    elapsedTime = (float)(end - start) / CLOCKS_PER_SEC;
    cout << "Completed in: " << elapsedTime << " s" << endl;

    cout << "====================== callbackLidarFusion: end ======================" << endl;
}

void PerceptionFusion::callbackFusion(const perception_sensor_msgs::ObjectList::ConstPtr& sensor_msg, const location_msgs::FusionDataInfo::ConstPtr& location_msg)
{
    eryuan_frequency_number++;
    if (eryuan_frequency_number > 30)
    {
        eryuan_frequency_number = 0;
    }
    if (eryuan_frequency_number == 0)
    {
        cout << "====================== callbackFusion: start ======================" << endl;
        std::cout << "fusion sensor time = " << std::setprecision(15) << sensor_msg->header.stamp << std::endl;
        std::cout << "fusion start time = " << std::setprecision(15) << ros::Time::now().toSec() << std::endl;
        // cout << "global_rviz = " << global_rviz_ << endl;
        clock_t start, end;
        float elapsedTime;
        start = clock();

        perception_sensor_msgs::ObjectList msg_object_list = *sensor_msg;

        veh_vxvy_[0] = location_msg->velocity.linear.x;
        veh_vxvy_[1] = location_msg->velocity.linear.y;

        eryuan_time = location_msg->header.stamp;
        
        // 关联矩阵
        Eigen::MatrixXd incidence_matrix;

        // 将目标信息转为BaseObject类型
        vector<BaseObject*> base_object_list;
        float att[3] = {location_msg->yaw, location_msg->pitch, location_msg->roll};
        Vector3d veh_llh(location_msg->pose.x, location_msg->pose.y, location_msg->pose.z);

        updateEryuanObject(eryuan_object_list, msg_object_list, att, veh_llh,
        range_xmax_, range_xmin_, range_ymax_, range_ymin_,
        agv_xmax_, agv_xmin_, agv_ymax_, agv_ymin_);

        cout << "sensor_msg->sensor_type: " << to_string(sensor_msg->sensor_type) << endl;
        // inputTypeTransform(msg_object_list, base_object_list, sensor_msg->sensor_type, att, veh_llh);   

        // if(sensor_msg->sensor_type == 1)
        // {
        //     // 处理第一帧数据
        //     if(global_object_.empty())
        //     {
        //         updateNewObject(base_object_list);
        //     }
        //     else
        //     {
        //         cout << "base_object_list.size: " << base_object_list.size() << endl;

        //         if(0 != base_object_list.size())
        //         {
        //             clock_t start2, end2;
        //             float elapsedTime2;
        //             start2 = clock();
        //             // 对疑似新目标和未匹配全局目标做数据关联
        //             base_association_->getIncidenceMatrix(global_object_, base_object_list, incidence_matrix);

        //             // 更新关联上的全局目标
        //             updateAssociatedObject(base_object_list, incidence_matrix);
        //             // 更新新目标
        //             updateUnassociatedObject(base_object_list, incidence_matrix);
        //             end2 = clock();
        //             elapsedTime2 = (float)(end2 - start2) / CLOCKS_PER_SEC;
        //             cout << "Completed2 in: " << elapsedTime2 << " s" << endl;

        //         }
        //     }
        //     publishFusionObject(sensor_msg->header.stamp, is_draw_, att, sensor_msg->sensor_type);
        // }
        // else if(sensor_msg->sensor_type == 0)
        // {
        //     publishFusionObjectWithCamera(sensor_msg->header.stamp, is_draw_, att, sensor_msg->sensor_type, base_object_list);
        // }

        map<uint32_t, BaseObject*> publish_map;
        removeObject(lidar_time, eryuan_time, global_object_, eryuan_object_list, eryuan_number, publish_map);
        publishFusionObjectWithCamera(location_msg->header.stamp, is_draw_, att, publish_map);

        end = clock();
        std::cout << "fusion end time = " << std::setprecision(15) << ros::Time::now().toSec() << std::endl;
        elapsedTime = (float)(end - start) / CLOCKS_PER_SEC;
        cout << "Completed in: " << elapsedTime << " s" << endl;

        cout << "====================== callbackFusion: end ======================" << endl;
    }
}

void PerceptionFusion::updateNewObject(vector<BaseObject*>& obj_list)
{
#ifdef DEBUG_PERCEPTION_FUSION
    cout << "updateNewObject start" << endl;
#endif

    for(int i = 0; i < obj_list.size(); i++)
    {
        global_object_[global_id_] = obj_list[i];
        if(global_id_ < 10000000)
            global_id_++;
        else
            global_id_ = 0;
        
    }

#ifdef DEBUG_PERCEPTION_FUSION
    cout << "updateNewObject end" << endl;
#endif
}

void PerceptionFusion::updateAssociatedObject(vector<BaseObject*>& new_obj, Eigen::MatrixXd& matrix)
{
#ifdef DEBUG_PERCEPTION_FUSION
    cout << "updateAssociatedObject start" << endl;
#endif

    // matrix [cols]:旧目标, [row]:新目标
    int cols = matrix.cols();
    int rows = matrix.rows();

    map<uint32_t, BaseObject*>::iterator it;
    for(int i = 0; i < rows; i++)
    {
        int j = 0;
        for(it = global_object_.begin(); it != global_object_.end(); it++)
        {
            if(matrix(i, j) == 1)
            {
                // cout << "****************** global_id: " << it->first << " start ****************"  << endl;
                new_obj[i]->update(*(it->second));
                delete new_obj[i];
                // cout << "****************** global_id: " << it->first << " end   ****************"  << endl;
            }
            j++;
        }
    }

#ifdef DEBUG_PERCEPTION_FUSION
    cout << "updateAssociatedObject end" << endl;
#endif
}

void PerceptionFusion::updateUnassociatedObject(vector<BaseObject*>& new_obj, Eigen::MatrixXd& matrix)
{
#ifdef DEBUG_PERCEPTION_FUSION
    cout << "updateUnassociatedObject start" << endl;
#endif

    // matrix [col]旧目标:, [row]:新目标
    int cols = matrix.cols();
    int rows = matrix.rows();

    vector<BaseObject*> obj_list;
    map<uint32_t, BaseObject*>::iterator it;

    for(int i = 0; i < rows; i++)
    {
        int j = 0;
        bool is_match = false;
        for(it = global_object_.begin(); it != global_object_.end(); it++)
        {
            if(matrix(i, j) == 1)
            {
                is_match = true;
            }
            j++;
        }

        if(!is_match)
        {
            obj_list.push_back(new_obj[i]);
        }
    }

    updateNewObject(obj_list);

#ifdef DEBUG_PERCEPTION_FUSION
    cout << "updateUnassociatedObject end" << endl;
#endif
}

// void PerceptionFusion::publishFusionObject(ros::Time pub_time, bool is_draw, float att[3], int sensor_type)
// {
//     cout << "********************** publish : start **********************" << endl;
    
//     cout << "total object before erase loss obj = " << global_object_.size() << endl;

//     // 声明决策需求的数据结构
//     perception_msgs::FusionDataInfo global_pub_object_list;
//     map<uint32_t, BaseObject*>::iterator it;
//     for(it = global_object_.begin(); it != global_object_.end();)
//     {
//         // std::cout << "#########################" << std::endl;
//         // std::cout << "time interval: " << fabs(pub_time.toSec() - (it->second)->timestamp_.toSec()) << std::endl;
//         // std::cout << "pub time: " << pub_time.toSec() << std::endl;
//         // std::cout << "object time: " << (it->second)->timestamp_.toSec() << std::endl;
//         if (fabs(pub_time.toSec() - (it->second)->timestamp_.toSec()) > 0.15)
//         {
//             delete it->second;
//             global_object_.erase(it++);
//         }
//         else
//         {
//             it->second->prediction(pub_time);
//             it++;
//         }
//     }
//     cout << "total object after erase loss obj = " << global_object_.size() << endl;

//     // 在rviz中显示检测结果
//     if(is_draw_)
//     {
//         visualization_msgs::Marker marker;
//         marker.header.frame_id = "/odom";
//         marker.header.stamp = ros::Time::now();
//         marker.ns = "basic_shapes";
//         marker.action = visualization_msgs::Marker::ADD;
//         marker.pose.orientation.w = 1.0;
//         marker.lifetime = ros::Duration(0.1);
//         marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

//         marker.scale.z = 2.0;
//         marker.color.b = 0;
//         marker.color.g = 1;
//         marker.color.r = 0;
//         marker.color.a = 1;
//         bool gobal_rviz = true;
//         for(it = global_object_.begin(); it != global_object_.end();)
//         {
//             marker.id = it->first;
//             geometry_msgs::Pose pose;
//             Eigen:: VectorXf state = it->second->getState();
//             float x = state(2);
//             float y = state(3);
//             float z = 2;
//             if (gobal_rviz)
//             {
//                 float gx = it->second->veh_llh_(0);
//                 float gy = it->second->veh_llh_(1);
//                 float yaw = att[0];
//                 yaw = -yaw * M_PI / 180;
//                 pose.position.x = gx+x*cos(yaw)-y*sin(yaw);
//                 pose.position.y = gy+y*cos(yaw)+x*sin(yaw);
//                 pose.position.z = 2;
//             }
//             else
//             {
//                 pose.position.x = x;
//                 pose.position.y = y;
//                 pose.position.z = 2;
//             }
            
//             marker.text = string("id: ") + to_string(it->first) + string(" vx: ") + to_string(state(4)).substr(0, 5) + string(" vy: ") + to_string(state(5)).substr(0, 5);
//             marker.pose=pose;
//             pub_rviz_bounding_box_info_.publish(marker);
//             it++;
//         }

//         // 在rviz上显示世界坐标系下bounding box
//         pcl::PointCloud<pcl::PointXYZRGB> show_point;
//         sensor_msgs::PointCloud2 msg_point;
//         showResultInRviz(global_object_, show_point, msg_point, att);
//         pub_rviz_bounding_box_.publish(msg_point);
//     }

//     // 将BaseObject类型转为决策需要的msg类型
//     outputTypeTransform(global_object_, global_pub_object_list, pub_time, veh_vxvy_, att);

//     cout << "total pub obj = " << global_pub_object_list.obstacles.size() << endl;
//     cout << "global_id = " << (int32_t)global_id_ << endl;

//     pub_obstacle_info_.publish(global_pub_object_list);

//     cout << "********************** publish : end **********************" << endl;
// }

void PerceptionFusion::publishFusionObjectWithCamera(const ros::Time pub_time, const bool is_draw, const float att[3], const map<uint32_t, BaseObject*> &publish_map)
{
    perception_msgs::FusionDataInfo global_pub_object_list;
    // 在rviz中显示检测结果
    if(is_draw_)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = ros::Duration(0.15);
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        marker.scale.z = 0.8;
        marker.color.b = 0;
        marker.color.g = 1;
        marker.color.r = 0;
        marker.color.a = 1;
        for(map<uint32_t, BaseObject*>::const_iterator it = publish_map.begin(); it != publish_map.end(); it++)
        {
            marker.id = it->first;
            geometry_msgs::Pose pose;
            Eigen:: VectorXf state = it->second->getState();
            pose.position.x = state(2);
            pose.position.y = state(3);
            pose.position.z = 2;
            marker.text = string("id: ") + to_string(it->first) + string(" vx: ") + to_string(state(4)).substr(0, 5) + string(" vy: ") + to_string(state(5)).substr(0, 5);
            marker.pose=pose;
            pub_rviz_bounding_box_info_.publish(marker);
        }
        pcl::PointCloud<pcl::PointXYZRGB> show_point;
        sensor_msgs::PointCloud2 msg_point;
        showResultInRvizWithCamera(publish_map, show_point, msg_point, att, global_rviz_);
        pub_rviz_bounding_box_.publish(msg_point);
    }
    outputTypeTransformCamera(publish_map, global_pub_object_list, pub_time, veh_vxvy_, att);
    pub_obstacle_info_.publish(global_pub_object_list);

    cout << "********************** publish : end **********************" << endl;
}

// void PerceptionFusion::publishFusionObjectWithCamera(ros::Time pub_time, bool is_draw, float att[3])
// {
//   perception_msgs::FusionDataInfo global_pub_object_list;
//   // 在rviz中显示检测结果
//   if(is_draw_)
//   {
//     visualization_msgs::Marker marker;
//     marker.header.frame_id = "odom";
//     marker.header.stamp = ros::Time::now();
//     marker.ns = "basic_shapes";
//     marker.action = visualization_msgs::Marker::ADD;
//     marker.pose.orientation.w = 1.0;
//     marker.lifetime = ros::Duration(0.06);
//     marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

//     marker.scale.z = 0.8;
//     marker.color.b = 0;
//     marker.color.g = 1;
//     marker.color.r = 0;
//     marker.color.a = 1; 
//     for(map<uint32_t, BaseObject*>::iterator it = global_object_.begin(); it != global_object_.end(); it++)
//     {
//         marker.id = it->first;
//         geometry_msgs::Pose pose;
//         Eigen:: VectorXf state = it->second->getState();
//         pose.position.x = state(2);
//         pose.position.y = state(3);
//         pose.position.z = 2;
//         marker.text = string("id: ") + to_string(it->first) + string(" vx: ") + to_string(state(4)).substr(0, 5) + string(" vy: ") + to_string(state(5)).substr(0, 5);
//         marker.pose=pose;
//         pub_rviz_bounding_box_info_.publish(marker);
//     }
//     // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//     for (int i = 0; i < eryuan_number; i++)
//     {
//         for(map<uint32_t, BaseObject*>::iterator it = eryuan_object_list[i].begin(); it != eryuan_object_list[i].end(); it++)
//         {
//             marker.id = it->first;
//             geometry_msgs::Pose pose;
//             Eigen:: VectorXf state = it->second->getState();
//             pose.position.x = state(2);
//             pose.position.y = state(3);
//             pose.position.z = 2;
//             marker.text = string("id: ") + to_string(marker.id) + string(" vx: ") + to_string(state(4)).substr(0, 5) + string(" vy: ") + to_string(state(5)).substr(0, 5);
//             marker.pose=pose;
//             pub_rviz_bounding_box_info_.publish(marker);
//         }    
//     }
//     // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//     // 在rviz上显示世界坐标系下bounding box
//     pcl::PointCloud<pcl::PointXYZRGB> show_point;
//     sensor_msgs::PointCloud2 msg_point;
//     showResultInRvizWithCamera(global_object_, show_point, msg_point, att, eryuan_object_list, eryuan_number, global_rviz_);
//     pub_rviz_bounding_box_.publish(msg_point);
//   }
//   outputTypeTransformCamera(global_object_, global_pub_object_list, pub_time, veh_vxvy_, att, eryuan_object_list, eryuan_number);
//   pub_obstacle_info_.publish(global_pub_object_list);

//   cout << "********************** publish : end **********************" << endl;
// }

// void PerceptionFusion::publishFusionObjectWithCamera(ros::Time pub_time, bool is_draw, float att[3], int sensor_type, vector<BaseObject*> base_object_list)
// {
//     cout << "********************** publish : start **********************" << endl;

//     perception_msgs::FusionDataInfo global_pub_object_list;
//     map<uint32_t, BaseObject*>::iterator it;

//     // 在rviz中显示检测结果
//     if(is_draw_)
//     {
//         visualization_msgs::Marker marker;
//         marker.header.frame_id = "/velodyne";
//         marker.header.stamp = ros::Time::now();
//         marker.ns = "basic_shapes";
//         marker.action = visualization_msgs::Marker::ADD;
//         marker.pose.orientation.w = 1.0;
//         marker.lifetime = ros::Duration(0.06);
//         marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

//         marker.scale.z = 0.8;
//         marker.color.b = 0;
//         marker.color.g = 1;
//         marker.color.r = 0;
//         marker.color.a = 1;

//         for(it = global_object_.begin(); it != global_object_.end();)
//         {
//             marker.id = it->first;
//             geometry_msgs::Pose pose;
//             Eigen:: VectorXf state = it->second->getState();
//             pose.position.x = state(2);
//             pose.position.y = state(3);
//             pose.position.z = 2;
//             marker.text = string("id: ") + to_string(it->first) + string(" vx: ") + to_string(state(4)).substr(0, 5) + string(" vy: ") + to_string(state(5)).substr(0, 5);
//             marker.pose=pose;
//             pub_rviz_bounding_box_info_.publish(marker);
//             it++;
//         }

//         // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//         for(int i = 0; i < base_object_list.size(); i++)
//         {
//             marker.id = base_object_list[i]->id;
//             geometry_msgs::Pose pose;
//             Eigen:: VectorXf state = base_object_list[i]->getState();
//             pose.position.x = state(2);
//             pose.position.y = state(3);
//             pose.position.z = 2;
//             marker.text = string("id: ") + to_string(base_object_list[i]->id) + string(" vx: ") + to_string(state(4)).substr(0, 5) + string(" vy: ") + to_string(state(5)).substr(0, 5);
//             marker.pose=pose;
//             pub_rviz_bounding_box_info_.publish(marker);
//         }
//         // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//         // 在rviz上显示世界坐标系下bounding box
//         pcl::PointCloud<pcl::PointXYZRGB> show_point;
//         sensor_msgs::PointCloud2 msg_point;
//         showResultInRvizWithCamera(global_object_, show_point, msg_point, att, base_object_list);
//         pub_rviz_bounding_box_.publish(msg_point);
//     }

//     // 将BaseObject类型转为决策需要的msg类型    
//     outputTypeTransformCamera(global_object_, global_pub_object_list, pub_time, veh_vxvy_, att, base_object_list);

//     cout << "total pub obj = " << global_pub_object_list.obstacles.size() + base_object_list.size() << endl;
//     cout << "global_id = " << (int32_t)global_id_ << endl;

//     pub_obstacle_info_.publish(global_pub_object_list);

//     cout << "********************** publish : end **********************" << endl;
// }

// void PerceptionFusion::updateEryuanObject(const perception_sensor_msgs::ObjectList msg_object_list, float att[], Vector3d &veh_llh)
// {
//     int index = msg_object_list.sensor_index - 1;
//     eryuan_object_list[index].clear();
//     for(int i = 0; i < msg_object_list.object_list.size(); i++)
//     {
//       BaseFilter* filter = new NormalKalmanFilter(msg_object_list.object_list[i].state, msg_object_list.object_list[i].measurement_cov);
//       float point4[8];
//       point4[0] = msg_object_list.object_list[i].peek[0].x;
//       point4[1] = msg_object_list.object_list[i].peek[0].y;
//       point4[2] = msg_object_list.object_list[i].peek[1].x;
//       point4[3] = msg_object_list.object_list[i].peek[1].y;
//       point4[4] = msg_object_list.object_list[i].peek[2].x;
//       point4[5] = msg_object_list.object_list[i].peek[2].y;
//       point4[6] = msg_object_list.object_list[i].peek[3].x;
//       point4[7] = msg_object_list.object_list[i].peek[3].y;
      
//       uint32_t id = msg_object_list.object_list[i].id + 10000000 * msg_object_list.sensor_index;
//       eryuan_object_list[index][id] = new CameraObject(
//                     id,
//                     msg_object_list.header.stamp,
//                     msg_object_list.object_list[i].obj_class,
//                     msg_object_list.object_list[i].confidence, 1.0, 
//                     att, veh_llh,
//                     filter, point4);
//     }
// }
