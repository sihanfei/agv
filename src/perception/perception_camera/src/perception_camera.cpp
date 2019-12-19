#include "perception_camera.h"

sensor_camera::PerceptionCamera::PerceptionCamera(
    BaseAssociation* base_association, 
    string calibration_file,
    string image_obstacle_topic,
    string lidar_topic,
    string pub_obstacle_info_topic,
    string pub_rviz_bounding_box_topic,
    string pub_rviz_bounding_box_info_topic,
    string pub_rviz_split_pointcloud_with_camera_fov_topic,
    string pub_rviz_split_pointcloud_with_image_obstacle_info_topic,
    bool is_draw):
base_association_(base_association),
is_draw_(is_draw)
{
#ifdef DEBUG_PERCEPTION_FUSION
    cout << "PerceptionCamera ctor start" << endl;
#endif

    pub_obstacle_info_ = nh_.advertise<perception_sensor_msgs::ObjectList >(pub_obstacle_info_topic, 1);
    pub_rviz_bounding_box_ = nh_.advertise<sensor_msgs::PointCloud2>(pub_rviz_bounding_box_topic, 1);
    pub_rviz_bounding_box_info_ = nh_.advertise<visualization_msgs::Marker>(pub_rviz_bounding_box_info_topic, 1);
    pub_rviz_split_pointcloud_with_camera_fov_ = nh_.advertise<sensor_msgs::PointCloud2>(pub_rviz_split_pointcloud_with_camera_fov_topic, 1);
    pub_rviz_split_pointcloud_with_image_obstacle_info_ = nh_.advertise<sensor_msgs::PointCloud2>(pub_rviz_split_pointcloud_with_image_obstacle_info_topic, 1);

    sub_image_obstacle_info_.subscribe(nh_, image_obstacle_topic, 100);
    sub_pointcloud_.subscribe(nh_, lidar_topic, 100);

    sync_camera_.reset(new Sync(MySyncPolicy(100), sub_image_obstacle_info_, sub_pointcloud_));

    sync_camera_->registerCallback(boost::bind(&sensor_camera::PerceptionCamera::callbackcamera, this, _1, _2));

    intrinsic_ = new cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
    distCoeffs_ = new cv::Mat(5, 1, CV_32F, cv::Scalar::all(0));
    rvec_ = new cv::Mat(3, 1, CV_32F, cv::Scalar::all(0));
    tvec_ = new cv::Mat(3, 1, CV_32F, cv::Scalar::all(0));

    cv::FileStorage file_read(calibration_file, cv::FileStorage::READ);
    file_read["cameraMatrix"] >> *intrinsic_;
    file_read["distCoeffs"] >> *distCoeffs_;
    file_read["rotation vector"] >> *rvec_;
    file_read["translation vector"] >> *tvec_;
    file_read["focus"] >> focus_;
    file_read["fdx"] >> fdx_;
    file_read["fdy"] >> fdy_;
    file_read.release();

    memset(dist_image_, -500, resize_width_*resize_height_*3);

#ifdef DEBUG_PERCEPTION_FUSION
    cout << "PerceptionCamera ctor endl" << endl;
#endif
}

sensor_camera::PerceptionCamera::~PerceptionCamera()
{
    delete base_association_;
    delete dist_image_;
    delete intrinsic_;
    delete distCoeffs_;
    delete rvec_;
    delete tvec_;
}

// 传感器类型 0：相机, 1: 激光雷达, 2: 毫米板雷达
void sensor_camera::PerceptionCamera::callbackcamera(const perception_camera::CameraObstacle::ConstPtr& camera_obstacle_msg, const sensor_msgs::PointCloud2::ConstPtr& msg_lidar)
{
    cout << "====================== callbackcamera: start ======================" << endl;
    clock_t start, end;
    float elapsedTime;
    start = clock();
    
    // 关联矩阵
    Eigen::MatrixXd incidence_matrix;

    // 将目标信息转为BaseObject类型
    vector<BaseObject*> base_object_list;
    sensor_camera::inputTypeTransform(*camera_obstacle_msg, base_object_list, camera_obstacle_msg->header.stamp);

    if(base_object_list.size() != 0)
    {
        projectLidarPoints(*msg_lidar);
        splitPointCloudWithImageObstacleInfo(base_object_list);
    }

    // 处理第一帧数据
    if(global_object_.empty() && 0 != base_object_list.size())
    {
        updateNewObject(base_object_list);
    }
    else
    {
        cout << "base_object_list.size: " << base_object_list.size() << endl;

        if(0 != base_object_list.size())
        {
            // 对疑似新目标和未匹配全局目标做数据关联
            base_association_->getIncidenceMatrix(global_object_, base_object_list, incidence_matrix);

            // 更新关联上的全局目标
            updateAssociatedObject(base_object_list, incidence_matrix);
            // 更新新目标
            updateUnassociatedObject(base_object_list, incidence_matrix);
        }
    }

    publishFusionObject(camera_obstacle_msg->header.stamp, is_draw_);

    end = clock();
    elapsedTime = (float)(end - start) / CLOCKS_PER_SEC;
    cout << "Completed in: " << elapsedTime << " s" << endl;

    cout << "====================== callbackcamera: end ======================" << endl;
}

void sensor_camera::PerceptionCamera::updateNewObject(vector<BaseObject*>& obj_list)
{
#ifdef DEBUG_PERCEPTION_FUSION
    cout << "updateNewObject start" << endl;
#endif

    for(int i = 0; i < obj_list.size(); i++)
    {
        global_object_[global_id_] = obj_list[i];
        global_id_++;
    }

#ifdef DEBUG_PERCEPTION_FUSION
    cout << "updateNewObject end" << endl;
#endif
}

void sensor_camera::PerceptionCamera::updateAssociatedObject(vector<BaseObject*>& new_obj, Eigen::MatrixXd& matrix)
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
                // cout << "****************** global_id: " << it->first << " end   ****************"  << endl;
            }
            j++;
        }
    }

#ifdef DEBUG_PERCEPTION_FUSION
    cout << "updateAssociatedObject end" << endl;
#endif
}

void sensor_camera::PerceptionCamera::updateUnassociatedObject(vector<BaseObject*>& new_obj, Eigen::MatrixXd& matrix)
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


void sensor_camera::PerceptionCamera::publishFusionObject(ros::Time pub_time, bool is_draw)
{
    cout << "********************** publish : start **********************" << endl;
    
    cout << "total object before erase loss obj = " << global_object_.size() << endl;

    // 声明决策需求的数据结构
    perception_sensor_msgs::ObjectList global_pub_object_list;
    map<uint32_t, BaseObject*>::iterator it;
    for(it = global_object_.begin(); it != global_object_.end();)
    {
        if(fabs(pub_time.toSec() - (it->second)->timestamp_.toSec()) > 0.2)
        {
            delete it->second;
            global_object_.erase(it++);
        }
        else
        {
            // it->second->prediction(pub_time);
            it++;
        }
    }
    cout << "total object after erase loss obj = " << global_object_.size() << endl;

    // 在rviz中显示检测结果
    if(is_draw_)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/velodyne";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = ros::Duration(0.1);
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        marker.scale.z = 0.8;
        marker.color.b = 1;
        marker.color.g = 1;
        marker.color.r = 1;
        marker.color.a = 1;

        for(it = global_object_.begin(); it != global_object_.end();)
        {
            marker.id = it->first;
            geometry_msgs::Pose pose;
            Eigen::VectorXf state_3d = it->second->getState3d();
            Eigen::VectorXf world_state = it->second->getWorldState();
            pose.position.x = state_3d(2);
            pose.position.y = state_3d(3);
            pose.position.z = 2;
            marker.text = string("id: ") + to_string(it->first) + string(" vx: ") + to_string(world_state(2)).substr(0, 5) + string(" vy: ") + to_string(world_state(3)).substr(0, 5);
            marker.pose=pose;
            pub_rviz_bounding_box_info_.publish(marker);
            it++;
        }

        // 在rviz上显示世界坐标系下bounding box
        pcl::PointCloud<pcl::PointXYZRGB> show_point;
        sensor_msgs::PointCloud2 msg_point;
        sensor_camera::showResultInRviz(global_object_, show_point, msg_point);
        pub_rviz_bounding_box_.publish(msg_point);
    }

    // 将BaseObject类型转为决策需要的msg类型
    sensor_camera::outputTypeTransform(global_object_, global_pub_object_list, pub_time);

    cout << "total pub obj = " << global_pub_object_list.object_list.size() << endl;
    cout << "global_id = " << (int32_t)global_id_ << endl;

    pub_obstacle_info_.publish(global_pub_object_list);

    cout << "********************** publish : end **********************" << endl;
}

void sensor_camera::PerceptionCamera::projectLidarPoints(const sensor_msgs::PointCloud2& msg_lidar)
{
    // 激光点云投影到图像上
	pcl::PointCloud<pcl::PointXYZ> laser;
    pcl::fromROSMsg(msg_lidar, laser);

    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_ptr{new pcl::PointCloud< pcl::PointXYZ >};
	pcl::fromROSMsg(msg_lidar, *laser_ptr);

    memset(dist_image_, -500, resize_width_*resize_height_*3);

	vector<cv::Point3f> laserPoints;
	vector<float> laser_dist;
	for(int i = 0; i < laser.points.size(); i ++)
	{
		pcl::PointXYZ point = laser.points[i];
		pcl::PointXYZ point2;
		point2.x = point.x * cos(alpha_) - point.y * sin(alpha_);
		point2.y = point.x * sin(alpha_) + point.y * cos(alpha_);
		point2.z = point.z - 0.35;

		float angle = atan2(point2.y, point2.x) * 180.0 / M_PI;
		if(fabs(angle) > fov_)
		{
			continue;
		}

		// laserPoints.push_back(cv::Point3f(point2.x * 1000, point2.y * 1000, point2.z * 1000));
        laserPoints.push_back(cv::Point3f(point2.x, point2.y, point2.z));
	}

	vector<cv::Point2f> projectedPoints;
	cv::projectPoints(laserPoints, *rvec_, *tvec_, *intrinsic_, *distCoeffs_, projectedPoints);

    if(projectedPoints.size() != laserPoints.size())
    {
        cout << "Error projectedPoints.size() != laserPoints.size() !!!" << endl;
        exit(-1);
    }

    pcl::PointCloud<pcl::PointXYZ>  split_pointcloud;
    for(int i = 0; i < projectedPoints.size(); i++)
	{
        if(projectedPoints[i].x >= 0 && projectedPoints[i].x < camera_width_ && projectedPoints[i].y >= 0 && projectedPoints[i].y < camera_height_)
        {
            int x = projectedPoints[i].x / camera_width_ * resize_width_;
            int y = projectedPoints[i].y / camera_height_ * resize_height_;
            dist_image_[y*resize_width_ + x + resize_width_*resize_height_*0] = laserPoints[i].x;
            dist_image_[y*resize_width_ + x + resize_width_*resize_height_*1] = laserPoints[i].y;
            dist_image_[y*resize_width_ + x + resize_width_*resize_height_*2] = laserPoints[i].z;
            
            pcl::PointXYZ point;
            point.x = laserPoints[i].x;
            point.y = laserPoints[i].y;
            point.z = laserPoints[i].z;
            split_pointcloud.points.push_back(point);
        }
    }

    sensor_msgs::PointCloud2 split_pointcloud_msg;
    pcl::toROSMsg(split_pointcloud, split_pointcloud_msg);
    split_pointcloud_msg.header.frame_id = "/velodyne";
    split_pointcloud_msg.header.stamp = ros::Time::now();

    pub_rviz_split_pointcloud_with_camera_fov_.publish(split_pointcloud_msg);  
}

void sensor_camera::PerceptionCamera::splitPointCloudWithImageObstacleInfo(vector<BaseObject*>& base_object_list)
{
#ifdef DEBUG_PERCEPTION_FUSION
    cout << "splitPointCloudWithImageObstacleInfo start" << endl;
#endif
    
    pcl::PointCloud<pcl::PointXYZ>  split_pointcloud;

// #pragma omp parallel for
    for(int i = 0; i < base_object_list.size(); i++)
    {
        Eigen::VectorXf pix_state = base_object_list[i]->getPixState();
        Eigen::VectorXf world_state = base_object_list[i]->getWorldState();

        int resize_pix_xmin = pix_state[0] / camera_width_ * resize_width_;
        int resize_pix_ymin = pix_state[1] / camera_height_ * resize_height_;
        int resize_pix_xmax = pix_state[2] / camera_width_ * resize_width_;
        int resize_pix_ymax = pix_state[3] / camera_height_ * resize_height_;
        
        int width = 0;
        int dist_pix_xmin = 0;
        int dist_pix_xmax = 0;
        uint8_t object_class = base_object_list[i]->getObjectClass();
        switch(object_class)
        {
            case 2:
                width = (resize_pix_xmax - resize_pix_xmin) * 0.1;
            case 4:
                width = (resize_pix_xmax - resize_pix_xmin) * 0.7;
            case 5:
                width = (resize_pix_xmax - resize_pix_xmin) * 0.7;
        }

        dist_pix_xmin = resize_pix_xmin + width / 2;
        dist_pix_xmax = resize_pix_xmax - width / 2;

        float xmin = 99999.0;
        float ymin = 0.0;
        float zmin = 0.0;

        float max_search_dist = 0;
        float min_search_dist = 0;

        if(world_state[0] < 2.3)
        {
            max_search_dist = 2.5;
            min_search_dist = 0.0;
        }
        else if(world_state[0] <= 10 && world_state[0] > 2.3)
        {
            max_search_dist = world_state[0];
            min_search_dist = 0.1;
        }
        else if(world_state[0] < 20 && world_state[0] > 10)
        {
            max_search_dist = world_state[0];
            min_search_dist = 0.1;
        }
        else
        {
            max_search_dist = world_state[0];
            min_search_dist = 0.1;
        }
        
        for(int x = dist_pix_xmin; x < dist_pix_xmax; x++)
        {
            for(int y = resize_pix_ymin; y < resize_pix_ymax; y++)
            {
                if(dist_image_[y*resize_width_ + x + resize_width_*resize_height_*0] < 0 || 
                    dist_image_[y*resize_width_ + x + resize_width_*resize_height_*2] < -1.3 ||
                    dist_image_[y*resize_width_ + x + resize_width_*resize_height_*0] < 0.1)
                {
                    continue;
                }

                if(dist_image_[y*resize_width_ + x + resize_width_*resize_height_*0] > max_search_dist || 
                    dist_image_[y*resize_width_ + x + resize_width_*resize_height_*0] < min_search_dist)
                {
                    continue;
                }

                pcl::PointXYZ point;
                point.x = dist_image_[y*resize_width_ + x + resize_width_*resize_height_*0];
                point.y = dist_image_[y*resize_width_ + x + resize_width_*resize_height_*1];
                point.z = dist_image_[y*resize_width_ + x + resize_width_*resize_height_*2];

                split_pointcloud.points.push_back(point);

                if(dist_image_[y*resize_width_ + x + resize_width_*resize_height_*0] < xmin)
                {
                    xmin = dist_image_[y*resize_width_ + x + resize_width_*resize_height_*0];
                    ymin = dist_image_[y*resize_width_ + x + resize_width_*resize_height_*1];
                    zmin = dist_image_[y*resize_width_ + x + resize_width_*resize_height_*2];
                }
            }
        }

        if(xmin < 90000.0)
        {
            int center = (pix_state[0] + pix_state[2]) / 2;
            int center_dist = camera_width_ / 2 - center;
            float distance_y = (xmin / focus_) * (center_dist / (fdx_ / focus_));
            float width = (xmin / focus_) * (fabs(pix_state[0] - pix_state[2]) / (fdx_ / focus_));
            float hight = (xmin / focus_) * (fabs(pix_state[1] - pix_state[3]) / (fdy_ / focus_));

            vector<cv::Point3f> laserPoints = {cv::Point3f(xmin, ymin, zmin)};
            vector<cv::Point2f> projectedPoints;
	        cv::projectPoints(laserPoints, *rvec_, *tvec_, *intrinsic_, *distCoeffs_, projectedPoints);
            float pix_left_width = projectedPoints[0].x - pix_state[0];
            float pix_right_width = pix_state[2] - projectedPoints[0].x;
            float src_pix_width = pix_state[2] - pix_state[0];
            float world_left_width = width * (pix_left_width / src_pix_width);
            float world_right_width = width * (pix_right_width / src_pix_width);
            float world_ymin = ymin - world_right_width;
            float world_ymax = ymin + world_left_width;

            world_state[0] = xmin;
            world_state[1] = (world_ymax + world_ymin) / 2;
            base_object_list[i]->setWorldState(world_state);
            Eigen::VectorXf state_3d = base_object_list[i]->getState3d();

            float prior_depth = state_3d[2] - state_3d[0];

            float xmax = -10;
            for(int x = resize_pix_xmin; x < resize_pix_xmax; x++)
            {
                for(int y = resize_pix_ymin; y < resize_pix_ymax; y++)
                {
                    if(dist_image_[y*resize_width_ + x + resize_width_*resize_height_*0] < 0 || 
                        dist_image_[y*resize_width_ + x + resize_width_*resize_height_*2] < -1.0 ||
                        dist_image_[y*resize_width_ + x + resize_width_*resize_height_*0] < 0.1)
                    {
                        continue;
                    }

                    if(dist_image_[y*resize_width_ + x + resize_width_*resize_height_*0] > max_search_dist || 
                        dist_image_[y*resize_width_ + x + resize_width_*resize_height_*0] < min_search_dist)
                    {
                        continue;
                    }

                    if(dist_image_[y*resize_width_ + x + resize_width_*resize_height_*0] > xmin && 
                        dist_image_[y*resize_width_ + x + resize_width_*resize_height_*0] < xmin+prior_depth &&
                        dist_image_[y*resize_width_ + x + resize_width_*resize_height_*0] > xmax)
                    {
                        xmax = dist_image_[y*resize_width_ + x + resize_width_*resize_height_*0];
                    }
                }
            }
            float src_depth = prior_depth + xmin;
            xmax = xmax > 0 ? xmax : src_depth;
            cout << "xmin = " << xmin << endl;
            cout << "xmax = " << xmax << endl;
            cout << "prior_depth + xmin = " << prior_depth + xmin << endl;
            float new_state_3d[6] = {xmin, world_ymin, xmax, world_ymax, width, hight};

            if(xmax-xmin > 6)
            {
                cout << "--------------------------------------------------" << endl;
                cout << "--------------------------------------------------" << endl;
            }
            base_object_list[i]->setState3d(new_state_3d);
        }
    }

    sensor_msgs::PointCloud2 split_pointcloud_msg;
    pcl::toROSMsg(split_pointcloud, split_pointcloud_msg);
    split_pointcloud_msg.header.frame_id = "/velodyne";
    split_pointcloud_msg.header.stamp = ros::Time::now();
    cout << " split_pointcloud.size = " << split_pointcloud.points.size() << endl;
    pub_rviz_split_pointcloud_with_image_obstacle_info_.publish(split_pointcloud_msg);  
    
#ifdef DEBUG_PERCEPTION_FUSION
    cout << "splitPointCloudWithImageObstacleInfo end" << endl;
#endif
}