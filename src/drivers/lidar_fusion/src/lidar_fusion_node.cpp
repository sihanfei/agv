#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/common/transforms.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>
#include <pcl/common/angles.h>
#include <pcl/io/pcd_io.h>
#include "ImageSegment.h"

using namespace message_filters;

ros::Publisher pub;
ImageSegment imageSegment;

//lidar_201 --> lidar_202
Eigen::Matrix4f R12;	
//lidar_202 --> car
Eigen::Matrix4f R2c;	
// lidar (IP: 192.168.2.109) --> lidar (IP: 192.168.2.110)
Eigen::Matrix4f transform_matrix_109_110;
// lidar (IP: 192.168.2.110) --> car
Eigen::Matrix4f transform_matrix_110_car;

std::ofstream log_file("/home/wangdigang/lidar_fusion.log", std::ios::out | std::ios::app);;

pcl::PointCloud<pcl::PointXYZI> lidar, lidar1, lidar2, lidar3, lidar4;

void lidar1Callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    log_file << "Timestamp of velodyne 201: " << std::setprecision(15) << msg->header.stamp << std::endl;
    log_file << "Start timestamp of lidar fusion is: " << ros::Time::now().toSec() << std::endl;
    double start_time = ros::Time::now().toSec();
    log_file << "Start delay time is: " << std::setprecision(15) << ros::Time::now().toSec() - msg->header.stamp.toSec() << std::endl;
    lidar1.clear();
    pcl::fromROSMsg(*msg, lidar1);
    // velodyne lidar (IP: 192.168.2.201)
    imageSegment.setMinAndMaxAngle(-98.0, 9.0);
    // imageSegment.setMinAndMaxAngle(-180.0, 180.0);
    imageSegment.setPointCloud(lidar1);
    imageSegment.segment();
    pcl::PointCloud<pcl::PointXYZI> filtered = imageSegment.getFilteredCloud();
    pcl::PointCloud<pcl::PointXYZI> ground = imageSegment.getGround();
    //lidar_201 --> lidar_202
    pcl::transformPointCloud(filtered, filtered, R12);
    //lidar_201 --> car
    pcl::transformPointCloud(filtered, filtered, R2c);

    Eigen::AngleAxisf q_v(pcl::deg2rad(90.0), Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f Rinv = Eigen::Matrix4f::Identity();
    Rinv.block<3,3>(0,0) = q_v.matrix();
    pcl::transformPointCloud(filtered, filtered, Rinv);

    lidar.clear();
    lidar1 = filtered;
    lidar += lidar1;
    lidar += lidar2;
    lidar += lidar3;
    lidar += lidar4;

    sensor_msgs::PointCloud2 newMsg;
    pcl::toROSMsg(lidar, newMsg);
    newMsg.header = msg->header;
    newMsg.header.frame_id = "velodyne";
    pub.publish(newMsg);

    log_file << "End timestamp is: " << std::setprecision(15) << ros::Time::now().toSec() << std::endl;
    log_file << "End delay time is: " << std::setprecision(15) << ros::Time::now().toSec() - msg->header.stamp.toSec() << std::endl;
    double elapsed_time = ros::Time::now().toSec() - start_time;
    log_file << "Elapsed time of lidar fusion is: " << std::setprecision(15) << elapsed_time << std::endl << std::endl;
}

void lidar2Callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    log_file << "Timestamp of velodyne 202: " << std::setprecision(15) << msg->header.stamp << std::endl;
    log_file << "Start timestamp of lidar fusion is: " << ros::Time::now().toSec() << std::endl;
    double start_time = ros::Time::now().toSec();
    log_file << "Start delay time is: " << std::setprecision(15) << ros::Time::now().toSec() - msg->header.stamp.toSec() << std::endl;
    lidar2.clear();
    pcl::fromROSMsg(*msg, lidar2);
    // velodyne lidar (IP: 192.168.2.202)
    imageSegment.setMinAndMaxAngle(-6.0, 99.0);
    // imageSegment.setMinAndMaxAngle(-180.0, 180.0);
    imageSegment.setPointCloud(lidar2);
    imageSegment.segment();
    pcl::PointCloud<pcl::PointXYZI> filtered = imageSegment.getFilteredCloud();
    pcl::PointCloud<pcl::PointXYZI> ground = imageSegment.getGround();
    //lidar_202 --> car
    pcl::transformPointCloud(filtered, filtered, R2c);

    Eigen::AngleAxisf q_v(pcl::deg2rad(90.0), Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f Rinv = Eigen::Matrix4f::Identity();
    Rinv.block<3,3>(0,0) = q_v.matrix();
    pcl::transformPointCloud(filtered, filtered, Rinv);

    lidar.clear();
    lidar2 = filtered;
    lidar += lidar1;
    lidar += lidar2;
    lidar += lidar3;
    lidar += lidar4;

    sensor_msgs::PointCloud2 newMsg;
    pcl::toROSMsg(lidar, newMsg);
    newMsg.header = msg->header;
    newMsg.header.frame_id = "velodyne";
    pub.publish(newMsg);

    log_file << "End timestamp is: " << std::setprecision(15) << ros::Time::now().toSec() << std::endl;
    log_file << "End delay time is: " << std::setprecision(15) << ros::Time::now().toSec() - msg->header.stamp.toSec() << std::endl;
    double elapsed_time = ros::Time::now().toSec() - start_time;
    log_file << "Elapsed time of lidar fusion is: " << std::setprecision(15) << elapsed_time << std::endl << std::endl;
}

void lidar3Callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    log_file << "Timestamp of robotsensor 109: " << std::setprecision(15) << msg->header.stamp << std::endl;
    log_file << "Start timestamp of lidar fusion is: " << ros::Time::now().toSec() << std::endl;
    double start_time = ros::Time::now().toSec();
    log_file << "Start delay time is: " << std::setprecision(15) << ros::Time::now().toSec() - msg->header.stamp.toSec() << std::endl;
    lidar3.clear();
    pcl::fromROSMsg(*msg, lidar3);
    // robotsensor lidar (IP: 192.168.2.109)
    imageSegment.setMinAndMaxAngle(8.0, 121.0);
    // imageSegment.setMinAndMaxAngle(-180.0, 180.0);
    imageSegment.setPointCloud(lidar3);
    imageSegment.segment();
    pcl::PointCloud<pcl::PointXYZI> filtered = imageSegment.getFilteredCloud();
    pcl::PointCloud<pcl::PointXYZI> ground = imageSegment.getGround();
    // lidar (IP: 192.168.2.109) --> lidar (IP: 192.168.2.110)
    pcl::transformPointCloud(filtered, filtered, transform_matrix_109_110);
    // lidar (IP: 192.168.2.109) --> car
    pcl::transformPointCloud(filtered, filtered, transform_matrix_110_car);

    Eigen::AngleAxisf q_v(pcl::deg2rad(90.0), Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f Rinv = Eigen::Matrix4f::Identity();
    Rinv.block<3,3>(0,0) = q_v.matrix();
    pcl::transformPointCloud(filtered, filtered, Rinv);

    lidar.clear();
    lidar3 = filtered;
    lidar += lidar1;
    lidar += lidar2;
    lidar += lidar3;
    lidar += lidar4;

    sensor_msgs::PointCloud2 newMsg;
    pcl::toROSMsg(lidar, newMsg);
    newMsg.header = msg->header;
    newMsg.header.frame_id = "velodyne";
    pub.publish(newMsg);

    log_file << "End timestamp is: " << std::setprecision(15) << ros::Time::now().toSec() << std::endl;
    log_file << "End delay time is: " << std::setprecision(15) << ros::Time::now().toSec() - msg->header.stamp.toSec() << std::endl;
    double elapsed_time = ros::Time::now().toSec() - start_time;
    log_file << "Elapsed time of lidar fusion is: " << std::setprecision(15) << elapsed_time << std::endl << std::endl;
}

void lidar4Callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    log_file << "Timestamp of robotsensor 110: " << std::setprecision(15) << msg->header.stamp << std::endl;
    log_file << "Start timestamp of lidar fusion is: " << ros::Time::now().toSec() << std::endl;
    double start_time = ros::Time::now().toSec();
    log_file << "Start delay time is: " << std::setprecision(15) << ros::Time::now().toSec() - msg->header.stamp.toSec() << std::endl;
    lidar4.clear();
    pcl::fromROSMsg(*msg, lidar4);
    // robotsensor lidar (IP: 192.168.2.110)
    imageSegment.setMinAndMaxAngle(-110.0, 5.0);
    // imageSegment.setMinAndMaxAngle(-180.0, 180.0);
    imageSegment.setPointCloud(lidar4);
    imageSegment.segment();
    pcl::PointCloud<pcl::PointXYZI> filtered = imageSegment.getFilteredCloud();
    pcl::PointCloud<pcl::PointXYZI> ground = imageSegment.getGround();
    //lidar (IP: 192.168.2.110) --> car
    pcl::transformPointCloud(filtered, filtered, transform_matrix_110_car);

    Eigen::AngleAxisf q_v(pcl::deg2rad(90.0), Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f Rinv = Eigen::Matrix4f::Identity();
    Rinv.block<3,3>(0,0) = q_v.matrix();
    pcl::transformPointCloud(filtered, filtered, Rinv);

    lidar.clear();
    lidar4 = filtered;
    lidar += lidar1;
    lidar += lidar2;
    lidar += lidar3;
    lidar += lidar4;

    sensor_msgs::PointCloud2 newMsg;
    pcl::toROSMsg(lidar, newMsg);
    newMsg.header = msg->header;
    newMsg.header.frame_id = "velodyne";
    pub.publish(newMsg);

    log_file << "End timestamp is: " << std::setprecision(15) << ros::Time::now().toSec() << std::endl;
    log_file << "End delay time is: " << std::setprecision(15) << ros::Time::now().toSec() - msg->header.stamp.toSec() << std::endl;
    double elapsed_time = ros::Time::now().toSec() - start_time;
    log_file << "Elapsed time of lidar fusion is: " << std::setprecision(15) << elapsed_time << std::endl << std::endl;
}

void laserCallback(const sensor_msgs::PointCloud2::ConstPtr &msg1, const sensor_msgs::PointCloud2::ConstPtr &msg2, const sensor_msgs::PointCloud2::ConstPtr &msg3, const sensor_msgs::PointCloud2::ConstPtr &msg4)
{
    // log_file.open("lidar_fusion.log", std::ios::app);
    // std::cout << "201 = " << std::setprecision(15) << msg1->header.stamp.toSec() << std::endl;
    log_file << "Timestamp of velodyne 201: " << std::setprecision(15) << msg1->header.stamp << std::endl;
    // std::cout << "202 = " << std::setprecision(15) << msg2->header.stamp << std::endl;
    log_file << "Timestamp of velodyne 202: " << std::setprecision(15) << msg2->header.stamp.toSec() << std::endl;
    log_file << "Timestamp of robotsensor 109: " << std::setprecision(15) << msg3->header.stamp.toSec() << std::endl;
    log_file << "Timestamp of robotsensor 110: " << std::setprecision(15) << msg4->header.stamp.toSec() << std::endl;
    // std::cout << "start time = " << std::setprecision(15) << ros::Time::now().toSec() << std::endl;
    log_file << "Start timestamp of lidar fusion is: " << ros::Time::now().toSec() << std::endl;
    double start_time = ros::Time::now().toSec();
    // std::cout << "start delay time = " << std::setprecision(15) << ros::Time::now().toSec() - msg1->header.stamp.toSec() << std::endl;
    log_file << "Start delay time is: " << std::setprecision(15) << ros::Time::now().toSec() - msg1->header.stamp.toSec() << std::endl;

    // laser1 represents velodyne lidar (IP: 192.168.2.201)
    // laser2 represents velodyne lidar (IP: 192.168.2.202)
    // laser3 represents Robotsensor lidar (IP: 192.168.2.109)
    // laser4 represents Robotsensor lidar (IP: 192.168.2.110)
    pcl::PointCloud<pcl::PointXYZI> laser1, laser2, laser3, laser4;
    pcl::fromROSMsg(*msg1, laser1);
    pcl::fromROSMsg(*msg2, laser2);
    pcl::fromROSMsg(*msg3, laser3);
    pcl::fromROSMsg(*msg4, laser4);

    // velodyne lidar (IP: 192.168.2.201)
    imageSegment.setMinAndMaxAngle(-103.0, 9.0);
    // imageSegment.setMinAndMaxAngle(-180.0, 180.0);
    imageSegment.setPointCloud(laser1);
    imageSegment.segment();
    pcl::PointCloud<pcl::PointXYZI> filtered1 = imageSegment.getFilteredCloud();
    pcl::PointCloud<pcl::PointXYZI> ground1 = imageSegment.getGround();

    // velodyne lidar 202 (IP: 192.168.2.202)
    imageSegment.setMinAndMaxAngle(-6.0, 104.0);
    // imageSegment.setMinAndMaxAngle(-180.0, 180.0);
    imageSegment.setPointCloud(laser2);
    imageSegment.segment();
    pcl::PointCloud<pcl::PointXYZI> filtered2 = imageSegment.getFilteredCloud();
    pcl::PointCloud<pcl::PointXYZI> ground2 = imageSegment.getGround();

    // robotsensor lidar (IP: 192.168.2.109)
    imageSegment.setMinAndMaxAngle(8.0, 121.0);
    // imageSegment.setMinAndMaxAngle(-180.0, 180.0);
    imageSegment.setPointCloud(laser3);
    imageSegment.segment();
    pcl::PointCloud<pcl::PointXYZI> filtered3 = imageSegment.getFilteredCloud();
    pcl::PointCloud<pcl::PointXYZI> ground3 = imageSegment.getGround();

    // robotsensor lidar (IP: 192.168.2.110)
    imageSegment.setMinAndMaxAngle(-110.0, 5.0);
    // imageSegment.setMinAndMaxAngle(-180.0, 180.0);
    imageSegment.setPointCloud(laser4);
    imageSegment.segment();
    pcl::PointCloud<pcl::PointXYZI> filtered4 = imageSegment.getFilteredCloud();
    pcl::PointCloud<pcl::PointXYZI> ground4 = imageSegment.getGround();

    //lidar_201 --> lidar_202
    pcl::transformPointCloud(filtered1, filtered1, R12);
    filtered2 += filtered1;

    //lidar_202 --> car
    pcl::transformPointCloud(filtered2, filtered2, R2c);

    // lidar (IP: 192.168.2.109) --> lidar (IP: 192.168.2.110)
    pcl::transformPointCloud(filtered3, filtered3, transform_matrix_109_110);
    filtered4 += filtered3;

    // lidar (IP: 192.168.2.110) --> car
    pcl::transformPointCloud(filtered4, filtered4, transform_matrix_110_car);
    /* Eigen::AngleAxisf rotation_z(pcl::deg2rad(-10.0), Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f transform_matrix_4 = Eigen::Matrix4f::Identity();
    transform_matrix_4.block<3,3>(0,0) = rotation_z.matrix();
    transform_matrix_4(0,3) = 2.8;
    transform_matrix_4(1,3) = -1.4;
    pcl::transformPointCloud(filtered4, filtered4, transform_matrix_4); */
    filtered2 += filtered4;

    Eigen::AngleAxisf q_v(pcl::deg2rad(90.0), Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f Rinv = Eigen::Matrix4f::Identity();
    Rinv.block<3,3>(0,0) = q_v.matrix();
    pcl::transformPointCloud(filtered2, filtered2, Rinv);

    sensor_msgs::PointCloud2 newMsg;
    pcl::toROSMsg(filtered2, newMsg);
    newMsg.header = msg1->header;
    newMsg.header.frame_id = "velodyne";
    pub.publish(newMsg);

    // std::cout << "end time = " << std::setprecision(15) << ros::Time::now().toSec() << std::endl;
    log_file << "End timestamp is: " << std::setprecision(15) << ros::Time::now().toSec() << std::endl;
    log_file << "End delay time is: " << std::setprecision(15) << ros::Time::now().toSec() - msg1->header.stamp.toSec() << std::endl;
    double elapsed_time = ros::Time::now().toSec() - start_time;
    log_file << "Elapsed time of lidar fusion is: " << std::setprecision(15) << elapsed_time << std::endl << std::endl;
    // log_file.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_fusion");
    ros::NodeHandle nh;

    // lidar_201 --> lidar_202
    R12 << 0.993971, 0.108927, -0.0125533, -0.101221,
	   -0.108862, 0.99404, 0.00572173, -2.48301,
	   0.0131017, -0.00432066, 0.999905, 0.0438108,
           0, 0, 0, 1;

    // lidar202 --> car
    R2c << 0.999657, -0.0243903, 0.0095587, 7.28946,
	   0.0244497, 0.999682, -0.00613892, 1.10708,
	   -0.00940593, 0.00637046, 0.999936, -0.928643,
	   0, 0, 0, 1;
 
    // lidar (IP: 192.168.2.109) --> lidar (IP: 192.168.2.110)
    transform_matrix_109_110 << 0.978032, 0.208448, -0.00188735, -0.0278459,
				-0.207741, 0.973886, -0.0915954, 2.55217,
				-0.0172548, 0.0899753, 0.995795, -0.0572025,
			         0, 0, 0, 1;

    // lidar (IP: 192.168.2.110) --> car
    transform_matrix_110_car << -0.99938, -0.0339211, 0.00944404, -7.25144,
				0.0335975, -0.998905, -0.0325456, 1.15262,
				0.0105377, -0.0322082, 0.999426, -0.881568,
				0, 0, 0, 1;

    //时间同步器
    // message_filters::Subscriber<sensor_msgs::PointCloud2> laser201_sub(nh, "/velodyne1/velodyne_points", 100);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> laser202_sub(nh, "/velodyne2/velodyne_points", 100);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> laser201_sub(nh, "/rs1/rslidar_points", 100);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> laser202_sub(nh, "/rs2/rslidar_points", 100);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> robotsensor_109_sub(nh, "/velodyne1/velodyne_points", 100);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> robotsensor_110_sub(nh, "/velodyne2/velodyne_points", 100);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> robotsensor_109_sub(nh, "/rs1/rslidar_points", 100);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> robotsensor_110_sub(nh, "/rs2/rslidar_points", 100);
    // typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    // Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000),laser201_sub, laser202_sub, robotsensor_109_sub, robotsensor_110_sub);
    // sync.registerCallback(boost::bind(&laserCallback, _1, _2, _3, _4));

    ros::Subscriber laser201_sub = nh.subscribe("/velodyne1/velodyne_points", 1, &lidar1Callback);
    ros::Subscriber laser202_sub = nh.subscribe("/velodyne2/velodyne_points", 1, &lidar2Callback);
    ros::Subscriber robotsensor_109_sub = nh.subscribe("/rs1/rslidar_points", 1, &lidar3Callback);
    ros::Subscriber robotsensor_110_sub = nh.subscribe("/rs2/rslidar_points", 1, &lidar4Callback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/drivers/velodyne/velodyne_points", 100);

    ros::spin();
    
    return 0;
}

