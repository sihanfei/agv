#include "show_trajectory.h"
#include "STrajectory.h"

using namespace std;
using namespace display;

namespace display
{

  STrajectory::STrajectory(ros::NodeHandle &nh)
  {
 
    cout << "Object is created." << endl;

    nh.getParam("/control/mode",mode);
	nh.getParam("/control/pathtype",pathtype);

    marker_pub = nh.advertise<visualization_msgs::Marker>("trajectory",10);

    STrajectory::getRouteFromFile();
    STrajectory::publines();

    STrajectory::main_loop();

  }

  STrajectory::~STrajectory()
  {

    cout << "Object is deleted." << endl;

  }

  void STrajectory::publines()
  {
          points.header.frame_id = line_strip.header.frame_id = "odom";
          points.header.stamp = line_strip.header.stamp = ros::Time::now();
          points.ns = line_strip.ns = "showtraj";
          points.action = line_strip.action = visualization_msgs::Marker::ADD;
          points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

          points.id = 0;
          line_strip.id = 1;

          //points.lifetime = ros::Duration(10);
          //line_strip.lifetime = ros::Duration(10);

          points.type = visualization_msgs::Marker::POINTS;
          line_strip.type = visualization_msgs::Marker::LINE_STRIP;

          //scale and line width
          points.scale.x = 1.0;
          points.scale.y = 1.0;
          line_strip.scale.x = 0.1;

          //color
          points.color.g = 1.0f;
          points.color.a = 1.0;//points are green
          line_strip.color.g = 1.0;
          line_strip.color.a = 1.0;//line strip is blue

          for(int i = 0;i < route_data_.size();i++)
          {
              geometry_msgs::Point p;
              p.x = route_data_[i].x;
              p.y = route_data_[i].y;
              p.z = 0;
 
              points.points.push_back(p);
              line_strip.points.push_back(p);
          }

          
  }

  void STrajectory::getRouteFromFile()
  {

    ROS_INFO("Start to read route file......");

    char *home_path = getenv("HOME");
    char route_data_path_name[1024] = {0};
	
	if(mode == 0)
	{
		if(pathtype == 0)//仿真模式环形路径点
		{
			sprintf(route_data_path_name,"%s"DATA_PATH""DATA_NAME_SIM_RING,home_path);
		}
		else//仿真模式正常路径点
		{
			sprintf(route_data_path_name,"%s"DATA_PATH""DATA_NAME_SIM_ROUTE,home_path);
		}
	}
	else
	{
		if(pathtype == 0)//实车模式环形路径点
		{
			sprintf(route_data_path_name,"%s"DATA_PATH""DATA_NAME_REAL_RING,home_path);
		}
		else//实车模式正常路径点
		{
			sprintf(route_data_path_name,"%s"DATA_PATH""DATA_NAME_REAL_ROUTE,home_path);
		}
	}
	
    //sprintf(route_data_path_name,"%s"DATA_PATH""DATA_NAME,home_path);
    printf("route data path name:%s\n",route_data_path_name);

    FILE * route_data_read_fp = NULL;
    route_data_read_fp = fopen(route_data_path_name,"r");
    positionConf read_position = {0};
    if(route_data_read_fp == NULL)
    {
       printf("Open route_data.bin error!\n");
       exit(1);
    }
    else
    {
       printf("route_data.bin is opened!\n");
       while(!feof(route_data_read_fp))
       {
         fscanf(route_data_read_fp,"%u %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                                  &read_position.n_gps_sequence_num,
                                  &read_position.x,
                                  &read_position.y,
                                  &read_position.z,
                                  &read_position.lon,
                                  &read_position.lat,
                                  &read_position.height,
                                  &read_position.velocity,
                                  &read_position.velocity_x,
                                  &read_position.velocity_y,
                                  &read_position.velocity_z,
                                  &read_position.heading,
                                  &read_position.pitch,
                                  &read_position.roll,
                                  &read_position.dist
                                  );
         route_data_.push_back(read_position);
       }

       fclose(route_data_read_fp);
       printf("read %lu row, route_date is ok!\n",route_data_.size());
    }
  }

  void STrajectory::main_loop()
  {

    ROS_INFO("Loop start:");
    ros::Rate loop_rate(1);

    while(ros::ok())
    {

       //marker_pub.publish(points);
       marker_pub.publish(line_strip);

       ros::spinOnce();
       loop_rate.sleep();
    }
    
  }

}
