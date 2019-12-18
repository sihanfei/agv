#ifndef STRAJECTORY_H_
#define STRAJECTORY_H_

#include "ros/ros.h"
#include "show_trajectory.h"

#include "iostream"

using namespace std;

namespace display
{

  struct positionConf
  {
    u_int32_t n_gps_sequence_num;
    double x;
    double y;
    double z;
    double lon;
    double lat;
    double height;
    double velocity_x;//velocity north
    double velocity_y;//velocity east
    double velocity_z;//velocity up	
    double heading;
    double pitch;
    double roll;
    double gps_seconds;
    double velocity;
    double dist;//里程，供saveroutepoint模式使用
  };

  class STrajectory
  {

    public:
   
      STrajectory(ros::NodeHandle &nh);
      ~STrajectory();

      void main_loop();
      void getRouteFromFile();
      void publines();

    protected:

      vector <positionConf> route_data_;
      
      visualization_msgs::Marker points,line_strip;

      ros::Publisher marker_pub;
	  
	  int mode;
	  int pathtype;
  
  };
}

#endif
