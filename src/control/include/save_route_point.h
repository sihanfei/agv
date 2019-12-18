#ifndef SAVE_ROUTE_POINT_H_
#define SAVE_ROUTE_POINT_H_

#include "ros/ros.h"
#include <math.h>
#include "control_utils.h"
#include "control.h"
//#include "novatel_gps/NovatelPosition.h"
#include "novatel_gps_msgs/Inspva.h"
#include "novatel_gps_msgs/NovatelPosition.h"
#include <location_sensor_msgs/IMUAndGNSSInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"
#include "vector"
#include "control_msgs/com2veh.h"

using namespace std; 

namespace control
{
  class SaveRoutePoint{
    public:
      SaveRoutePoint(ros::NodeHandle &nh);
      ~SaveRoutePoint();
    
	  ////回调函数
      //void recvNovatelGPSCallback(const novalet_gps::NovatelPositionConstPtr& msg);
	  //void recvbestposCallback(const novatel_gps_msgs::NovatelPositionConstPtr& msg);
	  //void recvInspvaCallback(const novatel_gps_msgs::InspvaConstPtr& msg);
      //void recvVehDataCallback(const control_msgs::com2veh::ConstPtr &msg);
	  void recvHuacePosCallback(const location_sensor_msgs::IMUAndGNSSInfo &msg);
      void recvPrescanPosCallback(const geometry_msgs::PoseStamped &msg);
	  
      ////坐标转换函数
	  // Prescan仿真路径点记录传递函数
	  void prescanxy2xy(positionConf &xy_p,const positionConf &real_p);
      // 经纬度转换成局部坐标系函数
	  void gps2xy(positionConf &xy_p,const positionConf &real_p);
	  // 组合导航的全局坐标系转成车辆中心点的全局坐标系函数
	  void insgps2center(positionConf &xy_p);
	  
	  ////保存数据函数
      void save2File();

      ////主循环函数
      void msg2vector();
	  
	  ////键盘操作函数
      int get_char();

    protected:
	  ////flags
	  int32_t num_;
      int start_tip_;
	  int16_t save_tip_;
	  
	  ////paramaters from yaml
	  int mode;
	  int pathtype;
	  double insgps_x;
	  double insgps_y;
	  
	  double equal_length;
	  
	  double L0;
	  double lamda0;
	  double hb;
	  
	  ////structs
      vector <positionConf> route_data_;
      positionConf real_position_;
      
	  ////Subscriber and Publisher
      ros::NodeHandle nh_;
      //ros::Subscriber sub_;
	  //ros::Subscriber sub_bestpos;
	  //ros::Subscriber sub_Inspva;
      //ros::Subscriber veh_sub_;
	  ros::Subscriber pos_sub_;
      ros::Publisher pub_;
      
      
  };
}//end namespace control
#endif
