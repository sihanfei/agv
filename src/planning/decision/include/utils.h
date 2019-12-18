#ifndef UTILS_H
#define UTILS_H

// 系统头文件
#include "ros/ros.h"
#include "ros/console.h"
#include <float.h>
#include <iostream>
#include <map>
#include <math.h>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>
#include <stdint.h>
#include <inttypes.h>
#include <cstdlib>
#include <fstream>
#include <time.h>
#include <stdio.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <sys/stat.h> 　
#include <sys/types.h> 　
#include <dirent.h> 


// 消息头文件
#include <hmi_msgs/VMSControlAD.h>
#include <hmi_msgs/ADStatus.h>
#include <perception_msgs/FusionDataInfo.h>
#include <location_msgs/FusionDataInfo.h>
#include <map_msgs/REFPointArray.h>
#include <plan_msgs/DecisionInfo.h>
#include <control_msgs/AGVRunningStatus.h>
#include <control_msgs/AGVStatus.h>
#include <location_sensor_msgs/FixedLidarInfo.h>
#include <status_msgs/NodeStatus.h>
#include <visualization_msgs/MarkerArray.h>

// 自定义头文件
#include "common/cartesian_point.h"
#include "common/frenet_point.h"
#include "common/cartesian_frenet_converter.h"
#include "common/ref_line.h"
#include "common/control.h"
#include "common/vcu.h"
#include "common/vms_cmd.h"
#include "common/perception_Info.h"
#include "common/speed_interval.h"
#include "common/box2d.h"
#include "curve/curve.h"
#include "curve/quartic_polynomial.h"
#include "curve/quintic_polynomial.h"
#include "lattice/trajectory.h"
#include "lattice/lattice_planner.h"
#include "lattice/trajectory_curve.h"
#include "lattice/trajectory_pair.h"
#include "decision/decision.h"
#include "decision/lane_change.h"

namespace pnc
{
// namespace common {

//若无特殊标注，则距离单位都为m，速度单位都为m/s，时间单位都为s，角度单位都为deg，加速度减速度单位都为m/s^2

// math
const double EPSILON = 1e-5;
const double PI      = 3.14159265358979323846;
const double MAX_NUM = 100000000000000000000.0;

const double MAX_COM_ERROR_COUNT = 5.0; 				//最大容忍通讯失败次数
const double MAX_OBS_DISAPPEAREND_COUNT = 50.0;	// 障碍物消失计数器阈值
const double MAX_PLANNING_ERROR_COUNT = 5.0; 		// 规划失败计数器阈值

// decision
const double DISTANCE_POSITION_JUMP = 5.0;		// 位置跳变容忍距离 m
const double DISTANCE_HEADING_JUMP = 10.0;		// 位置跳变容忍距离 °
const double DISTANCE_OBSTRACLE_STOP = 20.0;		// 触发障碍物停车规划的距离
const double DISTANCE_TASK_FINSH_X = 0.10;			// 指令完成X方向的容忍值
const double DISTANCE_TASK_FINSH_Y = 0.10;			// 指令完成X方向的容忍值
const double DISTANCE_FIXED_LIDAR_VALID = 3.0;	// 场端数据有效距离


// lanechange
const double WidthLane				 =  3.6;				// 标准车道宽4m
const int LANE_CHANGE_FORBID	 =  0.0;			// 车辆禁止换道
const int LANE_CHANGE_LEFT		 = -1.0;			// 车辆向左换道
const int LANE_CHANGE_RIGHT		 =  1.0;			// 车辆向右换道
const double DISTANCE_LANE_CHANGE = 25.0;	  	//换道触发距离



// plan
const double SAFE_DISTANCE_LON = 5.0;			//纵向安全距离
const double SAFE_DISTANCE_LAT = 0.5;			//横向安全距离
const double DISTANCE_VEL_CHANGE = 10.0;	  //速度提前切换距离
const double DISTANCE_STOP_PLAN = 10.0;	  	//停车规划提前距离
const double VEL_LIMIT = 3.0;				//允许的最大速度
const double VEL_START = 0.1;				//需要的最小启动速度
const double PLAN_TIME_RESOLUTION = 0.1; //规划时间间隔
const double PLAN_TIME_MAX        = 8.0; //最大规划时间
const double PLAN_TIME_MIN        = 1.0; //最小规划时间
const double TIME_DENSITY 				= 1.0; //时间间隔
const double VEL_SAMPLE_NUM 			= 6.0; //将速度撒样区间分为6等分

const double L_MAX_OFFSET					= 8.0; //最大横向撒点值
const double L_RESOLUTION 				= 0.2;	//横向撒点间隔


// replan
const double OFFSET_HEADING_THRESHOLD = 10.0; //重规划的角度偏差阈值
const double OFFSET_Y_THRESHOLD       = 0.5;  //重规划的参考线偏差阈值
const double OFFSET_SPEED_THRESHOLD   = 0.5;  //重规划的速度偏差阈值
const uint32_t REPLAN_COUNT_THRESHOLD = 5;    //重规划的次数阈值，表示在上一条轨迹的执行时间
const uint32_t OFFSET_COUNT_THRESHOLD = 5; 		//重规划的偏差数量阈值，表示连续几次超过偏差阈值触发重规划

// stop plan
const double STOP_DISTANCE    = 0.05; //到达停车点的距离限制
const double STOP_VEL         = 0.05; //到达停车点的速度限制
const double PARKING_DISTANCE = 40.0; //开始停车规划的距离
const double CREEP_VEL        = 0.1;  //爬行速度
const double CREEP_DIS        = 0.25; //爬行距离



// 车体性能参数
const double CAR_LENGTH = 15.6; 		//车长
const double CAR_WIDTH  = 2.9;  		//车宽


const double ACC_LIMIT = 2.109705;	// 最大加速度
const double DEC_LIMIT = 1.68;			// 最大减速度
const double COMFORT_ACC    = 0.3;  //舒适加速度
const double COMFORT_DEC    = 0.3;  //舒适减速度


const double LAT_ACC_LIMIT  = 4.6125;	//横向加速度限制
const double KAPPA_LIMIT    = 0.216;	//曲率限制

const double TURN_RAD_LIMIT = 6.46;		//转弯半径限制



const double OBSTACLE_HIGH_SPEED        = 5.0;  //高速障碍物的速度值
const double HIGH_SPEED_FOLLOW_DISTANCE = 20.0; //高速障碍物的跟车距离
const double LOW_SPEED_FOLLOW_DISTANCE  = 10.0; //低速障碍物的跟车距离
const double GENERAL_FOLLOW_DISTANCE    = 10.0;  //静止障碍物的跟车距离

// score param
const double SCORE_TIME_RESOLUTION = 0.1; //对时间的采样间隔
const double SCORE_DIS_RESOLUTION  = 0.2; //对距离的采样间隔


// 用于轨迹的评价参数
const double WEIGHT_TOTAL_LAT = 1.0; // 横向轨迹的权重系数
const double WEIGHT_TOTAL_LON = 1.0; // 纵向轨迹的权重系数
const double WEIGHT_LAT_JERK = 1.0;	// 横向规划时，Jerk的权重系数
const double WEIGHT_LAT_S = 1.0;		// 横向规划时，S的权重系数（路径的长度）
const double WEIGHT_LAT_L = 1.0;		// 横向规划时，L的权重系数
const double WEIGHT_LON_JERK = 1.0;	// 纵向规划时，Jerk的权重系数
const double WEIGHT_LON_T = 1.0;		// 纵向规划时，T的权重系数
const double WEIGHT_LON_S = 1.0;		// 纵向规划时，S的权重系数(停车，靠近目标点的距离)
const double WEIGHT_LON_V = 1.0;		// 纵向规划时，V的权重系数(巡航)


// 用于文件保存
const double MAX_TRACE_NUM = 600.0; // 每个文件最大数据流，由于程序执行的周期为10HZ，取值600代表60s保存文件一次
const double MAX_FILE_NUM = 60.0; 	// 能保存的最多文件数


/*将角度任意圆整到：0~+2*PI*/
static double wrapTo2PI(double lambda)
{
  double b;
  b = fmod(lambda, 2.0 * PI);
  if (b < 0)
  {
    b = b + 2.0 * PI;
  }
  return b;
}

/*将角度任意圆整到：-PI~+PI*/
static double wrapToPI(double lambda)
{
  double b = lambda;
  if (lambda > PI || lambda < -PI)
  {
    b = wrapTo2PI(lambda + PI) - PI;
  }
  return b;
}

// 将角度变为弧度
static double angle2Radian(double angle)
{
  return angle * PI / 180;
}
// 将弧度变为角度
static double radian2Angle(double radian)
{
  return radian * 180 / PI;
}
// 将mm转换为m
static double millimeter2Meter(uint32_t millimeter)
{
  return millimeter * 0.001;
}
// 将m转换为mm
static uint32_t meter2Millimeter(double meter)
{
  return meter * 1000;
}



// linear_interpolation插值部分
template < typename T > static T lerp(const T &x0, const double t0, const T &x1, const double t1, const double t)
{
  if (std::abs(t1 - t0) <= EPSILON)
  {
    return x0;
  }
  const double r = (t - t0) / (t1 - t0);
  const T x      = x0 + r * (x1 - x0);
  return x;
}




static double ThetaTransform(double theta)
{
	// 由于坐标系不一致，需要将角度进行相位的相互转换
	if(theta > 0.5*PI)
	{
		return PI*2.5 - theta;
	}
	else
	{
		return PI*0.5 - theta;
	}
}







} 

#endif // UTILS_H

