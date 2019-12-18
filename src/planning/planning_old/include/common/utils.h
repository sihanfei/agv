#ifndef UTILS_H
#define UTILS_H

#include "ros/console.h"

#include <float.h>
#include <iostream>
#include <map>
#include <math.h>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>

//#include "common/cartesian_point.h"
//#include "common/frenet_point.h"
//#include "common/reference_line.h"
//#include "common/perception_info.h"
//#include "common/control_info.h"
//#include "common/trajectory.h"

namespace planning
{
// namespace common {

//若无特殊标注，则距离单位都为m，速度单位都为m/s，时间单位都为s，角度单位都为deg，加速度减速度单位都为m/s^2

// math
const double EPSILON = 1e-6;
const double PI      = 3.14159265358979323846;

// sample point
// const double SAFE_DISTANCE = 0.5; //撒点安全距离
// const double SAFE_DISTANCE = 0.15; //撒点安全距离
const double SAFE_DISTANCE = 3.0; //撒点安全距离
// const double VEL_SAMPLE_NUM = 4;
const double VEL_SAMPLE_NUM = 6.0;

// plan
const double PLAN_TIME_RESOLUTION = 0.1; //规划时间间隔
const double PLAN_TIME            = 8.0; //规划总时间

// replan
const double OFFSET_HEADING_THRESHOLD = 10.0; //重规划的角度偏差阈值
const double OFFSET_Y_THRESHOLD       = 0.5;  //重规划的参考线偏差阈值
const double OFFSET_SPEED_THRESHOLD   = 0.5;  //重规划的速度偏差阈值
const uint32_t REPLAN_COUNT_THRESHOLD = 5;    //重规划的次数阈值，表示在上一条轨迹的执行时间
const uint32_t OFFSET_COUNT_THRESHOLD = 5; //重规划的偏差数量阈值，表示连续几次超过偏差阈值触发重规划

// stop plan
const double STOP_DISTANCE    = 0.05; //到达停车点的距离限制
const double STOP_VEL         = 0.05; //到达停车点的速度限制
const double PARKING_DISTANCE = 40.0; //开始停车规划的距离
const double CREEP_VEL        = 0.1;  //爬行速度
const double CREEP_DIS        = 0.25; //爬行距离

const double PARKING_BUFFER = 10.0;

const bool USE_REF_VEL_PLAN_FOR_PARKING = false; //是否使用参考速度进行停车规划
const bool USE_NEW_SCORE_METHOD         = false;
// const bool USE_PUBLISH_ALL_DEBUG = false;

// car attributes
const double CAR_LENGTH = 15.6; //车长
const double CAR_WIDTH  = 2.9;  //车宽
// const double VEL_LIMIT = 6.0;
// const double VEL_LIMIT = 3.0;
const double VEL_LIMIT = 1.0;

const double ACC_LIMIT = 2.109705;
const double DEC_LIMIT = 1.68;

const double LAT_ACC_LIMIT = 4.194; //横向加速度限制

const double TURN_RAD_LIMIT = 6.46; //转弯半径限制
// const double COMFORT_ACC    = 0.3;  //舒适加速度
// const double COMFORT_DEC    = 0.3;  //舒适减速度

const double COMFORT_ACC = 0.1; //舒适加速度
const double COMFORT_DEC = 0.1; //舒适减速度

 const double KAPPA_LIMIT    = 0.1165 * 2; //曲率限制
//const double KAPPA_LIMIT = 0.17;

// st graph
const double TIME_DENSITY = 1.0; //时间间隔

const double OBSTACLE_HIGH_SPEED        = 5.0;  //高速障碍物的速度值
const double HIGH_SPEED_FOLLOW_DISTANCE = 20.0; //高速障碍物的跟车距离
const double LOW_SPEED_FOLLOW_DISTANCE  = 10.0; //低速障碍物的跟车距离
const double GENERAL_FOLLOW_DISTANCE    = 3.0;  //静止障碍物的跟车距离

// score param
const double SCORE_TIME_RESOLUTION = 0.1; //对时间的采样间隔
const double SCORE_DIS_RESOLUTION  = 1.0; //对距离的采样间隔

////longitudinal speed
// const double WEIGHT_TARGET_SPEED = 10.0;//速度权重
// const double WEIGHT_DIST_TRAVELLED = 1.0;//距离权重
////const double WEIGHT_TIME_LEN = 5.0;
////longitudinal jerk
// const double LONGITUDINAL_JERK_UPPER_BOUND = 4.0;

////longitudinal collision
// const double LON_COLLISION_COST_STD = 0.5;
// const double LON_COLLISION_YIELD_BUFFER = 1.0;
// const double LON_COLLISION_OVERTAKE_BUFFER = 5.0;
////centripetal acceleration

////the offset of the center line of the lane
// const double LAT_OFFSET_BOUND = 3.0;
// const double WEIGHT_OPPOSITE_SIDE_OFFSET = 10.0;
// const double WEIGHT_SAME_SIDE_OFFSET = 1.0;

////lateral acceleration

////weight of every cost
// const double WEIGHT_LON_OBJECTIVE = 10.0;
// const double WEIGHT_LON_COMFORT = 1.0;
// const double WEIGHT_LON_COLLISION = 5.0;
// const double WEIGHT_LAT_OFFSET = 2.0;
// const double WEIGHT_LAT_COMFORT = 10.0;
// const double WEIGHT_CENTRIPETAL_ACCELERATION = 1.5;

/*
//debug param for score

//longitudinal speed
extern double WEIGHT_TARGET_SPEED;//速度权重
extern double WEIGHT_DIST_TRAVELLED;//距离权重
//const double WEIGHT_TIME_LEN = 5.0;
//longitudinal jerk
extern double LONGITUDINAL_JERK_UPPER_BOUND;

//longitudinal collision
extern double LON_COLLISION_COST_STD;
extern double LON_COLLISION_YIELD_BUFFER;
extern double LON_COLLISION_OVERTAKE_BUFFER;
//centripetal acceleration

//the offset of the center line of the lane
extern double LAT_OFFSET_BOUND;
extern double WEIGHT_OPPOSITE_SIDE_OFFSET;
extern double WEIGHT_SAME_SIDE_OFFSET;

//lateral acceleration

//weight of every cost
extern double WEIGHT_LON_OBJECTIVE;
extern double WEIGHT_LON_COMFORT;
extern double WEIGHT_LON_COLLISION;
extern double WEIGHT_LAT_OFFSET;
extern double WEIGHT_LAT_COMFORT;
extern double WEIGHT_CENTRIPETAL_ACCELERATION;
*/

// std::vector<Trajectory> all_traj_for_show;

//  根据新的损失函数，设计新的轨迹评价函数
// const double WEIGHT_LAT_KJ = 1;
extern double WEIGHT_LAT_JERK;     // 横向规划时，Jerk的权重系数
extern double WEIGHT_LAT_S_LEN;    // 横向规划时，S的权重系数
extern double WEIGHT_LAT_L_OFFSET; // 横向规划时，D的权重系数

extern double WEIGHT_LON_JERK;   // 纵向规划时，Jerk的权重系数
extern double WEIGHT_LON_T_SPEN; // 纵向规划时，T的权重系数
extern double WEIGHT_LON_V_DIF;  // 纵向规划时，D或V的权重系数

extern double WEIGHT_LON_STOP_DIS_TO_GOAL; //
extern double WEIGHT_LON_STOP_END_V;       //

extern double WEIGHT_TOTAL_LAT; // 横向轨迹的权重系数
extern double WEIGHT_TOTAL_LON; // 纵向轨迹的权重系数

static double angle2Radian(double angle)
{
  return angle * PI / 180;
}

static double radian2Angle(double radian)
{
  return radian * 180 / PI;
}

static double millimeter2Meter(uint32_t millimeter)
{
  return millimeter * 0.001;
}

static uint32_t meter2Millimeter(double meter)
{
  return meter * 1000;
}

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

// linear_interpolation插值部分
template < typename T > static T lerp(const T &x0, const double t0, const T &x1, const double t1, const double t)
{
  if (std::abs(t1 - t0) <= 1.0e-6)
  {
    // ROS_DEBUG_STREAM("input time difference is too small");
    return x0;
  }
  const double r = (t - t0) / (t1 - t0);
  const T x      = x0 + r * (x1 - x0);
  return x;
}

// extern Map* map_p_;
// extern CartesianPoint* carlocation_p_;
// extern FrenetPoint* frelocation_p_;
// extern FrenetPoint* fregoal_p_;
// extern EnviInfo* envi_info_p_;
// extern Routing* routing_p_;
// extern ReferenceLine* ref_line_p_;
// extern Traj* best_traj_p_;

//}//end namespace common
} // end namespace planning

#endif // UTILS_H
