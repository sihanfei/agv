#ifndef LANECHANGE_H
#define LANECHANGE_H


#include "common/ref_line.h"
#include "common/cartesian_point.h"
#include "common/frenet_point.h"
#include "common/cartesian_frenet_converter.h"
#include "common/perception_Info.h"
#include "common/box2d.h"


namespace pnc
{

class LaneChange
{

public:

	LaneChange() = default;		// 构造函数
	~LaneChange() = default;	// 析构函数

	LaneChange(CartesianPoint car_center_cartesian,FrenetPoint target_point_frenet,ReferenceLine ref_line,PerceptionInfo envi_info);


	bool CalCarBoundary();		// 计算车辆的边界信息
	bool CalLaneRangeCarIn();	// 计算车辆所在的车道范围
	bool ClassifyObstacles();	//根据车辆位置将障碍物分为三类：左侧，中间，右侧
	double CalMinDisWithFrontObstacles(std::vector< Obstacle > obstacle_vec);	// 计算和前方障碍物的最小距离
	bool isObstacleOccupation(std::vector< Obstacle > obstacle_vec,double s_range);	//判断车辆前方一定范围内被障碍物占据


	int getLaneChangeCmd() const;		// 获取换道指令

private:

	int cmd_type_;			// 换道命令：0--禁止换道；-1--向左换道；1--向右换道

	// 车辆四个顶点信息
	std::vector< CartesianPoint > car_corner_cartesian_;		//车辆在cartesian坐标系下四个顶点坐标
	std::vector< FrenetPoint > car_corner_frenet_;					//车辆在frenet坐标系下四个顶点坐标
	CartesianPoint car_center_cartesian_;	//车辆中心点在cartesian坐标系的点
	FrenetPoint car_center_frenet_;				//车辆中心点在frenet坐标系下的坐标
	FrenetPoint target_point_frenet_;			//任务目标点在frenet坐标系下的坐标
	ReferenceLine ref_line_;							//参考线信息
	PerceptionInfo envi_info_;						//感知信息

	double car_s_max_;		// 车辆自身在frenet下的边界信息
	double car_s_min_;
	double car_l_max_;
	double car_l_min_;

	double LaneRange_CarIn_min;	//车辆所在的车道Lmin值
	double LaneRange_CarIn_max; //车辆所在的车道Lmax值

	double LaneRange_min;	//车道Lmin值
	double LaneRange_max; //车道Lmax值


	std::vector< Obstacle > obstacle_vec_left_;					// 车辆左侧的障碍物
	std::vector< Obstacle > obstacle_vec_left_middle_;	// 车辆左中的障碍物
	std::vector< Obstacle > obstacle_vec_middle_;				// 车辆道路的障碍物
	std::vector< Obstacle > obstacle_vec_right_;				// 车辆右侧的障碍物
	std::vector< Obstacle > obstacle_vec_right_middle_;	// 车辆右中的障碍物

	std::vector< LaneRange >lane_vec_left_;			// 左侧车道
	std::vector< LaneRange >lane_vec_middle_;		// 中间车道
	std::vector< LaneRange >lane_vec_right_;		// 右侧车道


};




}
#endif 
