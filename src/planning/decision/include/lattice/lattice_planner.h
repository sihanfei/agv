#ifndef LATTICE_PLANNER_H
#define LATTICE_PLANNER_H

#include "common/cartesian_point.h"
#include "common/frenet_point.h"
#include "common/ref_line.h"
#include "common/perception_Info.h"

#include "curve/curve.h"
#include "curve/quartic_polynomial.h"
#include "curve/quintic_polynomial.h"

#include "lattice/trajectory.h"
#include "lattice/trajectory_pair.h"

namespace pnc
{


class LatticePlanner
{

public:

	LatticePlanner() = default;
	~LatticePlanner() = default;

	LatticePlanner(FrenetPoint point_start,FrenetPoint point_end,ReferenceLine ref_line,PerceptionInfo envi_info,SpeedMap speed_map,FrenetPoint car_point_frenet,int lane_change_cmd,bool theta_modify_flag);

	bool isPlanningOK();						// 轨迹是否规划成功
	Trajectory getBestTrajectory();	// 返回生成好的最佳轨迹点集

	bool getStopFlag();									// 返回停车规划的标志位，20190912
	FrenetPoint getPlanningEndPoint();	// 获取lattice规划的终点，20190912


private:

	void TrajectoryGenerate_Latitude();			// 生成横向轨迹
	void TrajectoryGenerate_Longitude();		// 生成纵向轨迹
	void Trajectory_Pair();									// 横纵轨迹两两配对
	void Trajectory_Cost();									// 轨迹评分
	void Trajectory_Combine();							// 横纵轨迹集合

	bool Trajectory_Check(const Trajectory& trajectory);	// 轨迹检测：包括有效性检测和碰撞检测

	double calJerkInt_LS(const TrajectoryPair &traj_pair);		// LS轨迹JERK积分
 	double calJerkInt_ST(const TrajectoryPair &traj_pair);		// ST轨迹JERK积分

//	double calTrajectoryCost_Pair(const TrajectoryPair& traj_pair);								// 计算轨迹对的cost值：横纵向综合评分
//	double calTrajectoryCost_Lat(const TrajectoryPair& traj_pair);								// 对横向轨迹进行评分
//	double calTrajectoryCost_Lon_StopingMerging(const TrajectoryPair& traj_pair);	// 对纵向轨迹进行评分：停车或混行
//	double calTrajectoryCost_Lon_Cruise(const TrajectoryPair& traj_pair);					// 对纵向轨迹进行评分：巡航

	Box2d box_car_;		// 车辆四个顶点的包络

	FrenetPoint car_point_fre_;		// 车辆位置信息
	FrenetPoint point_start_;			// 轨迹规划起点
	FrenetPoint point_end_;				// 轨迹规划终点
	ReferenceLine ref_line_;			// 规划参考线
	SpeedMap speed_map_;					// 速度地图
	PerceptionInfo envi_info_;		// 感知信息

	bool stop_flag_;						// 停车规划标志位
	bool planning_ok_;					// 轨迹是否规划成功
	double v_cruise_;						// 巡航速度



	Trajectory best_trajectory_;	// 生成的最佳轨迹点集合
	std::vector< std::shared_ptr< Curve > > paths_vec_ls_;	// 生成的横向轨迹
	std::vector< std::shared_ptr< Curve > > paths_vec_st_;	// 生成的纵向轨迹
	std::vector< TrajectoryPair > traj_pair_vec_;	// 轨迹对


	
	double l_mid_;				// 横向撒点中间值
	int lane_change_cmd_;				// 换道命令：0--禁止换道；-1--向左换道；1--向右换道
	bool theta_modify_flag_;	//车辆航向角修改标志位，用来控制车辆是否进行斜行操作

};

}//end namespace

#endif // LATTICE_PLANNER_H

