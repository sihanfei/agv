#ifndef DECISION_H
#define DECISION_H

#include "common/cartesian_point.h"
#include "common/frenet_point.h"
#include "common/cartesian_frenet_converter.h"
#include "common/ref_line.h"
#include "common/control.h"
#include "common/vcu.h"
#include "common/vms_cmd.h"
#include "common/perception_Info.h"
#include "common/speed_interval.h"

#include "lattice/trajectory.h"
#include "lattice/lattice_planner.h"
#include "lattice/trajectory_pair.h"

#include "curve/curve.h"
#include "curve/quartic_polynomial.h"
#include "curve/quintic_polynomial.h"

#include "lane_change.h"


namespace pnc
{

class Decision
{

public:

    Decision(ros::NodeHandle& n);	// 构造函数
    ~Decision() = default;				// 析构函数

private:

    ros::NodeHandle n_;

		//需要订阅的消息
    ros::Subscriber vms_info_sub_;					// 远程调度消息
    ros::Subscriber perception_sub_;				// 感知消息
    ros::Subscriber location_sub_;					// 定位消息
    ros::Subscriber control_sub_;						// 控制消息
    ros::Subscriber ref_line_sub_;					// 参考线消息
    ros::Subscriber vcu_sub_;								// VCU消息
    ros::Subscriber fixed_lidar_sub_;				// 雷达消息

		// 需要发布的消息
    ros::Publisher decision_pub_;						// 决策消息
    ros::Publisher adstatus_pub_;						// 车辆状态消息
    ros::Publisher node_status_pub_;				// 节点状态消息
    ros::Publisher rviz_path_decision_pub_;	// 轨迹消息


		// 输入信息处理函数
		void vcuCallback(const control_msgs::AGVStatus::ConstPtr &vcu_msg);												// 接受VCU消息
		void VMSCmdHandle(const hmi_msgs::VMSControlAD::ConstPtr &cmd_msg);												// 接收远程VMS指令
		void locationCallback(const location_msgs::FusionDataInfo::ConstPtr &location_msg);				// 接收定位信息
		void controlCallback(const control_msgs::AGVRunningStatus::ConstPtr &control_msg);				// 接收控制信息
		void refLineCallback(const map_msgs::REFPointArray::ConstPtr &ref_line_msg);							// 接收参考线信息
		void perceptionCallback(const perception_msgs::FusionDataInfo::ConstPtr &perception_msg);	// 接收感知消息
		void fixedLidarInfoCallback (const location_sensor_msgs::FixedLidarInfo::ConstPtr& fixed_lidar_msg);	// 接收场端定位消息

		void Init();											//内部变量初始化
		void task_management();						// 任务管理
		bool isObstacleSafe();						// 障碍物安全性判断
		bool getValidCMD();								// 获取当前有效的任务 
		void PublishEstopMsg();						// 下发急停命令
		void PublishFixedLidarStopMsg();	// 下发场端精确停位命令
		void PublishBestTrajectory();			// 下发历史最佳轨迹
		void TraceData();									// 数据保存

		LatticePlanner lattice_planner_;	// Lattice轨迹规划
		LaneChange lane_change_;					// 换道策略



		// 决策变量
		uint32_t location_seq_;													//定位层的指令序列
		uint8_t error_communication_count_location_;		//通讯故障计数器，用于判断和定位层通讯是否断开
		CartesianPoint center_point_cartesian_;					//车辆中心点在cartesian坐标系下的坐标
		FrenetPoint center_point_frenet_;								//车辆中心点在frenet坐标系下的坐标

		uint32_t vcu_seq_;															//VCU的指令序列
		uint8_t error_communication_count_vcu_;					//通讯故障计数器，用于判断和VCU通讯是否断开
		VCUStatus vcu_info_;														// VCU状态消息

		uint32_t control_seq_;													//VCU的指令序列
		uint8_t error_communication_count_control_;			//通讯故障计数器，用于判断和VCU通讯是否断开
		ControlInfo control_info_;											// 车辆控制状态

		uint32_t perception_seq_;												//感知的指令序列
		uint8_t error_communication_count_perception_;	//通讯故障计数器，用于判断和感知通讯是否断开
		PerceptionInfo envi_info_;											//感知信息

		ReferenceLine ref_line_;							// 地图的参考线信息
		SpeedMap speed_map_;									// 速度地图

		uint8_t replan_obs_count_;		// 障碍物重规划计数器
		uint8_t replan_count_;				// 重规划计数器

		std::vector<VMS_Cmd> VMS_Cmd_list_;					//	VMS指令序列
		bool CMD_finish_flag_;											//指令完成标志位
    FrenetPoint planning_start_point_frenet_;		// planning的规划起点
    FrenetPoint planning_end_point_frenet_;			// planning的规划终点
		Trajectory best_trajectory_;								// 最佳轨迹信息


		// 变量锁
    mutable boost::mutex location_mutex_;
    mutable boost::mutex vcu_mutex_;
    mutable boost::mutex control_mutex_;
    mutable boost::mutex ref_line_mutex_;
    mutable boost::mutex perception_mutex_;

		// 场端探测到的信息
		double target_dis_relative_x_;	// 场端探测到的AGV距离目标点的相对X方向相对位置
		double target_dis_relative_y_;	// 场端探测到的AGV距离目标点的相对Y方向相对位置
		double target_dis_relative_z_;	// 场端探测到的AGV距离目标点的相对Z方向相对位置

		// 车辆四个顶点信息
		std::vector< CartesianPoint > car_corner_cartesian_;		//车辆在cartesian坐标系下四个顶点坐标
		std::vector< FrenetPoint > car_corner_frenet_;					//车辆在frenet坐标系下四个顶点坐标
		double car_s_max_;		// 车辆自身在frenet下的边界信息
		double car_s_min_;
		double car_l_max_;
		double car_l_min_;

		// 模块准备信息
		bool Ready_vcu_;				//VCU准备就绪
		bool Ready_location_;		//位置信息准备就绪
		bool Ready_control_;		//控制信息准备就绪
		bool Ready_perception_;	//感知信息准备就绪
		bool Ready_ref_line_;		//参考线准备就绪
		bool Ready_car_all_;		//车辆准备就绪，所有准备就绪信号都正常，则该位才表示正常

		// 障碍物
		double min_dis_obs_front_car_;	// 前方障碍物和车辆的最小距离，实时量
		double s_min_obs_front_car_;	// 前方障碍物最小的s值，最近历史值，可能不是最新的
		bool accurate_stop_flag_;	// 场端探测到的精确停车标志位
		bool Estop_flag_;						// 急停标志位
		bool obs_stop_flag_;				// 障碍物停车标志位		
		bool obs_stop_flag_pre_;		// 上一时刻的障碍物停车标志位		
		uint8_t obs_disappeared_count;	//障碍物消失计数器
		uint8_t error_planning_count;		//轨迹规划失败计数器

		int cmd_type_;			// 换道命令：0--禁止换道；-1--向左换道；1--向右换道

		bool theta_modify_flag_;	//车辆航向角修改标志位，用来控制车辆是否进行斜行操作



		int TraceDataCount;	// 文本保存计数器
		std::vector< std::string > decision_data_vec_;					//需要保存的决策信息
		std::vector< std::string > obstacle_data_vec_;					//需要保存的障碍物信息
		std::vector< std::string > trajectory_data_vec_;				//需要保存的障碍物信息

};


}
#endif 
