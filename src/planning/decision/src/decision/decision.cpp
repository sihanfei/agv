#include "utils.h"


namespace pnc
{

Decision::Decision(ros::NodeHandle &n) : n_(n)
{
	// 初始化
	Init();

	// 接收参考线topic
  ref_line_sub_ = n_.subscribe< map_msgs::REFPointArray >("/map/ref_point_info", 10, &Decision::refLineCallback, this);
  //接收vcu状态topic
  vcu_sub_ = n_.subscribe< control_msgs::AGVStatus >("/drivers/com2agv/agv_status", 10, &Decision::vcuCallback, this);
	 //接收调度topic
  vms_info_sub_ = n_.subscribe< hmi_msgs::VMSControlAD >("/map/vms_control_info", 10, &Decision::VMSCmdHandle, this);
  //接收定位topic
  location_sub_ = n_.subscribe < location_msgs::FusionDataInfo >("/localization/fusion_msg", 10, &Decision::locationCallback, this); 
  //接收控制topic
  control_sub_ = n_.subscribe< control_msgs::AGVRunningStatus >("/control/agv_running_status", 10, &Decision::controlCallback, this);	
  //接收感知topic
  perception_sub_ = n_.subscribe < perception_msgs::FusionDataInfo >("/perception/obstacle_info", 10, &Decision::perceptionCallback, this);  
	//接收场端topic
	fixed_lidar_sub_  = n_.subscribe< location_sensor_msgs::FixedLidarInfo >("/drivers/localization/fixed_lidar_msg", 10, &Decision::fixedLidarInfoCallback,this);

  //发布轨迹
  decision_pub_ = n_.advertise< plan_msgs::DecisionInfo >("/plan/decision_info", 10); 
  //发布节点状态给monitor
  node_status_pub_ = n_.advertise< status_msgs::NodeStatus >("/node/node_status", 10);
  //仿真debug
  rviz_path_decision_pub_ = n_.advertise< visualization_msgs::MarkerArray >("/monitor/rviz_path_decision_info", 10);

  //创建执行任务的thread
  boost::thread task_management_thread(boost::bind(&Decision::task_management, this));

}


void Decision::Init() // 内部变量初始化
{
	ROS_INFO("Decision Node start init ...... ");

	center_point_cartesian_.setX(0.0);
	center_point_cartesian_.setY(0.0);
	center_point_cartesian_.setTheta(0.0);
	center_point_cartesian_.setKappa(0.0);
	center_point_cartesian_.setVel(0.0);
	center_point_cartesian_.setAcc(0.0);

	center_point_frenet_.setS(0.0);
	center_point_frenet_.setdS(0.0);
	center_point_frenet_.setddS(0.0);
	center_point_frenet_.setL(0.0);
	center_point_frenet_.setdL(0.0);
	center_point_frenet_.setddL(0.0);

	planning_start_point_frenet_.setS(0.0);
	planning_start_point_frenet_.setdS(0.0);
	planning_start_point_frenet_.setddS(0.0);
	planning_start_point_frenet_.setL(0.0);
	planning_start_point_frenet_.setdL(0.0);
	planning_start_point_frenet_.setddL(0.0);


	planning_end_point_frenet_.setS(0.0);
	planning_end_point_frenet_.setdS(0.0);
	planning_end_point_frenet_.setddS(0.0);
	planning_end_point_frenet_.setL(0.0);
	planning_end_point_frenet_.setdL(0.0);
	planning_end_point_frenet_.setddL(0.0);

	car_s_max_ = 0;
	car_s_min_ = 0;
	car_l_max_ = 0;
	car_l_min_ = 0;

	VMS_Cmd_list_.clear();

	target_dis_relative_x_ = 0;
	target_dis_relative_y_ = 0;
	target_dis_relative_z_ = 0;

	Ready_vcu_ = 0;
	Ready_location_ = 0;
	Ready_control_ = 0;
	Ready_perception_ = 0;
	Ready_ref_line_ = 0;
	Ready_car_all_ = 0;

	min_dis_obs_front_car_ = 0;	
	s_min_obs_front_car_ = 0;
  accurate_stop_flag_ = 0;
	Estop_flag_ = 0;
	obs_stop_flag_ = 0;	
	obs_stop_flag_pre_ = 0;	
	obs_disappeared_count = 0;
	CMD_finish_flag_ = 0;

	cmd_type_ = 0;
	theta_modify_flag_ = 0;

	TraceDataCount = 0;
	decision_data_vec_.clear();
	obstacle_data_vec_.clear();
	trajectory_data_vec_.clear();


	// 新建Trace文件保存的文件夹 20191011
	if(-1 == access("TraceData",0)) 
	{
		if(0 != mkdir("TraceData",S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
		{
			ROS_WARN("Decision--Init:Cannot Creat TraceLog Folder.");
		}
	}
	if(-1 == access("TraceData/Decision",0)) 
	{
		if(0 != mkdir("TraceData/Decision",S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
		{
			ROS_WARN("Decision--Init:Cannot Creat DecisionData Folder.");
		}
	}
	if(-1 == access("TraceData/Perception",0)) 
	{
		if(0 != mkdir("TraceData/Perception",S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
		{
			ROS_WARN("Decision--Init:Cannot Creat PerceptionData Folder.");
		}
	}
	if(-1 == access("TraceData/Trajectory",0)) 
	{
		if(0 != mkdir("TraceData/Trajectory",S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
		{
			ROS_WARN("Decision--Init:Cannot Creat TrajectoryData Folder.");
		}
	}

	ROS_INFO("Decision Node Init OK .");
}


void Decision::vcuCallback(const control_msgs::AGVStatus::ConstPtr &vcu_msg)
{
	// 函数功能：处理收到的VCU信息
	Ready_vcu_ = 0;

/*
	// 仿真环境中不考虑通讯判断
	uint32_t seq = vcu_msg->header.seq;
	if (vcu_seq_ + 1 != seq)
	{nt lane range left is  = -11.814555,
		error_communication_count_vcu_++;
		if (error_communication_count_vcu_ > MAX_COM_ERROR_COUNT)
		{
			error_communication_count_vcu_ = 0;
			ROS_ERROR("Decision:vcu sequence mismatch.");
		}
	}
	else
	{
		error_communication_count_vcu_ = 0;
	}
	vcu_seq_ = seq;
*/

	// 保留该有效的VCU信息
	vcu_mutex_.lock();
	vcu_info_ = VCUStatus(vcu_msg->VEHMode, vcu_msg->VEHFlt, vcu_msg->SOC,
												vcu_msg->HVStatus, vcu_msg->EStopStatus,vcu_msg->LiftStatus, vcu_msg->Rolling_Counter);
	vcu_mutex_.unlock();
	

	
	// 根据VCU信息进行故障检测
	if(vcu_msg->VEHFlt == 3)		//车辆存在三级故障
	{
		ROS_ERROR("Decision--VCU:VCU is high fault.");
		return;
	}

	if(vcu_msg->HVStatus == 0)	//车辆未处于高压状态
	{
		ROS_ERROR("Decision--VCU:AGV is not at start status.");
		return;
	}

	if(vcu_msg->SOC == 3)			//VCU电量过低
	{
		ROS_ERROR("Decision--VCU:VCU is low battery.");
		return;
	}

	Ready_vcu_ = 1;
}

void Decision::VMSCmdHandle(const hmi_msgs::VMSControlAD::ConstPtr &cmd_msg)
{
	// 函数功能：处理收到的远程VMS指令

  ROS_INFO("Decision--VMS:Received a VMS cmd: id=%d; type=%d; x=%f; y=%f; heading=%f",
cmd_msg->task_ID, cmd_msg->CMD_Type,cmd_msg->target_position.x,cmd_msg->target_position.y,ThetaTransform(angle2Radian(cmd_msg->target_position.heading)));
/*
	// 根据车辆的控制模式，判断是否要接收VMS指令信息
	if(vcu_info_.getControlMode() != 2)
	{
		ROS_ERROR("Decision--VMS:agv control mode is not auto,ingnore the vms cmd.");
		return;
	}
*/

	// 根据收到的调度信息，生成指令信息
	VMS_Cmd vms_temp = VMS_Cmd(std::to_string(cmd_msg->task_ID), cmd_msg->CMD_Type,cmd_msg->target_position.x,cmd_msg->target_position.y,ThetaTransform(angle2Radian(cmd_msg->target_position.heading)));

	// 需要增加一些额外的指令有效性检测，比如，目标点是否会导致车辆超出地图范围

	// 判断该指令是否为新指令
	if(VMS_Cmd_list_.empty()) // 原来不存在指令信息，则将该指令直接加入到指令列表中去
	{
		ROS_INFO("Decision--VMS: it is a New VMS cmd.");
		VMS_Cmd_list_.emplace_back(vms_temp);
		CMD_finish_flag_ = 0;
		Ready_ref_line_ = 0;
	}
	else
	{
		std::string id_frist = VMS_Cmd_list_.front().getID();	// 获取第一条指令的指令ID
		if(id_frist == std::to_string(cmd_msg->task_ID)) // 收到了新指令，到时指令序号没有更新，不将该指令放到指令序列中去
		{
 			ROS_INFO("Decision--VMS: the task id is not update. vms cmd invalid.");
		}
		else
		{
			VMS_Cmd_list_.emplace_back(vms_temp);
			ROS_INFO("Decision--VMS: it is a New VMS cmd.");
			CMD_finish_flag_ = 0;
			Ready_ref_line_ = 0;
		}
	}


/*
	// 考虑到是先有指令后有参考线，所以，不能在收到指令的那一瞬间就直接将指令进行转换
	// 判断参考线是否存在，若不存在，则直接返回false
	if(Ready_ref_line_ == 0 )
	{
		ROS_WARN("Decision--VMS: the ref line is not ready. ");
		return;
	}

	// 将目标指令信息坐标转换到frenet坐标系下
	CartesianPoint point_temp;
	point_temp.setX(cmd_msg->target_position.x);
	point_temp.setY(cmd_msg->target_position.y);
	point_temp.setTheta(ThetaTransform(angle2Radian(cmd_msg->target_position.heading)));
	point_temp.setVel(0.0);
	point_temp.setAcc(0.0);
	point_temp.setKappa(0.0);

	FrenetPoint point_frenet_temp;
	if (!cartesianToFrenet(point_temp, ref_line_, point_frenet_temp))
	{
		ROS_ERROR("Decision--VMS: VMS target cartesian point transform to Frenet failed.");
		return;
	}

	if(point_frenet_temp.getS() + 0.5*CAR_LENGTH > ref_line_.getReferenceLineMaxS())
	{
		ROS_ERROR("Decision--VMS: VMS target frenet point is invalid , out of the refline.");
		return;
	}
*/

}



void Decision::locationCallback(const location_msgs::FusionDataInfo::ConstPtr &location_msg)
{
	// 函数功能：处理收到的位置信息
	Ready_location_ = 0;

/*
	// 仿真环境中不考虑通讯判断
	//判断位置信息是否更新	
	uint32_t seq = location_msg->header.seq;
	if (location_seq_ + 1 != seq)
	{
		error_communication_count_location_++;
		if (error_communication_count_location_ > MAX_COM_ERROR_COUNT)
		{
			error_communication_count_location_ = 0;
			ROS_ERROR("Decision:Locatiion sequence mismatch.");
			return;
		}
	}
	else
	{
		error_communication_count_location_ = 0;
	}
	location_seq_ = seq;locationCallback
*/

	double theta_radian = ThetaTransform(angle2Radian(location_msg->yaw));	// 将角度转换为弧度
	double v_car = sqrt(pow(location_msg->velocity.linear.x, 2) + pow(location_msg->velocity.linear.y, 2));
	double a_var = sqrt(pow(location_msg->accel.linear.x, 2) + pow(location_msg->accel.linear.y, 2));


	ROS_INFO("Decision--Location: x = %f,y = %f,heading = %f,v = %f,a = %f.",location_msg->pose.x,location_msg->pose.y, theta_radian,v_car,a_var);


	// 判断参考线是否存在，若不存在，则直接返回false
	if(Ready_ref_line_ ==0 )
	{
		ROS_WARN("Decision--Location: the ref line is not ready. ");
		return;
	}

	// 判断位置信息是否跳变
	if(center_point_cartesian_.getX() != 0 && center_point_cartesian_.getY() != 0)	// 不是第一次收到位置信息
	{
		double dis_offset = sqrt(pow(center_point_cartesian_.getX()-location_msg->pose.x, 2) + pow(center_point_cartesian_.getY()-location_msg->pose.y, 2));

		//注意：定位给的角度单位为°，内部计算用的角度是弧度
		double theat_offset = fabs(radian2Angle(wrapToPI(center_point_cartesian_.getTheta()-theta_radian)));	

		if(dis_offset >= DISTANCE_POSITION_JUMP || theat_offset >= DISTANCE_HEADING_JUMP)	// 中心点位置跳变位置超过5m或者航向角超过10°，则报错
		{
				ROS_WARN("Decision--Location: Location jump too much.dis_offset = %f,theat_offset = %f.",dis_offset,theat_offset);
				return;
		}
	}

	// 将坐标转换到frenet坐标系下
	CartesianPoint car_point;
	car_point.setX(location_msg->pose.x);
	car_point.setY(location_msg->pose.y);
	car_point.setTheta(theta_radian);
	car_point.setVel(v_car);
	car_point.setAcc(a_var);
	car_point.setKappa(0.0);

	FrenetPoint car_point_frenet;
	if (!cartesianToFrenet(car_point, ref_line_, car_point_frenet))
	{
		ROS_WARN("Decision--Location: car center position cartesian transform to Frenet failed.");
		return;
	}
/*
	// 取消车辆位置有效性尾部超过车道参考线开始位置的判断，这是因为地图发出来的参考线不是一条完整的全局路径参考线，有可能参考线的起点就刚好超过车体了 20190917
	// 判断位置信息是否在有效地图范围内
	if((car_point_frenet.getS() - 0.5*CAR_LENGTH < 0.0) || (car_point_frenet.getS() + 0.5*CAR_LENGTH > ref_line_.getReferenceLineMaxS()))
	{
		ROS_ERROR("Decision--Location: center frenet point is invalid , out of the refline.");
		return;
	}
*/
	Ready_location_ = 1;

	ROS_INFO("Decision--Location: Car's Pose is valid. s = %f,l = %f.",car_point_frenet.getS(),car_point_frenet.getL());

	// 通过车辆中心点坐标，计算车辆四个顶点的坐标
	double x[4],y[4];
	x[0] = location_msg->pose.x + CAR_LENGTH/2*cos(theta_radian) - CAR_WIDTH/2*sin(theta_radian); 
	y[0] = location_msg->pose.y + CAR_LENGTH/2*sin(theta_radian) + CAR_WIDTH/2*cos(theta_radian);
	x[1] = location_msg->pose.x + CAR_LENGTH/2*cos(theta_radian) + CAR_WIDTH/2*sin(theta_radian); 
	y[1] = location_msg->pose.y + CAR_LENGTH/2*sin(theta_radian) - CAR_WIDTH/2*cos(theta_radian); 
	x[2] = location_msg->pose.x - CAR_LENGTH/2*cos(theta_radian) + CAR_WIDTH/2*sin(theta_radian); 
	y[2] = location_msg->pose.y - CAR_LENGTH/2*sin(theta_radian) - CAR_WIDTH/2*cos(theta_radian); 
	x[3] = location_msg->pose.x - CAR_LENGTH/2*cos(theta_radian) - CAR_WIDTH/2*sin(theta_radian); 
	y[3] = location_msg->pose.y - CAR_LENGTH/2*sin(theta_radian) + CAR_WIDTH/2*cos(theta_radian); 

	// 计算车辆在frenet坐标下最大最小的s值和l值
	double s_temp_max = -MAX_NUM;
	double s_temp_min = MAX_NUM;
	double l_temp_max = -MAX_NUM;
	double l_temp_min = MAX_NUM;

	for(int i = 0; i < 4; i++)
	{
		CartesianPoint p_cartesian;
		p_cartesian.setX(x[i]);
		p_cartesian.setY(y[i]);
		p_cartesian.setTheta(theta_radian);
		p_cartesian.setVel(v_car);
		p_cartesian.setAcc(a_var);
		p_cartesian.setKappa(0.0);

		// 将坐标转换到frenet坐标系下
		FrenetPoint p_frenet;
		if (!cartesianToFrenet(p_cartesian, ref_line_, p_frenet))
		{
			ROS_WARN("Decision--Location: car corner %d point cartesian transform to Frenet failed.",i);
			return;
		}

		// 保存车辆四个顶点在frenet和cartesian坐标下的坐标
		car_corner_cartesian_.push_back(p_cartesian);
		car_corner_frenet_.push_back(p_frenet);
		
		// 计算frenet边界最大最小范围
		if(s_temp_max < p_frenet.getS())
		{
			s_temp_max = p_frenet.getS();
		}
		if(s_temp_min > p_frenet.getS())
		{
			s_temp_min = p_frenet.getS();
		}

		if(l_temp_max < p_frenet.getL())
		{
			l_temp_max = p_frenet.getL();
		}
		if(l_temp_min > p_frenet.getL())
		{
			l_temp_min = p_frenet.getL();
		}
	}

	// 设置车辆的边界信息
	car_s_max_ = s_temp_max;
	car_s_min_ = s_temp_min;
	car_l_max_ = l_temp_max;
	car_l_min_ = l_temp_min;


	ROS_INFO("Decision--Location: s_max = %f,s_min = %f,l_max = %f,l_min = %f",car_s_max_,car_s_min_,car_l_max_,car_l_min_);

	// 保留该有效的位置信息
	location_mutex_.lock();
	center_point_cartesian_ = car_point;
	center_point_frenet_ = car_point_frenet;
	location_mutex_.unlock();     

	
//	Ready_location_ = 1;

}

void Decision::controlCallback(const control_msgs::AGVRunningStatus::ConstPtr &control_msg)
{
	// 函数功能：处理收到的控制信息
	Ready_control_ = 0;
/*
  //判断序列号是否合理	// 仿真环境中不考虑通讯判断
	uint32_t seq = control_msg->header.seq;
	if (control_seq_ + 1 != seq)
	{
		error_communication_count_control_++;
		if (error_communication_count_control_ > MAX_COM_ERROR_COUNT)
		{
			error_communication_count_control_ = 0;
			ROS_ERROR("Decision--Control:Control sequence mismatch.");
		}
	}
	control_seq_ = seq;
*/
	// 保留有效的控制消息
	control_mutex_.lock();
	control_info_ = ControlInfo(control_msg->fault_status, control_msg->running_status, control_msg->offset_y,
                                    control_msg->offset_heading, control_msg->offset_speed);
	control_mutex_.unlock();


	// 检查控制状态是否合理
	if(control_msg->fault_status == 3)	// 存在严重故障
	{
		ROS_ERROR("Decision--Control:Control fault status is severity level .");
		return;
	}

	Ready_control_ = 1;

}


void Decision::refLineCallback(const map_msgs::REFPointArray::ConstPtr &ref_line_msg)
{
	// 处理收到的地图参考线信息

	ref_line_.clearReferencePoints();	// 参考线初始化，清除上面所有参考点信息
	Ready_ref_line_ = 0;

	ROS_INFO("Decision--RefLine: the size of ref_line is %d",ref_line_msg->REF_line_INFO.size());


	std::vector<ReferenceLinePoint> ref_points;
	for (uint32_t i = 0; i < ref_line_msg->REF_line_INFO.size(); ++i)
	{
		// 处理车道宽信息
		std::vector< LaneRange > lane_ranges;
		for (uint32_t j = 0; j < ref_line_msg->REF_line_INFO[i].lane_ranges.size(); ++j)
		{
			LaneRange lane_range;
			lane_range.left_boundary_  = ref_line_msg->REF_line_INFO[i].lane_ranges[j].left_boundary;
			lane_range.right_boundary_ = ref_line_msg->REF_line_INFO[i].lane_ranges[j].right_boundary;
			lane_ranges.emplace_back(lane_range);
		}

		// 处理参考点信息，注意角度的转变
		ReferenceLinePoint ref_point(ref_line_msg->REF_line_INFO[i].rs, ref_line_msg->REF_line_INFO[i].rx, ref_line_msg->REF_line_INFO[i].ry,
		                      				ThetaTransform(angle2Radian(ref_line_msg->REF_line_INFO[i].rtheta)), ref_line_msg->REF_line_INFO[i].rkappa,
		                      				ref_line_msg->REF_line_INFO[i].rdkappa, ref_line_msg->REF_line_INFO[i].max_speed, lane_ranges);
		// 整理参考点信息
		ref_points.emplace_back(ref_point);

		// 输出参考线信息
		ROS_INFO("Decision--RefLine:s=%f,x=%f,y=%f,theta=%f,kappa=%f,v=%f",ref_line_msg->REF_line_INFO[i].rs,ref_line_msg->REF_line_INFO[i].rx, ref_line_msg->REF_line_INFO[i].ry,ThetaTransform(angle2Radian(ref_line_msg->REF_line_INFO[i].rtheta)),ref_line_msg->REF_line_INFO[i].rkappa,ref_line_msg->REF_line_INFO[i].max_speed);
	}

	// 保存参考线信息
	ref_line_mutex_.lock();
	ref_line_.setReferenceLinePoints(ref_points);
	ref_line_mutex_.unlock();

	Ready_ref_line_ = 1;

	ROS_INFO("Decision--RefLine: the ref_line is READY .");

	// 生成速度地图
	speed_map_ = SpeedMap(ref_line_);

	ROS_INFO("Decision--RefLine: SpeedMap is OK .");
}


void Decision::perceptionCallback(const perception_msgs::FusionDataInfo::ConstPtr &perception_msg)
{
	// 函数功能：处理收到的障碍物信息	

	Ready_perception_ = 0;
/*
	//判断序列号是否合理	// 仿真环境中不考虑通讯判断
	uint32_t seq = perception_msg->header.seq;
	if (perception_seq_ + 1 != seq)
	{
		error_communication_count_perception_++;
		if (error_communication_count_perception_ > MAX_COM_ERROR_COUNT)
		{
			error_communication_count_perception_ = 0;
			ROS_ERROR("Decision--Perception:Perception sequence mismatch.");
		}
	}
	else
	{
		error_communication_count_perception_ = 0;
	}
	perception_seq_ = seq;
*/

	if(Ready_ref_line_ == 0 )
	{
		ROS_WARN("Decision--Perception: the ref line is not ready. ");
		return;
	}

	// 保存障碍物信息
	uint32_t i = 0;
	std::vector<Obstacle> obs_vec;
	for (common_msgs::ObstacleInfo obs_info : perception_msg->obstacles)
	{
		i = i + 1;
		Obstacle obstacle(obs_info.id, 
											Eigen::Vector2d(obs_info.peak[0].x, obs_info.peak[0].y),
											Eigen::Vector2d(obs_info.peak[1].x, obs_info.peak[1].y),
											Eigen::Vector2d(obs_info.peak[2].x, obs_info.peak[2].y),
											Eigen::Vector2d(obs_info.peak[3].x, obs_info.peak[3].y),
											obs_info.velocity, ThetaTransform(angle2Radian(obs_info.theta)));

		if(obstacle.init(ref_line_))	// 障碍物初始化后，加入到环境中去
		{
			obs_vec.emplace_back(obstacle);
		}
		else
		{
			ROS_WARN("Decision--Perception:Perception Obstacle init failed.");
			continue;		
		}
	}

	ROS_INFO("Decision--Perception: the num of Obstacle is %d",i);

	// 记录障碍物信息
	perception_mutex_.lock();
	envi_info_.setObstacleVec(obs_vec);
	perception_mutex_.unlock();

	Ready_perception_ = 1;
}



void Decision::fixedLidarInfoCallback (const location_sensor_msgs::FixedLidarInfo::ConstPtr& fixed_lidar_msg)
{
	// 函数功能：处理收到的场端信息

	// 需要考虑信息校验的问题，此处只是简单的根据位置来进行判断，是否触发场端精确停车
	accurate_stop_flag_ = 0;		// 不触发精确停车

/*
	// 当车辆接近与目标点时，才信赖场端数据，否则，则不信任，防止场端进行误触发
	if(fabs(planning_end_point_frenet_.getS() - center_point_frenet_.getS() >= 10))
	{
		return;
	}
*/

	if(fabs(fixed_lidar_msg->agv_distance.y) > DISTANCE_FIXED_LIDAR_VALID)	// 场端提供的目标相对位置大于3m，则不触发精确停车
	{
		return;	
	}

	accurate_stop_flag_ = 1;
	target_dis_relative_x_ = fixed_lidar_msg->agv_distance.x;
	target_dis_relative_y_ = fixed_lidar_msg->agv_distance.y;
	target_dis_relative_z_ = fixed_lidar_msg->agv_distance.z;

}



void Decision::PublishEstopMsg()	// 下发急停命令
{
	plan_msgs::DecisionInfo decision_info;
	decision_info.CMD_estop       = 1;	// 车辆急停
	decision_info.CMD_gear        = 0;
	decision_info.CMD_hand_brake  = 0;
	decision_info.CMD_lift        = 0;
	decision_info.path_plan_valid = 0;	// 轨迹无效
	decision_info.move_permit     = Ready_car_all_;	// 允许动车
	decision_info.path_mode       = 1;	// lattice
			
	decision_pub_.publish(decision_info);

	// 清除指令信息 20190918
	VMS_Cmd_list_.clear();
	Estop_flag_ = 0;
	ROS_INFO("Decision--Task: EStop. Clear the cmd list. ");

}


void Decision::PublishFixedLidarStopMsg()	// 下发场端精确停车命令
{

	// 触发场端精确停车，此时不进行轨迹规划，直接把场端探测到的相对位置信息发给控制节点就可以

	plan_msgs::DecisionInfo decision_info;

	decision_info.relative_position.x = target_dis_relative_x_;
	decision_info.relative_position.y = target_dis_relative_y_;
	decision_info.relative_position.z = target_dis_relative_z_;

	decision_info.CMD_estop       = 0;
	decision_info.CMD_gear        = 0;
	decision_info.CMD_hand_brake  = 0;
	decision_info.CMD_lift        = 0;
	decision_info.path_plan_valid = 1;	// 轨迹有效
	decision_info.move_permit     = Ready_car_all_;	// 允许动车
	decision_info.path_mode       = 4;	// 场端定位
	decision_pub_.publish(decision_info);

	ROS_INFO("Decision--Task:Fixed lidar accurate stop, dx = %f,dy = %f,dz = %f.",target_dis_relative_x_,target_dis_relative_y_,target_dis_relative_z_);
}


void Decision::PublishBestTrajectory()	// 下发最佳轨迹
{
	plan_msgs::DecisionInfo decision_info;
	// 挨个写入轨迹信息
	for (uint32_t i = 0; i < best_trajectory_.getTrajectorySize(); ++i)
	{
		CartesianPoint path_point = best_trajectory_.getCartesianPointByIndex(i);

		common_msgs::PathPoint path_data;
		path_data.x = path_point.getX();
		path_data.y = path_point.getY();
		path_data.theta = radian2Angle(ThetaTransform(path_point.getTheta()));
		path_data.v = path_point.getVel();

		decision_info.path_data_REF.emplace_back(path_data);
	}

	decision_info.CMD_estop       = 0;
	decision_info.CMD_gear        = 0;
	decision_info.CMD_hand_brake  = 0;
	decision_info.CMD_lift        = 0;
	decision_info.path_plan_valid = 1;	// 轨迹有效
	decision_info.move_permit     = Ready_car_all_;	// 允许动车
	decision_info.path_mode       = 1;	// lattice 
	decision_pub_.publish(decision_info);
}


bool Decision::isObstacleSafe()	// 判断障碍物是否危险
{

	//计算当前车辆所在车道的L值范围，用于对障碍物信息进行筛选
	ReferenceLinePoint ref_point = ref_line_.getReferenceLinePointByS(center_point_frenet_.getS());	

	std::vector< LaneRange > lane_ranges_car_in = ref_point.getLaneRanges();

	double LaneRange_CarIn_min = MAX_NUM;
	double LaneRange_CarIn_max = -MAX_NUM;
	
	// 修改车道判断逻辑，将原来在哪一条车道上的判断逻辑修改为车辆占据那些车道范围的判断逻辑，若查询失败，则采用车道宽进行默认补充计算 20190918
	ROS_INFO("Decision--Safe: car_s_min_ =  %f ,car_s_max_ =  %f",car_s_min_,car_s_max_);
	ROS_INFO("Decision--Safe: car_l_min_ =  %f ,car_l_max_ =  %f",car_l_min_,car_l_max_);
	ROS_INFO("Decision--Safe: the num of lanes is = %d ",lane_ranges_car_in.size());

	bool left_range_flag = 0;
	bool right_range_flag = 0;
	for(uint32_t i = 0; i < lane_ranges_car_in.size(); ++i)
	{
		if (lane_ranges_car_in[i].left_boundary_  <= car_l_min_ && car_l_min_ <= lane_ranges_car_in[i].right_boundary_)
		{
			LaneRange_CarIn_min = lane_ranges_car_in[i].left_boundary_;
			left_range_flag = 1;
		}
		if (lane_ranges_car_in[i].left_boundary_  <= car_l_max_ && car_l_max_ <= lane_ranges_car_in[i].right_boundary_)
		{
			LaneRange_CarIn_max = lane_ranges_car_in[i].right_boundary_;
			right_range_flag = 1;
		}
	}
	// 如果车辆宽度没有计算出来，则强制修改车辆占据的车道宽度
	if((left_range_flag == 0)&&(right_range_flag == 0))
	{
		ROS_WARN("Decision--Safe:Calculate lane rang of car failed, range in use default value");
		LaneRange_CarIn_min = center_point_frenet_.getL() - 2*CAR_WIDTH;
		LaneRange_CarIn_max = center_point_frenet_.getL() + 2*CAR_WIDTH; 
	}
	else if(left_range_flag == 0)
	{
		ROS_WARN("Decision--Safe:Calculate left lane rang of car failed, left range in use default value");
		LaneRange_CarIn_min = LaneRange_CarIn_max - 2*CAR_WIDTH;
	}
	else if(right_range_flag == 0)
	{
		ROS_WARN("Decision--Safe:Calculate right lane rang of car failed, right range in use default value");
		LaneRange_CarIn_max = LaneRange_CarIn_min + 2*CAR_WIDTH;
	}
	ROS_INFO("Decision--Safe: Current lane range left is  = %f , right is  = %f",LaneRange_CarIn_min,LaneRange_CarIn_max);


	// 计算障碍物和车辆前方的最小距离
	double obs_s_min = MAX_NUM;

	for (Obstacle &obs : envi_info_.getObstacleVec())	// 依次遍历障碍物在frenet坐标系下的最小s值
	{		
		// 忽略在车辆后方的障碍物
		if(obs.getSmax() < car_s_min_)
		{
			ROS_INFO("Decision--Safe: Obstacle %d behind the vehicle, obs_s_max = %f ",obs.getId(),obs.getSmax());
			continue;
		}

		// 忽略不在当前车道范围的障碍物
		if((obs.getLmax() <= LaneRange_CarIn_min) || (obs.getLmin() >= LaneRange_CarIn_max))
		{
			ROS_INFO("Decision--Safe: Obstacle %d out of the lane range,obs_l_max = %f,obs_l_min = %f ",obs.getId(),obs.getLmax(),obs.getLmin());
			continue;
		}

		// 忽略和车辆横向距离太大的障碍物
		if((obs.getLmax() <= car_l_min_ - SAFE_DISTANCE_LAT)||(obs.getLmin() >= car_l_max_ + SAFE_DISTANCE_LAT))
		{
			ROS_INFO("Decision--Safe: the lateral gap between Obstacle %d and the vehicle is too big.obs_l_max = %f ,obs_l_min = %f.",obs.getId(),obs.getLmax(),obs.getLmin());
			continue;
		}

		// 更新车辆和障碍物的最小距离
		if(obs.getSmin() < obs_s_min)
		{
			obs_s_min = obs.getSmin();
			ROS_INFO("Decision--Safe:Obstacle %d in current lane,obs_l_max = %f,obs_l_min = %f,obs_s_max = %f,obs_s_min = %f",obs.getId(),obs.getLmax(),obs.getLmin(),obs.getSmax(),obs.getSmin());
		}
	}

	min_dis_obs_front_car_ = obs_s_min - car_s_max_;



	// 若车辆在换道模式中，则清除车辆和障碍物的最小距离，避免换道结束返回原车道时，误触发停车轨迹，导致规划失败 20190930
	if(cmd_type_ != LANE_CHANGE_FORBID)
	{
		obs_stop_flag_ = 0;
		min_dis_obs_front_car_ = MAX_NUM;
	}


	ROS_INFO("Decision--Safe:front obstacle min distance  = %f ",min_dis_obs_front_car_);

	// 若最小障碍物距离车辆小于纵向安全距离，则认为障碍物危险（5m）
	if(min_dis_obs_front_car_ < SAFE_DISTANCE_LON)
	{
		ROS_WARN("Decision--Safe:Front obstacle is too close . Dangerous !!! ");
		Estop_flag_ = 1;
		return false;
	}

	// 判断是否触发障碍物停车规划（20m）
	if(min_dis_obs_front_car_ <= DISTANCE_OBSTRACLE_STOP) 
	{
		obs_stop_flag_ = 1;
		ROS_INFO("Decision--Safe:Front obstacle is too close,less than 20m .Start stopping planning.");
	}
	else
	{
		// 增加延时处理，连续5个周期没有问题，则认为是安全	 20190916	
		obs_disappeared_count ++;
		if(obs_disappeared_count > MAX_OBS_DISAPPEAREND_COUNT)
		{
			ROS_INFO("Decision--Safe:All obstacle are safety.");
			obs_disappeared_count = 0;
			obs_stop_flag_ = 0;
		}
	}

	// 触发停车规划时，更新最近障碍物的最小s值
	if((obs_stop_flag_pre_ == 0)&&(obs_stop_flag_ == 1))
	{
		s_min_obs_front_car_ = obs_s_min;
	}

	obs_stop_flag_pre_ = obs_stop_flag_;

	return true;

}



bool Decision::getValidCMD()	// 获取当前有效的任务指令 
{
	// 指令序列管理，删除已完成的指令
	if(VMS_Cmd_list_.empty()) //当前无指令，等待指令
	{
		ROS_INFO("Decision--CMD: No VMS CMD or All CMD has been done. ");
		return false;
	}
	else
	{
		// 判断当前指令是否已经完成，若完成，则删除当前指令	
		if(CMD_finish_flag_ == 1)
		{

			TraceData();	// 数据保存 20191009

			std::vector<VMS_Cmd>::iterator i = VMS_Cmd_list_.begin();
			VMS_Cmd_list_.erase(i); // 删除第一条指令
			ROS_INFO("Decision--CMD: Current CMD has been done. Task is Finished. ");

			// 删除历史轨迹信息
			best_trajectory_.clearTrajectoryPoint();
		}
		// 删除指令后，没有指令等待，则指令完成
		if(VMS_Cmd_list_.empty()) 
		{
			return false;
		}
	}

	CMD_finish_flag_ = 0;

	// 将目标指令信息坐标转换到frenet坐标系下
	CartesianPoint cmd_point_temp;
	cmd_point_temp.setX(VMS_Cmd_list_.front().getX());
	cmd_point_temp.setY(VMS_Cmd_list_.front().getY());
	cmd_point_temp.setTheta(VMS_Cmd_list_.front().getHeading());
	cmd_point_temp.setVel(0.0);
	cmd_point_temp.setAcc(0.0);
	cmd_point_temp.setKappa(0.0);
	
	ROS_INFO("Decision--CMD: x = %f, y =%f,heading = %f. ",cmd_point_temp.getX(),cmd_point_temp.getY(),cmd_point_temp.getTheta());

	FrenetPoint cmd_point_frenet_temp;
	if (!cartesianToFrenet(cmd_point_temp, ref_line_, cmd_point_frenet_temp))
	{
		ROS_WARN("Decision--CMD: VMS target cartesian point transform to Frenet failed.");
		return false;
	}

	if(cmd_point_frenet_temp.getS() + 0.5*CAR_LENGTH > ref_line_.getReferenceLineMaxS())
	{
		ROS_WARN("Decision--CMD: VMS target frenet point is invalid , out of the refline.");
		return false;
	}

	ROS_INFO("Decision--CMD: VMS target Frenet point S = %f , L = %f ",cmd_point_frenet_temp.getS(),cmd_point_frenet_temp.getL());
	
	// 更新指令终点
	planning_end_point_frenet_ = cmd_point_frenet_temp;		


	// 计算轨迹规划起点
	if(control_info_.getOffsetY() >= OFFSET_Y_THRESHOLD || 
		 control_info_.getOffsetHeading() >= OFFSET_HEADING_THRESHOLD ||
		 control_info_.getOffsetSpeed() >= OFFSET_SPEED_THRESHOLD)	
	{
		// 偏差过大（横向偏差超过0.5m，角度偏差超过10°，速度偏差超过0.5m/s），规划点为车辆中心点
		planning_start_point_frenet_ = center_point_frenet_;

		ROS_INFO("Decision--CMD: control offset is too big, use the center point as the start point. offsetY = %f,offsetHeading = %f,offsetSpeed = %f",control_info_.getOffsetY(),control_info_.getOffsetHeading(),control_info_.getOffsetSpeed());

	}
	else
	{
		// 偏差不大，若之前存在轨迹信息，则用上一轨迹上相对应的映射点，作为轨迹规划的起点，若不存在，则把当前车辆位置作为规划起点
		if (best_trajectory_.getTrajectorySize() == 0)
		{
			planning_start_point_frenet_ = center_point_frenet_;
		}
		else
		{
			uint32_t dis_min_index = best_trajectory_.getNearestCartesianPointIndex(center_point_cartesian_);
			planning_start_point_frenet_ = best_trajectory_.getFrenetPointByIndex(dis_min_index);
		}
	}

	// 计算轨迹规划终点：找到最早的一条未完成的指令的坐标信息，作为轨迹规划的终点
	CartesianPoint target_point;
	target_point.setX(VMS_Cmd_list_.front().getX());
	target_point.setY(VMS_Cmd_list_.front().getY());
	target_point.setTheta(VMS_Cmd_list_.front().getHeading());
	target_point.setVel(0);
	target_point.setAcc(0);
	target_point.setKappa(0.0);

	// 将坐标转换到frenet坐标系下
	if (!cartesianToFrenet(target_point, ref_line_, planning_end_point_frenet_))
	{
		ROS_ERROR("Decision--CMD: Target cartesian point transform to Frenet point failed.");
		return false;
	}

	// 判断任务是否完成
	if(center_point_frenet_.getS() >= planning_end_point_frenet_.getS())
	{
		CMD_finish_flag_ = 1;
		return false;
	}
	else if(planning_end_point_frenet_.getS() - center_point_frenet_.getS() <= STOP_DISTANCE)	//增加车辆距离目标点小于5cm时，判定车辆完成当前任务,20191010
	{
		CMD_finish_flag_ = 1;
		return false;
	}

	return true;
}



void Decision::TraceData()	// 数据保存
{

	TraceDataCount ++;

	// 需要保存的决策变量
	std::string decision_buf = ",";
 	decision_buf = decision_buf + 
								 std::to_string(center_point_cartesian_.getX()) + "," + 
							   std::to_string(center_point_cartesian_.getY()) + "," + 
 							   std::to_string(center_point_cartesian_.getTheta()) + "," + 
							   std::to_string(center_point_cartesian_.getVel()) + "," + 
 							   std::to_string(center_point_cartesian_.getAcc()) + "," + 
							   std::to_string(center_point_cartesian_.getKappa()) + "," + 
 							   std::to_string(center_point_frenet_.getS()) + "," + 
							   std::to_string(center_point_frenet_.getdS()) + "," + 
 							   std::to_string(center_point_frenet_.getddS()) + "," + 
							   std::to_string(center_point_frenet_.getL()) + "," + 
 							   std::to_string(center_point_frenet_.getdL()) + "," + 
							   std::to_string(center_point_frenet_.getddL()) + "," + 
 							   std::to_string(car_s_max_) + "," + 
							   std::to_string(car_s_min_) + "," + 
 							   std::to_string(car_l_max_) + "," + 
							   std::to_string(car_l_min_) + "," + 
 							   std::to_string(Ready_vcu_) + "," + 
							   std::to_string(Ready_location_) + "," + 
 							   std::to_string(Ready_control_) + "," + 
							   std::to_string(Ready_perception_) + "," + 
 							   std::to_string(Ready_ref_line_) + "," + 
							   std::to_string(accurate_stop_flag_) + "," + 
 							   std::to_string(obs_stop_flag_) + "," + 
							   std::to_string(theta_modify_flag_) + "," + 
 							   std::to_string(CMD_finish_flag_) + "," + 
							   std::to_string(planning_start_point_frenet_.getS()) + "," + 
 							   std::to_string(planning_start_point_frenet_.getdS()) + "," + 
							   std::to_string(planning_start_point_frenet_.getddS()) + "," + 
 							   std::to_string(planning_start_point_frenet_.getL()) + "," + 
							   std::to_string(planning_start_point_frenet_.getdL()) + "," + 
 							   std::to_string(planning_start_point_frenet_.getddL()) + "," + 
							   std::to_string(planning_end_point_frenet_.getS()) + "," + 
 							   std::to_string(planning_end_point_frenet_.getdS()) + "," + 
							   std::to_string(planning_end_point_frenet_.getddS()) + "," + 
 							   std::to_string(planning_end_point_frenet_.getL()) + "," + 
							   std::to_string(planning_end_point_frenet_.getdL()) + "," + 
 							   std::to_string(planning_end_point_frenet_.getddL()) + "," + 
 							   std::to_string(target_dis_relative_x_) + "," + 
							   std::to_string(target_dis_relative_y_) + "," + 
 							   std::to_string(target_dis_relative_z_) + "," + 
							   std::to_string(cmd_type_) + "," + 
 							   std::to_string(min_dis_obs_front_car_) + "," + 
 							   VMS_Cmd_list_.front().getID() + "," + 
							   std::to_string(VMS_Cmd_list_.front().getX()) + "," + 
 							   std::to_string(VMS_Cmd_list_.front().getY()) + "," + 
							   std::to_string(VMS_Cmd_list_.front().getHeading()) + "," + 
 							   std::to_string(vcu_info_.getControlMode()) + "," + 
							   std::to_string(vcu_info_.getFaultStatus()) + "," + 
 							   std::to_string(vcu_info_.getSOC()) + "," + 
							   std::to_string(vcu_info_.getStartStatus()) + "," + 
 							   std::to_string(vcu_info_.getEstopStatus()) + "," + 
							   std::to_string(vcu_info_.getHeatbeat()) + "," + 
 							   std::to_string(control_info_.getFaultStatus()) + "," + 
							   std::to_string(control_info_.getRunningStatus()) + "," + 
 							   std::to_string(control_info_.getOffsetY()) + "," + 
							   std::to_string(control_info_.getOffsetHeading()) + "," + 
 							   std::to_string(control_info_.getOffsetSpeed()) + "," + 
								 std::to_string(lattice_planner_.getStopFlag()) + "," + 
								 std::to_string(lattice_planner_.getPlanningEndPoint().getS()) + "," + 
								 std::to_string(lattice_planner_.getPlanningEndPoint().getdS()) + "," + 
								 std::to_string(lattice_planner_.getPlanningEndPoint().getddS()) + "," + 
								 std::to_string(lattice_planner_.getPlanningEndPoint().getL()) + "," + 
								 std::to_string(lattice_planner_.getPlanningEndPoint().getdL()) + "," + 
								 std::to_string(lattice_planner_.getPlanningEndPoint().getddL());

	decision_data_vec_.push_back(decision_buf);

	// 需要保存的障碍物信息
	std::string perception_buf = ",";
	uint32_t obs_num = 0;
	for (Obstacle &obs : envi_info_.getObstacleVec())	// 依次遍历障碍物在frenet坐标系下的最小s值
	{		
		perception_buf = perception_buf + 
										 std::to_string(obs.getId()) + "," + 
										 std::to_string(obs.getHeading()) + "," + 
										 std::to_string(obs.getSmax()) + "," + 
										 std::to_string(obs.getSmin()) + "," + 
										 std::to_string(obs.getLmax()) + "," + 
										 std::to_string(obs.getLmin()) + ",";

		for(Eigen::Vector2d corner : obs.getAllCorners())	
		{
			perception_buf = perception_buf + std::to_string(corner(0)) + "," + std::to_string(corner(1)) + ",";
		}

		obs_num ++ ;
	}
	perception_buf = perception_buf + std::to_string(obs_num);

	obstacle_data_vec_.push_back(perception_buf);
	

	// 需要保存的轨迹信息
	std::string trajectory_buf = ",";
	for (uint32_t i = 0; i < best_trajectory_.getTrajectorySize(); ++i)
	{
		CartesianPoint path_point_Cartesian = best_trajectory_.getCartesianPointByIndex(i);
		FrenetPoint path_point_Frenet = best_trajectory_.getFrenetPointByIndex(i);

		trajectory_buf = trajectory_buf + 
										 std::to_string(path_point_Cartesian.getX()) + "," + 
										 std::to_string(path_point_Cartesian.getY()) + "," + 
										 std::to_string(radian2Angle(ThetaTransform(path_point_Cartesian.getTheta()))) + "," + 
										 std::to_string(path_point_Cartesian.getVel()) + "," + 
										 std::to_string(path_point_Frenet.getS()) + "," + 
										 std::to_string(path_point_Frenet.getL()) + "," + 
										 std::to_string(path_point_Frenet.getdS()) + ",";
	}
	trajectory_buf = trajectory_buf + std::to_string(best_trajectory_.getTrajectorySize());

	trajectory_data_vec_.push_back(trajectory_buf);




	if((TraceDataCount == MAX_TRACE_NUM) || (CMD_finish_flag_ == true) || (Estop_flag_ == 1))
	{

		ROS_INFO("Decision--Trace: Start Save trace file .");

		// 获取当前时间
		time_t t = time(NULL);
		tm *local = localtime(&t);

		std::string current_time = std::to_string(local->tm_year+1900)+"_"+std::to_string(local->tm_mon+1)+"_"+std::to_string(local->tm_mday)+"_"+std::to_string(local->tm_hour)+"_"+std::to_string(local->tm_min)+"_"+std::to_string(local->tm_sec);


		std::string decision_data = "TraceData/Decision/DecisionData_" + current_time + ".txt";	// 决策Trace文件名	 
		std::ofstream decision_data_file_(decision_data); 	// 保存决策信息	
		if(decision_data_file_.is_open())
		{
			for(uint32_t i = 0; i < decision_data_vec_.size();i++ )
			{
				decision_data_file_ << decision_data_vec_[i] << std::endl;
			}
			decision_data_file_.close();
			
			ROS_INFO("Decision--Trace: Save Decision trace file OK .");
		}
		else
		{
			ROS_WARN("Decision--Trace: Create Decision trace file failed.");
		}


		std::string perception_data = "TraceData/Perception/PerceptionData_" + current_time + ".txt";// 感知Trace文件名
		std::ofstream perception_data_file_(perception_data);// 保存感知信息	 
		if(perception_data_file_.is_open())
		{
			for(uint32_t i = 0; i < obstacle_data_vec_.size();i++ )
			{
				perception_data_file_ << obstacle_data_vec_[i] << std::endl;
			}
			perception_data_file_.close();

			ROS_INFO("Decision--Trace: Save Perception trace file OK .");
		}
		else
		{
			ROS_WARN("Decision--Trace: Create Perception trace file failed.");
		}

		std::string trajectory_data = "TraceData/Trajectory/TrajectoryData_" + current_time + ".txt";// 轨迹Trace文件名
		std::ofstream trajectory_data_file_(trajectory_data);// 保存轨迹信息	 
		if(trajectory_data_file_.is_open())
		{
			for(uint32_t i = 0; i < trajectory_data_vec_.size();i++ )
			{
				trajectory_data_file_ << trajectory_data_vec_[i] << std::endl;
			}
			trajectory_data_file_.close();

			ROS_INFO("Decision--Trace: Save Trajectory trace file OK .");
		}
		else
		{
			ROS_WARN("Decision--Trace: Create Trajectory trace file failed.");
		}

		TraceDataCount = 0;	// 计数器清0
		// 清空数据保存容器 20191010
		decision_data_vec_.clear();
		obstacle_data_vec_.clear();
		trajectory_data_vec_.clear();
	}


	// 遍历文件夹中的所有文件,删除文件夹中多余的文件 20191012

  DIR *dir=NULL;
  struct dirent* pDir=NULL;
	std::string Fliname_Oldest;

	// 删除决策文件中多余的文件
	int FilesNum = 0;
  dir = opendir("TraceData/Decision");
  if(dir == NULL)
	{
		 ROS_WARN("Decision--Trace: Can't open Decision dir");
		return;
	} 	
	while(true)
	{
		pDir = readdir(dir);
		if((pDir == NULL))
		{
			break;
		}
		if (pDir->d_type == DT_REG)
		{
			// 找出文件名最小的一个文件，即生成时间最早的一个文件
			FilesNum ++;
			if(FilesNum == 1)
			{
				Fliname_Oldest = pDir ->d_name;
			}
			else
			{
				if(Fliname_Oldest > pDir ->d_name)
				{
					Fliname_Oldest = pDir ->d_name;
				}
			}
		}
	}
	closedir(dir);
	if(FilesNum > MAX_FILE_NUM)
	{
		std::string deletfn = "TraceData/Decision/" + Fliname_Oldest;
		if(-1 == remove(deletfn.c_str()))
		{
			ROS_WARN("Decision--Trace: delete Decision trace file failed.");
			return;
		}
	}  

	// 删除感知文件中多余的文件
	FilesNum = 0;
  dir = opendir("TraceData/Perception");
  if(dir == NULL)
	{
		 ROS_WARN("Decision--Trace: Can't open Perception dir");
		return;
	} 	
	while(true)
	{
		pDir = readdir(dir);
		if((pDir == NULL))
		{
			break;
		}
		if (pDir->d_type == DT_REG)
		{
			// 找出文件名最小的一个文件，即生成时间最早的一个文件
			FilesNum ++;
			if(FilesNum == 1)
			{
				Fliname_Oldest = pDir ->d_name;
			}
			else
			{
				if(Fliname_Oldest > pDir ->d_name)
				{
					Fliname_Oldest = pDir ->d_name;
				}
			}
		}
	}
	closedir(dir);
	if(FilesNum > MAX_FILE_NUM)
	{
		std::string deletfn = "TraceData/Perception/" + Fliname_Oldest;
		if(-1 == remove(deletfn.c_str()))
		{
			ROS_WARN("Decision--Trace: delete Perception trace file failed.");
			return;
		}
	} 


	// 删除轨迹文件中多余的文件
	FilesNum = 0;
  dir = opendir("TraceData/Trajectory");
  if(dir == NULL)
	{
		 ROS_WARN("Decision--Trace: Can't open Trajectory dir");
		return;
	} 	
	while(true)
	{
		pDir = readdir(dir);
		if((pDir == NULL))
		{
			break;
		}
		if (pDir->d_type == DT_REG)
		{
			// 找出文件名最小的一个文件，即生成时间最早的一个文件
			FilesNum ++;
			if(FilesNum == 1)
			{
				Fliname_Oldest = pDir ->d_name;
			}
			else
			{
				if(Fliname_Oldest > pDir ->d_name)
				{
					Fliname_Oldest = pDir ->d_name;
				}
			}
		}
	}
	closedir(dir);
	if(FilesNum > MAX_FILE_NUM)
	{
		std::string deletfn = "TraceData/Trajectory/" + Fliname_Oldest;
		if(-1 == remove(deletfn.c_str()))
		{
			ROS_WARN("Decision--Trace: delete Trajectory trace file failed.");
			return;
		}
	} 
}



void Decision::task_management()
{
	// 场端辅助定位规划
	// planning轨迹生成
	// rtk支持

	ros::Rate loop_rate(10);	// 程序执行的周期为10HZ


	uint32_t index_debug = 0;

	while(n_.ok())
	{

		// 仅供调试用
		index_debug ++;
		if(index_debug == 2147483647)
		{
			index_debug = 0;
		}
		ROS_INFO("RunCycle: index_debug = %d.",index_debug);

		
		// 判断参考线是否存在，若不存在，则直接返回
		if(Ready_ref_line_ ==0 )
		{
			ROS_WARN("Decision--Task: not get the ref line. ");
			loop_rate.sleep();
			continue;	
		}
		// 判断位置信息是否有效，若不有效，则直接返回
		if(Ready_location_ ==0 )
		{
			ROS_WARN("Decision--Task: not get the valid location. ");
			loop_rate.sleep();
			continue;	
		}

/*
		// 仿真环境中不考虑整车状态
		// 若车辆不在自动驾驶模式，则车辆不执行pnc指令，清空指令序列，直接跳出任务逻辑
		if(vcu_info_.getControlMode() != 2)
		{
			ROS_ERROR("Decision--Task:agv control mode is not auto.");
			VMS_Cmd_list_.clear();
			loop_rate.sleep();
			continue;
		}
		// 判断车辆整车状态，用于决定是否允许动车
		Ready_car_all_ = Ready_vcu_ * Ready_location_ * Ready_control_ * Ready_perception_ * Ready_ref_line_ ;

		if(Ready_car_all_ == 0)
		{
			ROS_ERROR("Decision--Task:agv is not ready.");
			PublishEstopMsg();// 下发急停命令
			loop_rate.sleep();
			continue;			
		}
*/		



		// 当前无可执行指令，等待指令
		if(getValidCMD() == 0)
		{
			loop_rate.sleep();
			continue;
		}

		TraceData();	// 数据保存 20191009

		// 获取换道指令信息 20190920
		lane_change_ = LaneChange(center_point_cartesian_,planning_end_point_frenet_,ref_line_,envi_info_);
		cmd_type_ = lane_change_.getLaneChangeCmd();
		if(cmd_type_ != LANE_CHANGE_FORBID)
		{
			ROS_INFO("Decision--Task: car is changing the lane ... lane change type = %d",cmd_type_);	

			theta_modify_flag_ = true;	// 为了支持车辆斜行功能，增加角度修改标志位，20191009
		}
		else if((theta_modify_flag_ == true) && (fabs(center_point_frenet_.getL()) <= 0.1))
		{
			theta_modify_flag_ = false;
		}

//		cmd_type_ = LANE_CHANGE_FORBID;	//禁止绕障

		// 对障碍物和车辆的位置判断，若距离太近，则直接触发急停
		if(isObstacleSafe() == 0)
//		if((isObstacleSafe() == 0)&&(cmd_type_ == LANE_CHANGE_FORBID)) //　若车辆在换道过程中，则不增加该条件判断　20190926
		{
			PublishEstopMsg();	// 下发急停命令
			loop_rate.sleep();
			continue;
		}

		// 前方障碍物距离车辆距离为20m，触发障碍物停车规划	
		//if((obs_stop_flag_ == 1)&&(cmd_type_ == LANE_CHANGE_FORBID))
		if(obs_stop_flag_ == 1)
		{	
			ROS_INFO("Decision--Task: obstacle is too close, car is stopping.");	

			double s0 = center_point_frenet_.getS();
			// 生成停车目标点
			double s_stop = s_min_obs_front_car_  - 0.5*CAR_LENGTH - 1.2*SAFE_DISTANCE_LON;	// 留一点余量，防止在轨迹末端触发急停
			FrenetPoint end_point_obs;
			end_point_obs.setS(s_stop);
			end_point_obs.setdS(0.0);
			end_point_obs.setdS(0.0);
			end_point_obs.setddS(0.0);
			end_point_obs.setL(0.0);
			end_point_obs.setdL(0.0);
			end_point_obs.setddL(0.0);

			// 进行lattice planning规划，停车规划
			lattice_planner_ = LatticePlanner(planning_start_point_frenet_,end_point_obs,ref_line_,envi_info_,speed_map_,center_point_frenet_,cmd_type_,theta_modify_flag_);

			bool planning_OK = lattice_planner_.isPlanningOK();
			if(planning_OK == 0)
			{
				error_planning_count ++;
				if(error_planning_count < MAX_PLANNING_ERROR_COUNT)
				{
					ROS_WARN("Decision--Task:Use last best trajectory as current trajectory ");
					PublishBestTrajectory();	// 下发轨迹信息
				}
				else
				{
					ROS_ERROR("Decision--Task: EStop . obstacle stop planning planning failed over 5 cycle. ");
					PublishEstopMsg();	// 下发急停命令
					error_planning_count = 0;
				}

				loop_rate.sleep();
				continue;
			}
			else
			{
				error_planning_count = 0;
				// 此处，需要考虑连续5个周期保持原轨迹不动
				best_trajectory_ = lattice_planner_.getBestTrajectory();	// 得到生成的轨迹的轨迹信息				
				PublishBestTrajectory();	// 下发轨迹信息
			}

			loop_rate.sleep();
			continue;
		}

		// 判断是否要进行场端精确停车
		if(accurate_stop_flag_ == 1)
		{
			PublishFixedLidarStopMsg();	// 触发场端精确停车，此时不进行轨迹规划，直接把场端探测到的相对位置信息发给控制节点就可以
			loop_rate.sleep();
			continue;
		}

		// 进行任务轨迹规划
		lattice_planner_ = LatticePlanner(planning_start_point_frenet_,planning_end_point_frenet_,ref_line_,envi_info_,speed_map_,center_point_frenet_,cmd_type_,theta_modify_flag_);

		bool planning_OK = lattice_planner_.isPlanningOK();
		if(planning_OK == 0)
		{
			error_planning_count ++;
			if(error_planning_count < MAX_PLANNING_ERROR_COUNT)
			{
				ROS_WARN("Decision--Task:Use last best trajectory as current trajectory ");
				PublishBestTrajectory();	// 下发轨迹信息
			}
			else
			{
				ROS_ERROR("Decision--Task: EStop . the times of lattice planning failed over 5 cycle. ");
				error_planning_count = 0;
				PublishEstopMsg();	// 下发急停命令
			}

			loop_rate.sleep();
			continue;
		}
		else
		{	
			// 此处，需要考虑连续5个周期保持原轨迹不动，往后每个周期向后顺延一个点，并进行碰撞检测
			error_planning_count = 0;
			best_trajectory_ = lattice_planner_.getBestTrajectory();	// 得到生成的轨迹的轨迹信息			
			PublishBestTrajectory();	// 下发轨迹信息
		}

		loop_rate.sleep();
	}

	loop_rate.sleep();
}




}
