#include "utils.h"

namespace pnc
{

LaneChange::LaneChange(CartesianPoint car_center_cartesian,FrenetPoint target_point_frenet,ReferenceLine ref_line,PerceptionInfo envi_info)
:car_center_cartesian_(car_center_cartesian),target_point_frenet_(target_point_frenet),ref_line_(ref_line),envi_info_(envi_info)
{

	int cmd;	// 换道模块生成的换道指令

	// 计算车辆的边界信息
	if(CalCarBoundary() == 0)
	{
		ROS_WARN("LaneChange: Calculate car's boundary failed.");
		return;
	}

	// 计算车辆所在的车道范围
	if(CalLaneRangeCarIn() == 0)	
	{
		ROS_WARN("LaneChange: Calculate lane range failed.");
		return;
	}

	//根据车辆位置将障碍物进行分类
	if(ClassifyObstacles() == 0)
	{
		ROS_WARN("LaneChange: Classify Obstacles failed.");
		cmd = LANE_CHANGE_FORBID;
		return;
	}

	// 计算车辆换道时的前进距离
	double v = car_center_frenet_.getdS();	// 车辆当前速度
	double v_cross = v*0.5;			// 按照车辆转角为pi/6°，横向速度为车速的cos(pi/6)倍速，即为0.5
	double t_lanechange = WidthLane/(v_cross);	// 换道过程需要的时间
	double s_lanechange = v*t_lanechange;				// 换道过程中车辆向前走的距离

	// 计算换道时需要判断的距离
	double s_range = DISTANCE_LANE_CHANGE > s_lanechange ? DISTANCE_LANE_CHANGE : s_lanechange;

	
	if(cmd_type_ == LANE_CHANGE_FORBID)	// 之前不存在换道命令
	{
		// 距离目标点30m禁止换道
		if(target_point_frenet.getS() - car_center_frenet_.getS() <= 1.5*DISTANCE_OBSTRACLE_STOP)
		{
			ROS_INFO("LaneChange: It is close to the target, less than 30m, Forbid Lane Change.");
			cmd = LANE_CHANGE_FORBID;
			return;	
		}

		// 计算和前方障碍物的最小距离
		double mindis_middle = CalMinDisWithFrontObstacles(obstacle_vec_middle_);	
		
		// 最近距离小于25m，进入换道逻辑判断
		if(mindis_middle <= s_range)	
		{
			// 前方车道被占据，不允许换道
			if(isObstacleOccupation(obstacle_vec_middle_,s_range))			
			{
				ROS_INFO("LaneChange: Front Obstacles Occupy . Forbid Lane Change.");
				cmd = LANE_CHANGE_FORBID;
			}
			else if(lane_vec_left_.size() != 0)	// 左侧存在车道
			{
				if(isObstacleOccupation(obstacle_vec_left_middle_,s_range) == 1)		// 左中侧车道被占据
				{
					ROS_INFO("LaneChange: Left Middle Obstacles Occupy.");
					
					if(lane_vec_right_.size() == 0)	// 不存在右侧车道	
					{
						ROS_INFO("LaneChange: NO Right Lane . Forbid Lane Change.");
						cmd = LANE_CHANGE_FORBID;
					}
					else if((isObstacleOccupation(obstacle_vec_right_middle_,s_range) == 0)&&(isObstacleOccupation(obstacle_vec_right_,s_range) == 0))
					{
						cmd = LANE_CHANGE_RIGHT;
						ROS_INFO("LaneChange: Left Middle Forbid, Right PERMITE.");
					}
				}
				else if(isObstacleOccupation(obstacle_vec_left_,s_range))	// 左侧车道被占据
				{
					ROS_INFO("LaneChange: Left Obstacles Occupy .");

					// 不存在右侧车道	
					if(lane_vec_right_.size() == 0)
					{
						ROS_INFO("LaneChange: NO Right Lane . Forbid Right Change.");
						cmd = LANE_CHANGE_FORBID;
					}
					else if((isObstacleOccupation(obstacle_vec_right_middle_,s_range) == 0)&&(isObstacleOccupation(obstacle_vec_right_,s_range) == 0))
					{
						cmd = LANE_CHANGE_RIGHT;
						ROS_INFO("LaneChange: Left Forbid, Right PERMITE.");
					}
				}
				else
				{
					cmd = LANE_CHANGE_LEFT;
					ROS_INFO("LaneChange: LEFT PERMITE.");
				}
			}
			else if(lane_vec_right_.size() != 0)	// 不存在左侧车道，但存在右侧车道
			{
				// 判断右中侧是否允许换道
				if((isObstacleOccupation(obstacle_vec_right_middle_,s_range) == 0)&&(isObstacleOccupation(obstacle_vec_right_,s_range) == 0))
				{
					cmd = LANE_CHANGE_RIGHT;
					ROS_INFO("LaneChange: No Left Lane, Right PERMITE.");
				}
				else
				{
					cmd = LANE_CHANGE_FORBID;
					ROS_INFO("LaneChange: NO Left Lane .And Forbid Right Lane Change.");
				}
			}
			else
			{
				cmd = LANE_CHANGE_FORBID;
				ROS_INFO("LaneChange: NO Left or Right Lane,Forbid Lane Change.");
			}
		}
		else
		{
			cmd = LANE_CHANGE_FORBID;
			ROS_INFO("LaneChange: Not Need Change Lane.");
		}
	}
	else	// 若之前存在换道指令
	{
		cmd = cmd_type_;	// 冻结换道指令，直到换道过程结束
		
		if(cmd == LANE_CHANGE_LEFT)
		{
			if(lane_vec_left_.size() == 0)
			{
				ROS_WARN("LaneChange: Car is Left LaneChange,but has no left lane.");
				cmd = LANE_CHANGE_FORBID;
			}
			else if(car_center_frenet_.getL() <= lane_vec_left_[lane_vec_left_.size()-1].right_boundary_)	// 已经换到了左侧车道	
			//else if(car_l_max_ <= lane_vec_left_[lane_vec_left_.size()-1].right_boundary_)	// 已经换到了左侧车道	
			{				
				if((isObstacleOccupation(obstacle_vec_right_middle_,s_range) == 0) &&
					 (isObstacleOccupation(obstacle_vec_right_,s_range) == 0) && 
					 (isObstacleOccupation(obstacle_vec_middle_,s_range) == 0)	)	
				{
					// 原车道允许换道，则换道回去
					cmd = LANE_CHANGE_FORBID;
					ROS_INFO("LaneChange: Finish Left Lane Change, back to primary lane.");
				}
			}
			else
			{
				cmd = LANE_CHANGE_LEFT;
				ROS_INFO("LaneChange: Left Lane Changing...");
				ROS_INFO("LaneChange: Left Lane boundary left  = %f, right = %f",lane_vec_left_[0].left_boundary_,lane_vec_left_[lane_vec_left_.size()-1].right_boundary_);
			}
		}
		else if(cmd == LANE_CHANGE_RIGHT)
		{
			if(lane_vec_right_.size() == 0)
			{
				ROS_WARN("LaneChange: Car is Right LaneChange,but has no right lane.");
				cmd = LANE_CHANGE_FORBID;
			}
			else if(car_center_frenet_.getL() >= lane_vec_right_[0].left_boundary_)	// 已经换到了右侧车道
			//else if(car_l_min_ >= lane_vec_right_[0].left_boundary_)	// 已经换到了右侧车道
			{			
				if((isObstacleOccupation(obstacle_vec_left_middle_,s_range) == 0) &&
					 (isObstacleOccupation(obstacle_vec_left_,s_range) == 0) && 
					 (isObstacleOccupation(obstacle_vec_middle_,s_range) == 0))	
				//if(isObstacleOccupation(obstacle_vec_middle_,s_range) == 0)	
				{
					// 原车道允许换道，则换道回去
					cmd = LANE_CHANGE_FORBID;
					ROS_INFO("LaneChange: Finish Right Lane Change, back to primary lane.");
				}
			}
			else
			{
				cmd = LANE_CHANGE_RIGHT;
				ROS_INFO("LaneChange: Right Lane Changing...");
				ROS_INFO("LaneChange: Right Lane boundary left  = %f, right = %f",lane_vec_right_[0].left_boundary_,lane_vec_right_[lane_vec_right_.size()-1].right_boundary_);
			}
		}
		else
		{
			cmd = LANE_CHANGE_FORBID;
			ROS_INFO("LaneChange: Default Forbid.");
		}
	}

	cmd_type_ = cmd;
}



bool LaneChange::CalCarBoundary()	// 计算车辆的边界信息
{
	// 计算车辆中心点在cartesian坐标下的坐标信息
	double x_c = car_center_cartesian_.getX();
	double y_c = car_center_cartesian_.getY();
	double theta_radian = car_center_cartesian_.getTheta();
	double v_car = car_center_cartesian_.getVel();
	double a_var = car_center_cartesian_.getAcc();
	double x[4],y[4];
	x[0] = x_c + CAR_LENGTH/2*cos(theta_radian) - CAR_WIDTH/2*sin(theta_radian); 
	y[0] = y_c + CAR_LENGTH/2*sin(theta_radian) + CAR_WIDTH/2*cos(theta_radian);
	x[1] = x_c + CAR_LENGTH/2*cos(theta_radian) + CAR_WIDTH/2*sin(theta_radian); 
	y[1] = y_c + CAR_LENGTH/2*sin(theta_radian) - CAR_WIDTH/2*cos(theta_radian); 
	x[2] = x_c - CAR_LENGTH/2*cos(theta_radian) + CAR_WIDTH/2*sin(theta_radian); 
	y[2] = y_c - CAR_LENGTH/2*sin(theta_radian) - CAR_WIDTH/2*cos(theta_radian); 
	x[3] = x_c - CAR_LENGTH/2*cos(theta_radian) - CAR_WIDTH/2*sin(theta_radian); 
	y[3] = y_c - CAR_LENGTH/2*sin(theta_radian) + CAR_WIDTH/2*cos(theta_radian); 

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
			ROS_WARN("LaneChange: car corner %d position cartesian transform to Frenet failed.",i);
			return false;
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

	return true;
}





bool LaneChange::CalLaneRangeCarIn()	// 计算车辆所在的车道范围
{
	// 将车辆中心点坐标转换到frenet坐标系下
	if (!cartesianToFrenet(car_center_cartesian_, ref_line_, car_center_frenet_))
	{
		return false;
	}

	// 在参考线上找到距离车辆中心最近的参考点
	ReferenceLinePoint ref_point = ref_line_.getReferenceLinePointByS(car_center_frenet_.getS());	

	// 获取当前位置车道宽信息
	std::vector< LaneRange > lane_ranges = ref_point.getLaneRanges();

	// 计算道路的左边界和右边界
	LaneRange_min = lane_ranges[0].left_boundary_;
	LaneRange_max = lane_ranges[lane_ranges.size()-1].right_boundary_;
	ROS_INFO("LaneChange: Road boundary left is  = %f , right is  = %f",LaneRange_min,LaneRange_max);

	//计算当前车辆所在车道的L值范围
	LaneRange_CarIn_min = MAX_NUM;
	LaneRange_CarIn_max = -MAX_NUM;
	bool left_range_flag = 0;
	bool right_range_flag = 0;
	for(uint32_t i = 0; i < lane_ranges.size(); ++i)
	{
		ROS_INFO("LaneChange: Lane %d boundary left is  = %f , right is  = %f",i,lane_ranges[i].left_boundary_,lane_ranges[i].right_boundary_);
		if (lane_ranges[i].left_boundary_  <= car_l_min_ && car_l_min_ <= lane_ranges[i].right_boundary_)
		{
			LaneRange_CarIn_min = lane_ranges[i].left_boundary_;
			left_range_flag = 1;
		}
		if (lane_ranges[i].left_boundary_  <= car_l_max_ && car_l_max_ <= lane_ranges[i].right_boundary_)
		{
			LaneRange_CarIn_max = lane_ranges[i].right_boundary_;
			right_range_flag = 1;
		}

		// 给车道进行分类:左侧，中间，右侧
		if(lane_ranges[i].right_boundary_ < 0)
		{
			lane_vec_left_.push_back(lane_ranges[i]);	
		}
		else if(lane_ranges[i].left_boundary_ > 0)
		{
			lane_vec_right_.push_back(lane_ranges[i]);	
		}
		else if((lane_ranges[i].left_boundary_ < 0) && (lane_ranges[i].right_boundary_ > 0))
		{
			lane_vec_middle_.push_back(lane_ranges[i]);	
		}
		else
		{
			ROS_WARN("LaneChange: lane rang is illegal,left = %f,right = %f",lane_ranges[i].left_boundary_,lane_ranges[i].right_boundary_);
		}
	}

	// 如果车辆宽度没有计算出来，则强制修改车辆占据的车道宽度
	if((left_range_flag == 0)&&(right_range_flag == 0))
	{
		LaneRange_CarIn_min = car_center_frenet_.getL() - 2*CAR_WIDTH;
		LaneRange_CarIn_max = car_center_frenet_.getL() + 2*CAR_WIDTH; 
	}
	else if(left_range_flag == 0)
	{
		LaneRange_CarIn_min = LaneRange_CarIn_max - 2*CAR_WIDTH;
	}
	else if(right_range_flag == 0)
	{
		LaneRange_CarIn_max = LaneRange_CarIn_min + 2*CAR_WIDTH;
	}

	ROS_INFO("LaneChange: Current Car in lane range left is  = %f , right is  = %f",LaneRange_CarIn_min,LaneRange_CarIn_max);

	return true;
}



bool LaneChange::ClassifyObstacles()	//根据车辆位置将障碍物分为三类：左侧，中间，右侧
{
	// 清除障碍物信息
	obstacle_vec_left_.clear();
	obstacle_vec_left_middle_.clear();
	obstacle_vec_middle_.clear();
	obstacle_vec_right_.clear();
	obstacle_vec_right_middle_.clear();

	// 遍历障碍物信息，将障碍物进行分类
	for (Obstacle &obs : envi_info_.getObstacleVec())
	{
		// 忽略不在道路范围内的障碍物
		if((obs.getLmax() <= LaneRange_min) && (obs.getLmin() >= LaneRange_max) )
		{
			continue;
		}
	
		// 左侧障碍物
		if(obs.getLmax() <= LaneRange_CarIn_min) 
		{
			obstacle_vec_left_.push_back(obs);
			continue;
		}
		
		// 右侧障碍物
		if(obs.getLmin() >= LaneRange_CarIn_max) 
		{
			obstacle_vec_right_.push_back(obs);
			continue;
		}

		// 中间障碍物
		if((obs.getLmin() >= LaneRange_CarIn_min) && (obs.getLmax() <= LaneRange_CarIn_max))
		{
			obstacle_vec_middle_.push_back(obs);
			continue;
		}

		// 左中障碍物
		if((LaneRange_CarIn_min >= obs.getLmin()) && (LaneRange_CarIn_min <= obs.getLmax()))
		{
			obstacle_vec_left_middle_.push_back(obs);
			continue;
		}

		// 右中障碍物
		if((LaneRange_CarIn_max >= obs.getLmin()) && (LaneRange_CarIn_max <= obs.getLmax()))
		{
			obstacle_vec_right_middle_.push_back(obs);
			continue;
		}

		return false;
	}


	ROS_INFO("LaneChange--Classify:Valid Obstacle distribution left = %d, right = %d, middle = %d, left_middle = %d, right_middle = %d",obstacle_vec_left_.size(),obstacle_vec_right_.size(),obstacle_vec_middle_.size(),obstacle_vec_left_middle_.size(),obstacle_vec_right_middle_.size());

	return true;
}



double LaneChange::CalMinDisWithFrontObstacles(std::vector< Obstacle > obstacle_vec)	// 计算和前方障碍物的最小距离
{
	double min_dis = MAX_NUM;
	double obs_s_min = MAX_NUM;

	for (Obstacle &obs : obstacle_vec)	// 依次遍历障碍物信息
	{
		// 只考虑车辆前方的障碍物
		if(obs.getSmin() > car_s_max_)
		{
			// 更新车辆和障碍物的最小距离
			if(obs.getSmin() < obs_s_min) 
			{
				obs_s_min = obs.getSmin();
			}
		}
	}

	min_dis =  obs_s_min - car_s_max_;

	return min_dis;
}


bool LaneChange::isObstacleOccupation(std::vector< Obstacle > obstacle_vec,double s_range)	//判断车辆前方一定范围内被障碍物占据
{
	double smin = car_s_min_;
	double smax = car_s_max_ + 0.8*s_range; 	// 增加系数0.8，即换道判断容忍距离为20m，小于25m的触发距离 20190910

	ROS_INFO("LaneChange--Occupation: lane range protected is smin = %f, smax = %f,s_range = %f ",smin,smax,s_range);
	ROS_INFO("LaneChange--Occupation: Check obstacle_vec size = %d ",obstacle_vec.size());
	
	for (Obstacle &obs : obstacle_vec)	// 依次遍历障碍物信息
	{
		ROS_INFO("LaneChange--Occupation: obs_min = %f, obs_max = %f",obs.getSmin(),obs.getSmax());

		if(((obs.getSmax() > smin) && (obs.getSmax() < smax)) || ((obs.getSmin() > smin) && (obs.getSmin() < smax)))
		{

			ROS_INFO("LaneChange--Occupation: Obstacle cover range is obs_smin = %f, obs_smax = %f ",obs.getSmin(),obs.getSmax());
			return true;
		}
	}

	return false;
}



int LaneChange::getLaneChangeCmd() const	// 获取换道指令
{
	return cmd_type_;
}




























}
