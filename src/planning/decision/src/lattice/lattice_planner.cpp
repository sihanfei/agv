#include "utils.h"

namespace pnc
{


bool pairSortCost(TrajectoryPair &p1, TrajectoryPair &p2)
{
  return p1.getCost() < p2.getCost();
}


LatticePlanner::LatticePlanner(FrenetPoint point_start,FrenetPoint point_end,ReferenceLine ref_line,PerceptionInfo envi_info,SpeedMap speed_map,FrenetPoint car_point_frenet,int lane_change_cmd,bool theta_modify_flag)
:point_start_(point_start),point_end_(point_end),ref_line_(ref_line),envi_info_(envi_info),speed_map_(speed_map),car_point_fre_(car_point_frenet),lane_change_cmd_(lane_change_cmd),theta_modify_flag_(theta_modify_flag)
{

	planning_ok_ = 0;	// 初始化为0

	ROS_INFO("Planning:Start Point S0 = %f, L0 = %f .End Point S1 = %f, L1 = %f .",point_start_.getS(),point_start_.getL(),point_end_.getS(),point_end_.getL());

	// 计算车辆中心点在Cartesian坐标下的坐标
	CartesianPoint point_center;
	if (!frenetToCartesian(car_point_fre_, ref_line_, point_center))
	{
		ROS_WARN("Planning:car center Frenet point transform to Cartesian failed.");
		return;
	}

	double x0 = point_center.getX();
	double y0 = point_center.getY();
	double theta0 = point_center.getTheta();

	// 通过车辆中心点坐标，计算车辆四个顶点的坐标
	double x[4],y[4];
	x[0] = x0 + CAR_LENGTH/2*cos(theta0) - CAR_WIDTH/2*sin(theta0); 
	y[0] = y0 + CAR_LENGTH/2*sin(theta0) + CAR_WIDTH/2*cos(theta0);
	x[1] = x0 + CAR_LENGTH/2*cos(theta0) + CAR_WIDTH/2*sin(theta0); 
	y[1] = y0 + CAR_LENGTH/2*sin(theta0) - CAR_WIDTH/2*cos(theta0); 
	x[2] = x0 - CAR_LENGTH/2*cos(theta0) + CAR_WIDTH/2*sin(theta0); 
	y[2] = y0 - CAR_LENGTH/2*sin(theta0) - CAR_WIDTH/2*cos(theta0); 
	x[3] = x0 - CAR_LENGTH/2*cos(theta0) - CAR_WIDTH/2*sin(theta0); 
	y[3] = y0 - CAR_LENGTH/2*sin(theta0) + CAR_WIDTH/2*cos(theta0); 

	// 构造车辆四个顶点的包络
	box_car_ = Box2d(Vect(x[0], y[0]),Vect(x[1], y[1]),Vect(x[2], y[2]),Vect(x[3], y[3]));

	// 判断是否触发了停车规划
	stop_flag_ = 0;
	double dis_target = point_end_.getS() - car_point_fre_.getS();
	double dis_stop = 0.5*point_start_.getdS()*point_start_.getdS()/COMFORT_DEC;		// 舒适停车距离
	if(dis_target <= dis_stop + DISTANCE_STOP_PLAN)
	{
		stop_flag_ = 1;
		ROS_INFO("Planning:Start Stop Planning.stop planning distance  = %f ",dis_target);
	}

	// 若车辆已经到达目标点，则直接认为规划成功 20190917
	if(dis_target < EPSILON)
	{
		ROS_INFO("Planning: car has been arrived the target.");
		planning_ok_ = 1;
		return;
	}


	if(fabs(lane_change_cmd_) > 1)
	{
		ROS_WARN("Planning: Lane Change cmd is error ,lane change cmd_ = %d.",lane_change_cmd_);
		return;
	}

	// 计算撒点的中间值
	l_mid_ = WidthLane * lane_change_cmd_;

	TrajectoryGenerate_Latitude();		// 生成横向轨迹
	TrajectoryGenerate_Longitude();		// 生成纵向轨迹
	Trajectory_Pair();								// 横纵轨迹两两配对
	Trajectory_Cost();								// 轨迹对评分
	Trajectory_Combine();							// 横纵轨迹集合

}


// 生成横向轨迹
void LatticePlanner::TrajectoryGenerate_Latitude()	
{

	double s0 = point_start_.getS();			
	double l0 = point_start_.getL();
	double dl0 = point_start_.getdL();
	double ddl0 = point_start_.getddL();


	double v0 = car_point_fre_.getdS();		// 车辆当前车速
	double vc = VEL_LIMIT;								// 最大车体运行速度

  double t_plan_max = PLAN_TIME_MAX;	// 最大规划时间
  double t_plan_min = PLAN_TIME_MIN;	// 最小规划时间

  
	// 计算车辆能往前走的最大距离（用最大规划时间进行计算）
	double d_max = 0.0;			// 横向最大撒点距离
  if (v0 < vc)						// 当前车速小于最大车速
  {
    double ta1 = (vc - v0) / ACC_LIMIT;	// 最快的加速时间

    if (ta1 < t_plan_max)
    {
      d_max = vc * (t_plan_max - ta1) + 0.5 * (vc * vc - v0 * v0) / ACC_LIMIT;
    }
    else
    {
      d_max = 0.5 * (vc * vc - v0 * v0) / ACC_LIMIT;
    }
  }
  else	// 当前车速大于最大车速
  {
		double td1 = (v0 -vc)/DEC_LIMIT;	// 最短的减速时间

		if(td1 < t_plan_max)
		{
			d_max = vc * (t_plan_max - td1) + 0.5 * (v0 * v0 - vc * vc) / DEC_LIMIT;
		}
		else
		{
			d_max = 0.5 * (v0 * v0 - vc * vc) / DEC_LIMIT;
		}
  }


	// 计算车辆能往前走的最小距离（用最小规划时间进行计算）
  double d_min = 0.0;						// 横向最小撒点距离
  double td2 = v0 / DEC_LIMIT; 	// 减速停车需要的最小时间

  if (td2 > t_plan_min)	// 最小规划时间内，无法减速停车
  {
    d_min = v0 * t_plan_min - 0.5 * DEC_LIMIT * t_plan_min * t_plan_min;
  }
  else
  {
    d_min = 0.5 * (v0 * v0) / DEC_LIMIT;
  }


	//	将行走可区间4等分进行撒点，考虑目标点位置，防止横向撒点超过目标点
	double dis_target = point_end_.getS() - point_start_.getS();	

	if(d_max > dis_target)
	{
		d_max = dis_target;
	}

	// 在接近目标点时，会出现d_min<d_max的情况，因此需要将这种情况进行排除
	if(d_min >= d_max)
	{
		d_min = 0;
	}

	double d_interval = (d_max - d_min) / 4.0;

	//ROS_INFO("Planning--Generate:Latitude Trajectory Generate d_min = %f.d_max = %f",d_min,d_max);

	// 五次多项式曲线拟合，得到横向轨迹
	for (double ds = d_min; ds < d_max + EPSILON; ds += d_interval)
	{
		double s1 = s0 + ds;
		for (double l1 = -L_RESOLUTION + l_mid_; l1 <= L_RESOLUTION + l_mid_; l1 += L_RESOLUTION)	// l撒3个点，每0.2m撒一个点
		{
			std::shared_ptr< Curve > ls_path_ptr(new TrajectoryCurve(std::shared_ptr< Curve >(new QuinticPolynomial(l0, dl0, ddl0, l1, 0.0, 0.0, 0.0, ds))));

			if (ls_path_ptr->isValid())
			{
				paths_vec_ls_.emplace_back(ls_path_ptr);	// 记录该轨迹信息
			}
			else
			{
				ROS_WARN("Planning--Generate:Lane Change Latitude Trajectory Generate Failed. l0=%f;dl0=%f; ddl0=%f; l1=%f; ds =%f",l0, dl0, ddl0, l1,ds);
			}
		}
	}

}	


// 生成纵向轨迹		
void LatticePlanner::TrajectoryGenerate_Longitude()	
{

	double s0 = point_start_.getS();
	double v0 = point_start_.getdS();
	double a0 = point_start_.getddS();

	// 考虑到车体启动的时候，需要一个很小的运动速度，才能让车体动起来，因此，针对车辆处于静止的情况，轨迹的开始速度最低设置为0.1m/s
	if(fabs(car_point_fre_.getdS()) < EPSILON)
	{
		v0 = VEL_START;	
	}

	// 停车规划
	if(stop_flag_ == 1)
	{
		double s_target = point_end_.getS();
		double s_interval = (s_target - s0) / 4.0;	
		for (double s1 = s0; s1 < s_target + EPSILON; s1 += s_interval) //将撒点轨迹的终点由目标点修改为分段目标点，以避免直接撒向终点的轨迹全部不合理的情况
		{
			for (double t1 = PLAN_TIME_MIN; t1 <= PLAN_TIME_MAX; t1 = t1 + TIME_DENSITY)
			{
				// 五次多项式曲线拟合
				std::shared_ptr<Curve> st_path_ptr(new TrajectoryCurve(std::shared_ptr<Curve>(new QuinticPolynomial(s0, v0, a0, s1, 0.0, 0.0, 0.0, t1))));

				if (st_path_ptr->isValid())
				{
					paths_vec_st_.emplace_back(st_path_ptr);	// 记录该轨迹信息
				}
				else
				{
					ROS_WARN("Planning--Generate:Longitude Stop Trajectory Generate Failed. s0=%f; v0=%f; a0=%f; v1=0; a1=0; t0=0; t1=%f",s0,v0,a0,t1);
				}
			}
		}
		return;
	}


	// 巡航规划
	// 将速度地图上的速度结合起来
	uint8_t speed_map_index = speed_map_.getIntervalIndexByS(car_point_fre_.getS());	
	double speed_map_s_min = speed_map_.getSminByIndex(speed_map_index);
	double speed_map_s_max = speed_map_.getSmaxByIndex(speed_map_index);
	double speed_map_v_limit = speed_map_.getSpeedLimitByIndex(speed_map_index);

	double v_max_map = speed_map_v_limit < VEL_LIMIT ? speed_map_v_limit : VEL_LIMIT;

	// 车辆距离下一个速度区间10m时，切换最大撒点速度
	if(speed_map_s_max - car_point_fre_.getS() <= DISTANCE_VEL_CHANGE)	
	{
		if(speed_map_index != speed_map_.getMaxIndex())	// 车辆不在最后一段速度区间内
		{
			double speed_map_v_limit_next = speed_map_.getSpeedLimitByIndex(speed_map_index + 1);
			// 下一段速度大于当前段速度，采用当前速度，小于当前段速度，采用小一段速度
			v_max_map = speed_map_v_limit_next < v_max_map ? speed_map_v_limit_next : v_max_map;	
		}
	}
	
	v_cruise_ = v_max_map;	// 记录当前路段的巡航速度

	ROS_INFO("Planning--Generate:Start Cruise Planning.Max Cruise Speed is = %f ",v_cruise_);

	for (double t1 = PLAN_TIME_MIN; t1 <= PLAN_TIME_MAX; t1 = t1 + TIME_DENSITY)
	{
		// 计算撒点区间
		double vel_permite_max = car_point_fre_.getdS() + ACC_LIMIT * t1;
		double vel_permite_min = car_point_fre_.getdS() - DEC_LIMIT * t1;
		double vel_sample_min = vel_permite_min < 0.0 ? 0.0 : vel_permite_min;
		double vel_sample_max = vel_permite_max > v_cruise_ ? v_cruise_ : vel_permite_max;

		double vel_interval = (vel_sample_max - vel_sample_min)/ VEL_SAMPLE_NUM;

		for (double v1 = vel_sample_min; v1 <= vel_sample_max; v1 += vel_interval)
		{
			// 四次多项式曲线拟合
			std::shared_ptr<Curve> st_path_ptr(new TrajectoryCurve(std::shared_ptr<Curve>(new QuarticPolynomial(s0, v0, a0, v1, 0.0, 0.0, t1))));

			if (st_path_ptr->isValid())
			{
				paths_vec_st_.emplace_back(st_path_ptr);	// 记录该轨迹信息
			}
			else
			{
				ROS_WARN("Planning--Generate:Longitude Trajectory Generate Failed. s0=%f; v0=%f; a0=%f; v1=%f; a1=0; t0=0; t1=%f",s0,v0,a0,v1,t1);
			}
		}
	}


	// 超车和跟车规划，由于障碍物速度不稳定，暂时不考虑

}	


// 横纵轨迹两两配对		
void LatticePlanner::Trajectory_Pair()					
{
	ROS_INFO("Planning--Pair: ST traj num is %d", paths_vec_st_.size());
	ROS_INFO("Planning--Pair: LS traj num is %d", paths_vec_ls_.size());	


	std::vector< std::shared_ptr< Curve > > valid_paths_vec_ls_;	// 有效的横向轨迹
	std::vector< std::shared_ptr< Curve > > valid_paths_vec_st_;	// 有效的纵向轨迹

	//对纵向轨迹进行有效性判断
	uint32_t valid_num_st = 0;
	for (std::shared_ptr< Curve > st_path : paths_vec_st_)	// 依次遍历纵向轨迹
	{
		bool valid_flag_st = 1;
		for (double sample_t = st_path->paramMin(); sample_t <= st_path->paramMax(); sample_t = sample_t + SCORE_TIME_RESOLUTION)
		{
			double vel = st_path->evaluate(1, sample_t);

		  if (vel < -EPSILON || vel > VEL_LIMIT)
		  {
		    valid_flag_st = 0;
				//ROS_WARN("Planning--Pair: the vel is invalid. vel = %f",vel);
				break;
		  }

		  double acc = st_path->evaluate(2, sample_t);
		  if (acc > ACC_LIMIT || acc < -DEC_LIMIT)
		  {
		    valid_flag_st = 0;
				//ROS_WARN("Planning--Pair: the acc is invalid. acc = %f",acc);
				break;
		  }
		}
		if(valid_flag_st == 0)
		{
			continue;
		}
		valid_num_st ++;
		valid_paths_vec_st_.push_back(st_path);
	}

	//对横向轨迹进行有效性判断
	uint32_t valid_num_sl = 0;
	for (std::shared_ptr< Curve > ls_path : paths_vec_ls_)
	{
		// 检测横向轨迹是否有效
		bool valid_flag_sl = 1;
		for (double sample_s = ls_path->paramMin(); sample_s <= ls_path->paramMax(); sample_s = sample_s + SCORE_DIS_RESOLUTION)
		{
			double l_temp = ls_path->evaluate(0, sample_s);

			// 简化判断逻辑，直接通过车辆中心点来判断，防止生成的轨迹检验时间过长,20190930
			if((l_temp < -1.5*WidthLane - L_RESOLUTION) || (l_temp >1.5*WidthLane + L_RESOLUTION))
			{
				valid_flag_sl = 0;
				continue;
			}
/*
			// 获取当前点车道宽信息
			double s_temp = point_start_.getS() + sample_s;
			ReferenceLinePoint ref_point_temp = ref_line_.getReferenceLinePointByS(s_temp);	
			std::vector< LaneRange > lane_ranges_temp = ref_point_temp.getLaneRanges();
			double left_limit = lane_ranges_temp[0].left_boundary_;
			double right_limit = lane_ranges_temp[lane_ranges_temp.size()-1].right_boundary_;

			//ROS_INFO("Planning--Pair:left_limit = %f,right_limit = %f",left_limit,right_limit);

			// 将轨迹点转换到Cartesian下
			CartesianPoint check_p_cartesian;
			FrenetPoint check_p_frenet(s_temp, 0.0, 0.0, l_temp, 0.0, 0.0);
			if (!frenetToCartesian(check_p_frenet, ref_line_, check_p_cartesian))
			{
				ROS_WARN("Planning--Pair:Trajectory Frenet point transform to Cartesian failed.");
				valid_flag_sl = 0;
				continue;
			}
			// 计算车辆四个顶点信息
			double x0 = check_p_cartesian.getX();
			double y0 = check_p_cartesian.getY();
			double theta0 = check_p_cartesian.getTheta();
			double x[4],y[4];
			x[0] = x0 + CAR_LENGTH/2*cos(theta0) - CAR_WIDTH/2*sin(theta0); 
			y[0] = y0 + CAR_LENGTH/2*sin(theta0) + CAR_WIDTH/2*cos(theta0);
			x[1] = x0 + CAR_LENGTH/2*cos(theta0) + CAR_WIDTH/2*sin(theta0); 
			y[1] = y0 + CAR_LENGTH/2*sin(theta0) - CAR_WIDTH/2*cos(theta0); 
			x[2] = x0 - CAR_LENGTH/2*cos(theta0) + CAR_WIDTH/2*sin(theta0); 
			y[2] = y0 - CAR_LENGTH/2*sin(theta0) - CAR_WIDTH/2*cos(theta0); 
			x[3] = x0 - CAR_LENGTH/2*cos(theta0) - CAR_WIDTH/2*sin(theta0); 
			y[3] = y0 - CAR_LENGTH/2*sin(theta0) + CAR_WIDTH/2*cos(theta0); 
			// 计算车辆在frenet坐标下最大最小和l值
			double l_temp_max = -MAX_NUM;
			double l_temp_min = MAX_NUM;
			for(int i = 0; i < 4; i++)
			{
				CartesianPoint p_cartesian;
				p_cartesian.setX(x[i]);
				p_cartesian.setY(y[i]);
				p_cartesian.setTheta(theta0);
				p_cartesian.setVel(0.0);
				p_cartesian.setAcc(0.0);
				p_cartesian.setKappa(0.0);
				// 将坐标转换到frenet坐标系下
				FrenetPoint p_frenet;
				if (!cartesianToFrenet(p_cartesian, ref_line_, p_frenet))
				{
					//ROS_WARN("Decision--Pair: car corner %d point cartesian transform to Frenet failed.",i);
					valid_flag_sl = 0;
					break;
				}
				// 计算frenet边界L最大最小范围
				if(l_temp_max < p_frenet.getL())
				{
					l_temp_max = p_frenet.getL();
				}
				if(l_temp_min > p_frenet.getL())
				{
					l_temp_min = p_frenet.getL();
				}
			}

			// 判断车的边界是否会超出车道
			//if((l_temp_min < left_limit) || (l_temp_max >right_limit))
			if((l_temp_min < -2*WidthLane) || (l_temp_max >2*WidthLane))
			{
				//ROS_WARN("Decision--Pair: car 's boundary out of lanes range.l_temp_min = %f,l_temp_max = %f",l_temp_min,l_temp_max);
				valid_flag_sl = 0;
				continue;
			}
*/
		}

		if(valid_flag_sl == 0)
		{
			continue;
		}

		valid_num_sl ++;
		valid_paths_vec_ls_.push_back(ls_path);
	}


	//横向轨迹和纵向轨迹进行一一配对
	for (std::shared_ptr< Curve > st_path : valid_paths_vec_st_)
	{
		for (std::shared_ptr< Curve > ls_path : valid_paths_vec_ls_)
		{
			TrajectoryPair traj_pair(ls_path.get(), st_path.get());
			traj_pair_vec_.emplace_back(traj_pair);
		}
	}

	ROS_INFO("Planning--Pair:Trajectory Pair Valid num = %d ST Valid num = %d SL Valid num = %d", traj_pair_vec_.size(),valid_num_st,valid_num_sl);

}									


// 轨迹对评分	
void LatticePlanner::Trajectory_Cost()			
{

	// 定义轨迹的单项最大最小值
	double MIN_Cost_Lat_Jerk = MAX_NUM;
	double MAX_Cost_Lat_Jerk = -MAX_NUM;

	double MIN_Cost_Lat_S = MAX_NUM;
	double MAX_Cost_Lat_S = -MAX_NUM;

	double MIN_Cost_Lat_L = MAX_NUM;
	double MAX_Cost_Lat_L = -MAX_NUM;

	double MIN_Cost_Lon_Jerk = MAX_NUM;
	double MAX_Cost_Lon_Jerk = -MAX_NUM;

	double MIN_Cost_Lon_T = MAX_NUM;
	double MAX_Cost_Lon_T = -MAX_NUM;

	double MIN_Cost_Lon_DV = MAX_NUM;
	double MAX_Cost_Lon_DV = -MAX_NUM;

	double MIN_Cost_Lon_DS = MAX_NUM;
	double MAX_Cost_Lon_DS = -MAX_NUM;


	// 依次遍历每一对轨迹，求轨迹对的单项cost，并得到单项最大最小cost值
	for (uint32_t i = 0; i < traj_pair_vec_.size(); ++i)
	{
		TrajectoryPair &traj_pair = traj_pair_vec_[i];

		Curve *st_path_ptr = traj_pair.getSTPath();					// 获取ST轨迹
		Curve *ls_path_ptr = traj_pair.getLSPath();					// 获取LS轨迹
		double s0 = point_start_.getS(); 										// 计算车辆当前位置


		// 对横纵轨迹进行单项评分	
		double Jerk_Int_LAT = fabs(calJerkInt_LS(traj_pair));	// 计算横向轨迹的平滑度

//		double S    = traj_pair.getLSPath()->paramLength();		//计算横向轨迹的最大长度，
//		double S1  = 1/(S + EPSILON);	//为了表示轨迹越长越好，取倒数
//		double d1   = traj_pair.getLSPath()->getx1();					//计算横向轨迹最后偏移参考线的值

		// 结合ST轨迹取有效的轨迹长度，来进行评分 20190930
		double T_st = st_path_ptr->paramMax();															// 纵向轨迹的规划时间
		double S_st = st_path_ptr->evaluate(0, T_st) - point_start_.getS();	// 纵向轨迹的规划长度
		double S_sl = ls_path_ptr->paramLength();														// 横向轨迹的规划长度
		double S = S_sl > S_st ? S_st : S_sl;																// 有效长度取其中较小的
		double S1  = 1/(S + EPSILON);	//为了表示轨迹越长越好，取倒数

		// 根据最大预测时间计算轨迹最后的偏移量而不是根据轨迹的终点来计算 20190920
		double d1;	//计算横向轨迹最后偏移参考线的值

/*
		double s = st_path_ptr->evaluate(0, PLAN_TIME_MAX);	// 车辆在最大预测时间内可以到达的s值 
		// 预测时间周期内，车辆已经超过sl轨迹最大长度了
		if(s > s0 + ls_path_ptr->paramMax())
		{
			d1 = traj_pair.getLSPath()->getx1(); //计算横向轨迹最后偏移参考线的值
		}
		else
		{
			double s_l = s - s0;
			d1 = ls_path_ptr->evaluate(0, s_l); //计算最大预测时间内最后偏移参考线的值
		}
*/
		// 每1s在轨迹上取一个点，求偏差的平均值，作为评价指标d1
		double sum_d = 0;
		double d[8];
		for(uint8_t i = 0;i < PLAN_TIME_MAX; i++)
		{
			double t = i+1;
			double s = st_path_ptr->evaluate(0, t);
			if(s > s0 + ls_path_ptr->paramMax())
			{
				d[i] = traj_pair.getLSPath()->getx1(); //计算横向轨迹最后偏移参考线的值
			}
			else
			{
				double s_l = s - s0;
				d[i] = ls_path_ptr->evaluate(0, s_l); //计算最大预测时间内最后偏移参考线的值
			}
			sum_d = sum_d + fabs(d[i]);	
		}
		d1 = sum_d/8;

		// 换道时，越快偏离车道越好 20190927
		if(lane_change_cmd_ != LANE_CHANGE_FORBID)
		{
			d1 = 1/(d1 + EPSILON);
		}


		traj_pair.setCost_lat_jerk(Jerk_Int_LAT);	
		traj_pair.setCost_lat_s(S1);							
		traj_pair.setCost_lat_l(d1*d1);				

		if (stop_flag_) // 若停车或混行
		{			
			double Jerk_Int_LON = fabs(calJerkInt_ST(traj_pair));							// 计算纵向轨迹的平滑度	
			double T = traj_pair.getSTPath()->paramLength();									// 计算纵向轨迹的规划时间
			double T1 = 1/(T + EPSILON); 	//表示车辆越慢停车越好,20190912,防止减速度太大，车辆停不下来
			double d_S = point_end_.getS() - traj_pair.getSTPath()->getx1();	// 计算纵向轨迹终点和目标点的距离

			traj_pair.setCost_lon_jerk(Jerk_Int_LON);	
			// traj_pair.setCost_lon_t(T);	
			traj_pair.setCost_lon_t(T1);							
			traj_pair.setCost_lon_s(d_S*d_S);
		}
		else
		{	
			double Jerk_Int_LON   = fabs(calJerkInt_ST(traj_pair));		// 计算纵向轨迹的平滑度，将评价参数中的T改为1/T，表示巡航规划时T越大越好		
			double T = traj_pair.getSTPath()->paramLength();					// 计算纵向轨迹的规划时间
			double T1 = 1/(T + EPSILON); 	//为了表示轨迹越长越好，取倒数
			double d_V = traj_pair.getSTPath()->getdx1() - v_cruise_;	// 计算纵向轨迹最后偏离巡航速度的差值

			traj_pair.setCost_lon_jerk(Jerk_Int_LON);
			traj_pair.setCost_lon_t(T1);								
			traj_pair.setCost_lon_v(d_V*d_V);					
		}


		// 计算所有轨迹对的单项cost的最大最小值
		MAX_Cost_Lat_Jerk = MAX_Cost_Lat_Jerk < traj_pair.getCost_lat_jerk() ? traj_pair.getCost_lat_jerk() : MAX_Cost_Lat_Jerk;
		MIN_Cost_Lat_Jerk = MIN_Cost_Lat_Jerk > traj_pair.getCost_lat_jerk() ? traj_pair.getCost_lat_jerk() : MIN_Cost_Lat_Jerk;

		MAX_Cost_Lat_S = MAX_Cost_Lat_S < traj_pair.getCost_lat_s() ? traj_pair.getCost_lat_s() : MAX_Cost_Lat_S;
		MIN_Cost_Lat_S = MIN_Cost_Lat_S > traj_pair.getCost_lat_s() ? traj_pair.getCost_lat_s() : MIN_Cost_Lat_S;

		MAX_Cost_Lat_L = MAX_Cost_Lat_L < traj_pair.getCost_lat_l() ? traj_pair.getCost_lat_l() : MAX_Cost_Lat_L;
		MIN_Cost_Lat_L = MIN_Cost_Lat_L > traj_pair.getCost_lat_l() ? traj_pair.getCost_lat_l() : MIN_Cost_Lat_L;

		MAX_Cost_Lon_Jerk = MAX_Cost_Lon_Jerk < traj_pair.getCost_lon_jerk() ? traj_pair.getCost_lon_jerk() : MAX_Cost_Lon_Jerk;
		MIN_Cost_Lon_Jerk = MIN_Cost_Lon_Jerk > traj_pair.getCost_lon_jerk() ? traj_pair.getCost_lon_jerk() : MIN_Cost_Lon_Jerk;

		MAX_Cost_Lon_T = MAX_Cost_Lon_T < traj_pair.getCost_lon_t() ? traj_pair.getCost_lon_t() : MAX_Cost_Lon_T;
		MIN_Cost_Lon_T = MIN_Cost_Lon_T > traj_pair.getCost_lon_t() ? traj_pair.getCost_lon_t() : MIN_Cost_Lon_T;

		if (stop_flag_)
		{
			MAX_Cost_Lon_DS = MAX_Cost_Lon_DS < traj_pair.getCost_lon_s() ? traj_pair.getCost_lon_s() : MAX_Cost_Lon_DS;
			MIN_Cost_Lon_DS = MIN_Cost_Lon_DS > traj_pair.getCost_lon_s() ? traj_pair.getCost_lon_s() : MIN_Cost_Lon_DS;
		}
		else
		{
			MAX_Cost_Lon_DV = MAX_Cost_Lon_DV < traj_pair.getCost_lon_v() ? traj_pair.getCost_lon_v() : MAX_Cost_Lon_DV;
			MIN_Cost_Lon_DV = MIN_Cost_Lon_DV > traj_pair.getCost_lon_v() ? traj_pair.getCost_lon_v() : MIN_Cost_Lon_DV;
		}
	}

	// 实际过程中，会出现轨迹对中只有一条有效的SL轨迹或者ST轨迹，导致单项的最大值和最小值相等，从而出现计算到的cost为无穷大的情况
	if(MAX_Cost_Lat_Jerk <= MIN_Cost_Lat_Jerk)
	{
		MAX_Cost_Lat_Jerk = 1.0;
		MIN_Cost_Lat_Jerk = 0.0;
	}
	if(MAX_Cost_Lat_S <= MIN_Cost_Lat_S)
	{
		MAX_Cost_Lat_S = 1.0;
		MIN_Cost_Lat_S = 0.0;
	}
	if(MAX_Cost_Lat_L <= MIN_Cost_Lat_L)
	{
		MAX_Cost_Lat_L = 1.0;
		MIN_Cost_Lat_L = 0.0;
	}
	if(MAX_Cost_Lon_Jerk <= MIN_Cost_Lon_Jerk)
	{
		MAX_Cost_Lon_Jerk = 1.0;
		MIN_Cost_Lon_Jerk = 0.0;
	}
	if(MAX_Cost_Lon_T <= MIN_Cost_Lon_T)
	{
		MAX_Cost_Lon_T = 1.0;
		MIN_Cost_Lon_T = 0.0;
	}
	if(MAX_Cost_Lon_DS <= MIN_Cost_Lon_DS)
	{
		MAX_Cost_Lon_DS = 1.0;
		MIN_Cost_Lon_DS = 0.0;
	}
	if(MAX_Cost_Lon_DV <= MIN_Cost_Lon_DV)
	{
		MAX_Cost_Lon_DV = 1.0;
		MIN_Cost_Lon_DV = 0.0;
	}
/*
	ROS_INFO("Planning--Cost:MAX_Cost_Lat_Jerk = %f,MIN_Cost_Lat_Jerk = %f",MAX_Cost_Lat_Jerk,MIN_Cost_Lat_Jerk);
	ROS_INFO("Planning--Cost:MAX_Cost_Lat_S = %f,MIN_Cost_Lat_S = %f",MAX_Cost_Lat_S,MIN_Cost_Lat_S);
	ROS_INFO("Planning--Cost:MAX_Cost_Lat_L = %f,MIN_Cost_Lat_L = %f",MAX_Cost_Lat_L,MIN_Cost_Lat_L);
	ROS_INFO("Planning--Cost:MAX_Cost_Lon_Jerk = %f,MIN_Cost_Lon_Jerk = %f",MAX_Cost_Lon_Jerk,MIN_Cost_Lon_Jerk);
	ROS_INFO("Planning--Cost:MAX_Cost_Lon_T = %f,MIN_Cost_Lon_T = %f",MAX_Cost_Lon_T,MIN_Cost_Lon_T);
	ROS_INFO("Planning--Cost:MAX_Cost_Lon_DS = %f,MIN_Cost_Lon_DS = %f",MAX_Cost_Lon_DS,MIN_Cost_Lon_DS);
	ROS_INFO("Planning--Cost:MAX_Cost_Lon_DV = %f,MIN_Cost_Lon_DV = %f",MAX_Cost_Lon_DV,MIN_Cost_Lon_DV);
*/
	// 依次遍历每一对轨迹，计算每个轨迹对归一化的cost值
	for (uint32_t i = 0; i < traj_pair_vec_.size(); ++i)
	{
		TrajectoryPair &traj_pair = traj_pair_vec_[i];

		double normal_cost_lat_jerk = 0.0;
		double normal_cost_lat_s = 0.0;
		double normal_cost_lat_l = 0.0;
		double normal_cost_lon_jerk = 0.0;
		double normal_cost_lon_t = 0.0;
		double normal_cost_lon_ds = 0.0;
		double normal_cost_lon_dv = 0.0;

		// 轨迹cost归一化
		normal_cost_lat_jerk = (traj_pair.getCost_lat_jerk() - MIN_Cost_Lat_Jerk) / (MAX_Cost_Lat_Jerk - MIN_Cost_Lat_Jerk);
		normal_cost_lat_s = (traj_pair.getCost_lat_s() - MIN_Cost_Lat_S) / (MAX_Cost_Lat_S - MIN_Cost_Lat_S);
		normal_cost_lat_l = (traj_pair.getCost_lat_l() - MIN_Cost_Lat_L) / (MAX_Cost_Lat_L - MIN_Cost_Lat_L);

		normal_cost_lon_jerk = (traj_pair.getCost_lon_jerk() - MIN_Cost_Lon_Jerk) / (MAX_Cost_Lon_Jerk - MIN_Cost_Lon_Jerk);
		normal_cost_lon_t = (traj_pair.getCost_lon_t() - MIN_Cost_Lon_T) / (MAX_Cost_Lon_T - MIN_Cost_Lon_T);
		if (stop_flag_)
		{
			normal_cost_lon_ds = (traj_pair.getCost_lon_s() - MIN_Cost_Lon_DS) / (MAX_Cost_Lon_DS - MIN_Cost_Lon_DS);
		}
		else
		{
			normal_cost_lon_dv = (traj_pair.getCost_lon_v() - MIN_Cost_Lon_DV) / (MAX_Cost_Lon_DV - MIN_Cost_Lon_DV);
		}

		// 求轨迹对的cost值
		double normal_cost_lat = 0;
		double normal_cost_lon = 0;
		double normal_cost_traj = 0;

		// 计算横向轨迹的cost
		normal_cost_lat = WEIGHT_LAT_JERK * normal_cost_lat_jerk + WEIGHT_LAT_S * normal_cost_lat_s + WEIGHT_LAT_L * normal_cost_lat_l;

		// 计算纵向轨迹的cost
		if (stop_flag_)
		{
			normal_cost_lon = WEIGHT_LON_JERK * normal_cost_lon_jerk + WEIGHT_LON_T * normal_cost_lon_t + WEIGHT_LON_S * normal_cost_lon_ds;
		}
		else
		{
			normal_cost_lon = WEIGHT_LON_JERK * normal_cost_lon_jerk + WEIGHT_LON_T * normal_cost_lon_t + WEIGHT_LON_V * normal_cost_lon_dv;
		}

		// 计算轨迹对的cost
		normal_cost_traj = WEIGHT_TOTAL_LAT * normal_cost_lat + WEIGHT_TOTAL_LON * normal_cost_lon;

		// 设置轨迹对的cost
		traj_pair.setCost(normal_cost_traj);

/*
		if((normal_cost_traj > 12) || (normal_cost_traj < 0) )
		{
			ROS_ERROR("Planning--Cost:normal_cost_traj = %f,normal_cost_lat = %f,normal_cost_lon = %f",normal_cost_traj,normal_cost_lat,normal_cost_lon);
			ROS_ERROR("Planning--Cost:normal_cost_lat_jerk = %f,normal_cost_lat_s = %f,normal_cost_lat_l = %f",normal_cost_lat_jerk,normal_cost_lat_s,normal_cost_lat_l);
			ROS_ERROR("Planning--Cost:normal_cost_lon_jerk = %f,normal_cost_lon_t = %f,normal_cost_lon_ds = %f,normal_cost_lon_dv = %f",normal_cost_lon_jerk,normal_cost_lon_t,normal_cost_lon_ds,normal_cost_lon_dv);
			ROS_ERROR("Planning--Cost:lat_jerk = %f,lat_s = %f,lat_l = %f",traj_pair.getCost_lat_jerk(),traj_pair.getCost_lat_s(),traj_pair.getCost_lat_l());
			ROS_ERROR("Planning--Cost:lon_jerk = %f,lon_t = %f,lon_s = %f,lon_v = %f",traj_pair.getCost_lon_jerk(),traj_pair.getCost_lon_t(),traj_pair.getCost_lon_s(),traj_pair.getCost_lon_v());
		}
*/


/*
		// 调参测试用
		if (stop_flag_)
		{
			ROS_INFO("Planning--Cost: lat_jerk = %f,lat_s = %f,lat_l = %f,lon_jerk = %f,lon_t = %f,lon_ds = %f",normal_cost_lat_jerk,normal_cost_lat_s,normal_cost_lat_l,normal_cost_lon_jerk,normal_cost_lon_t,normal_cost_lon_ds);	
		}
		else
		{
			ROS_INFO("Planning--Cost: lat_jerk = %f,lat_s = %f,lat_l = %f,lon_jerk = %f,lon_t = %f,lon_dv = %f",normal_cost_lat_jerk,normal_cost_lat_s,normal_cost_lat_l,normal_cost_lon_jerk,normal_cost_lon_t,normal_cost_lon_dv);	
		}

		//ROS_INFO("Planning--Cost:cost_traj = %f,cost_lat = %f,cost_lon = %f",normal_cost_traj,normal_cost_lat,normal_cost_lon);
*/
	}

	// 按照轨迹得分从高到地依次排序
	std::sort(traj_pair_vec_.begin(), traj_pair_vec_.end(), pairSortCost);
}


double LatticePlanner::calJerkInt_LS(const TrajectoryPair &traj_pair) // LS轨迹JERK积分
{

  // 计算五次曲线的jrek积分[t0,t1]
  // a[6]是五次多项式的六个系数，t1为终止状态参数值，t0为初始状态参数值
  // p = a[0] + a[1]*t + a[2]*t^2 + a[3]*t^3 + a[4]*t^4 + a[5]*t^5;

  double jerkMin = traj_pair.getLSPath()->evaluate(4, traj_pair.getLSPath()->paramMin());
  double jerkMax = traj_pair.getLSPath()->evaluate(4, traj_pair.getLSPath()->paramMax());

  return jerkMax - jerkMin;
}
	



double LatticePlanner::calJerkInt_ST(const TrajectoryPair &traj_pair) // ST轨迹JERK积分
{

  // 计算五次曲线的jrek积分[t0,t1]
  // a[6]是五次多项式的六个系数，t1为终止状态参数值，t0为初始状态参数值
  // p = a[0] + a[1]*t + a[2]*t^2 + a[3]*t^3 + a[4]*t^4 + a[5]*t^5;

  double jerkMin = traj_pair.getSTPath()->evaluate(4, traj_pair.getSTPath()->paramMin());
  double jerkMax = traj_pair.getSTPath()->evaluate(4, traj_pair.getSTPath()->paramMax());

  return jerkMax - jerkMin;

}





// 横纵轨迹集合				
void LatticePlanner::Trajectory_Combine()	
{
	// 将轨迹对进行结合，得到s，l，v的三维轨迹
	for (TrajectoryPair traj_pair : traj_pair_vec_)
	{

		ROS_INFO("Planning--Combine:the cost of traj pair = %f.",traj_pair.getCost());	
		if (stop_flag_)
		{
			ROS_INFO("Planning--Combine:Stop: lat_jerk = %f,lat_s = %f,lat_l = %f,lon_jerk = %f,lon_t = %f,lon_ds = %f",traj_pair.getCost_lat_jerk(),1/traj_pair.getCost_lat_s(),traj_pair.getCost_lat_l(),traj_pair.getCost_lon_jerk(),traj_pair.getCost_lon_t(),traj_pair.getCost_lon_s());	
		}
		else
		{
			ROS_INFO("Planning--Combine:Cruise: lat_jerk = %f,lat_s = %f,lat_l = %f,lon_jerk = %f,lon_t = %f,lon_dv = %f",traj_pair.getCost_lat_jerk(),1/traj_pair.getCost_lat_s(),traj_pair.getCost_lat_l(),traj_pair.getCost_lon_jerk(),1/traj_pair.getCost_lon_t(),traj_pair.getCost_lon_v());	
		}


		Curve *st_path_ptr = traj_pair.getSTPath();	// 获取ST轨迹
		Curve *ls_path_ptr = traj_pair.getLSPath();	// 获取LS轨迹

		Trajectory trajectory;

		std::vector<double> s_vec;
		std::vector<double> l_vec;
		std::vector<double> ds_vec;

		double t_plan = st_path_ptr->paramMax();
		ROS_INFO("Planning--Combine: st traj's Time  =  %f.",t_plan);	
		ROS_INFO("Planning--Combine: sl traj's Length  =  %f.",ls_path_ptr->paramMax());	

		// 每0.1s取一个点t*，在st轨迹上得到s*，进一步在ls轨迹上得到l*
		for (double t = 0.0; t <= t_plan; t = t + PLAN_TIME_RESOLUTION)
		//for (double t = 0.0; t <= PLAN_TIME_MAX; t = t + PLAN_TIME_RESOLUTION)
		{
			// 曲线拟合方程参数从0开始，采用下面方法
			double s0 = point_start_.getS();
			double s = st_path_ptr->evaluate(0, t);
			if (s > s0 + ls_path_ptr->paramMax())
			{
				break;
			}
			double dot_s = std::max(st_path_ptr->evaluate(1, t), EPSILON);	// 保证轨迹上的每一点的v都是正值
			double dot_dot_s = st_path_ptr->evaluate(2, t);
			double s_l = s - s0;	// 对应到曲线上的s插值点，需要减去偏移点
			double l = ls_path_ptr->evaluate(0, s_l);
			double dot_l = ls_path_ptr->evaluate(1, s_l);
			double dot_dot_l = ls_path_ptr->evaluate(2, s_l);
			//ROS_INFO("s = %f, dot_s = %f, dot_dot_s = %f, l = %f, dot_l = %f, dot_dot_l = %f",s, dot_s, dot_dot_s, l, dot_l, dot_dot_l);

/*
			// 曲线拟合方程参数从当前位置开始，采用下面方法
			double s = st_path_ptr->evaluate(0, t);
			if (s > ls_path_ptr->paramMax())
			{
				break;
			}
			double dot_s = std::max(st_path_ptr->evaluate(1, t), EPSILON);	// 保证轨迹上的每一点的v都是正值
			double dot_dot_s = st_path_ptr->evaluate(2, t);
			double l = ls_path_ptr->evaluate(0, s);
			double dot_l = ls_path_ptr->evaluate(1, s);
			double dot_dot_l = ls_path_ptr->evaluate(2, s);
			ROS_INFO("s = %f, dot_s = %f, dot_dot_s = %f, l = %f, dot_l = %f, dot_dot_l = %f",s, dot_s, dot_dot_s, l, dot_l, dot_dot_l);
*/
			
			FrenetPoint point(s, dot_s, dot_dot_s, l, dot_l, dot_dot_l);

			s_vec.push_back(s);
			ds_vec.push_back(dot_s);
			l_vec.push_back(l);

			trajectory.addFrenetTrajectoryPoint(point);		// 记录轨迹点
		}

		trajectory.s_vec_ = s_vec;
		trajectory.ds_vec_ = ds_vec;
		trajectory.l_vec_ = l_vec;

		trajectory.setCost(traj_pair.getCost());	// 记录该轨迹的cost值

		// 将轨迹上所有Frenet点都转换为Cartesian点
		for (FrenetPoint fre_point : trajectory.getFrenetTrajectory())
		{

			CartesianPoint car_point;
			if (!frenetToCartesian(fre_point, ref_line_, car_point))
			{
				ROS_ERROR("Planning--Combine:Trajectory Frenet point transform to Cartesian failed.");
			}

			//强行修改轨迹上的角度信息，使其和参考线保持一致，20191009
			if(theta_modify_flag_)	
			{
				ReferenceLinePoint ref_p = ref_line_.getReferenceLinePointByS(fre_point.getS());
				car_point.setTheta(ref_p.getTheta());
			}

			trajectory.addCartesianTrajectoryPoint(car_point);

		}

		if (trajectory.getTrajectorySize() > 1)	// 轨迹点大于1个，才认为是有效轨迹，进行保存
		{
			// 轨迹合理性判断
			if(Trajectory_Check(trajectory) == 1)
			{
				planning_ok_ = 1;								// 规划成功
				best_trajectory_ = trajectory;	// 得到了最佳轨迹

				ROS_INFO("Planning--Combine:the best traj is OK.the size is %d",trajectory.getTrajectorySize());

				for (uint32_t i = 0; i < trajectory.getTrajectorySize(); ++i)
				{
					FrenetPoint path_point = trajectory.getFrenetPointByIndex(i);

					ROS_INFO("Best Traj: s = %f, l = %f, heading = %f, v = %f.",path_point.getS(),path_point.getL(),trajectory.getCartesianPointByIndex(i).getTheta(),path_point.getdS());	
				}		

				return;			
			}
		}
		else
		{
			ROS_INFO("Planning--Combine:Traj combine faild ,the size is %d not enough to publish.",trajectory.getTrajectorySize());		
		}
	}

}


// 轨迹检测：包括有效性检测和碰撞检测				
bool LatticePlanner::Trajectory_Check(const Trajectory& trajectory)		
{
	//依次遍历轨迹上的每个点
	for (CartesianPoint point : trajectory.getCartesianTrajectory())	
	{
		// 检查轨迹上是否存在无效点
		double x = point.getX();
		if (std::isnan(x)||std::isinf(x))
		{
			ROS_INFO("Planning--Check:Traj is invalid : x is nan or inf.");
			return false;
		}

		double y = point.getY();
		if (std::isnan(y)||std::isinf(y))
		{
			ROS_INFO("Planning--Check:Traj is invalid : y is nan or inf.");
			return false;
		}

		double theta = point.getTheta();
		if (std::isnan(theta)||std::isinf(theta))
		{
			ROS_INFO("Planning--Check:Traj is invalid : theta is nan or inf.");
			return false;
		}

		double lon_v = point.getVel();
		if (std::isnan(lon_v)||std::isinf(lon_v))
		{
			ROS_INFO("Planning--Check:Traj is invalid : v is nan or inf.");
			return false;
		}

		double lon_a = point.getAcc();
		if (std::isnan(lon_a)||std::isinf(lon_a))
		{
			ROS_INFO("Planning--Check:Traj is invalid : a is nan or inf.");
			return false;
		}

		double kappa = point.getKappa();
		if (std::isnan(kappa)||std::isinf(kappa))
		{
			ROS_INFO("Planning--Check:Traj is invalid : kappa is nan or inf.");
			return false;
		}

		// 检查轨迹上的点是否都符合物理性能

		if (lon_v > VEL_LIMIT || lon_v < -EPSILON)
		{
			ROS_INFO("Planning--Check:Traj is invalid : v is out of the limit, v = %f", lon_v);
			return false;
		}

		if (lon_a < -DEC_LIMIT || lon_a > ACC_LIMIT)
		{
			ROS_INFO("Planning--Check:Traj is invalid : lon_a is out of the limit, a = %f", lon_a);
			return false;
		}

		if (kappa < -KAPPA_LIMIT || kappa > KAPPA_LIMIT)
		{
			ROS_INFO("Planning--Check:Traj is invalid : kappa is out of the limit, kappa = %f", kappa);
			return false;
		}

		double lat_a = point.getVel() * point.getVel() * point.getKappa();
		if (lat_a < -LAT_ACC_LIMIT || lat_a > LAT_ACC_LIMIT)
		{
			ROS_INFO("Planning--Check:Traj is invalid : lat_a is out of the limit, kappa = %f", lat_a);
			return false;
		}

		// 检查轨迹上的点是否符合碰撞条件

		// 计算轨迹上的车辆box信息
		double x0_temp = point.getX();
		double y0_temp = point.getY();
		double theta0_temp =  point.getTheta();
		// 通过车辆中心点坐标，计算车辆四个顶点的坐标
		double x_temp[4],y_temp[4];
		x_temp[0] = x0_temp + CAR_LENGTH/2*cos(theta0_temp) - CAR_WIDTH/2*sin(theta0_temp); 
		y_temp[0] = y0_temp + CAR_LENGTH/2*sin(theta0_temp) + CAR_WIDTH/2*cos(theta0_temp);
		x_temp[1] = x0_temp + CAR_LENGTH/2*cos(theta0_temp) + CAR_WIDTH/2*sin(theta0_temp); 
		y_temp[1] = y0_temp + CAR_LENGTH/2*sin(theta0_temp) - CAR_WIDTH/2*cos(theta0_temp); 
		x_temp[2] = x0_temp - CAR_LENGTH/2*cos(theta0_temp) + CAR_WIDTH/2*sin(theta0_temp); 
		y_temp[2] = y0_temp - CAR_LENGTH/2*sin(theta0_temp) - CAR_WIDTH/2*cos(theta0_temp); 
		x_temp[3] = x0_temp - CAR_LENGTH/2*cos(theta0_temp) - CAR_WIDTH/2*sin(theta0_temp); 
		y_temp[3] = y0_temp - CAR_LENGTH/2*sin(theta0_temp) + CAR_WIDTH/2*cos(theta0_temp); 

		// 构造车辆四个顶点的包络
		Box2d box_car_temp = Box2d(Vect(x_temp[0], y_temp[0]),Vect(x_temp[1], y_temp[1]),Vect(x_temp[2], y_temp[2]),Vect(x_temp[3], y_temp[3]));

		for (Obstacle obstacle : envi_info_.getObstacleVec())	// 遍历环境中的障碍物信息
		{
			// 获取障碍物的四顶点包络
			Box2d box_obs = obstacle.getBox2d();
			// 碰撞检测
			if (box_car_temp.HasOverlap(box_obs))
			{
				ROS_INFO("Planning--Check:Traj is invalid : Trajectory collide.");
				return false;
			}
		}
	}

	return true;
}	



bool LatticePlanner::isPlanningOK()
{
	return planning_ok_;
}


Trajectory LatticePlanner::getBestTrajectory()
{
	return best_trajectory_;
}


bool LatticePlanner::getStopFlag()
{
	return stop_flag_;
}

FrenetPoint LatticePlanner::getPlanningEndPoint()
{
	return point_end_;
}




}



