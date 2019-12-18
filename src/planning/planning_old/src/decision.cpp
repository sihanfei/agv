#include "decision.h"

namespace planning
{

Decision::Decision(ros::NodeHandle &n) : n_(n)
{
  is_first_location_         = true;
  is_ref_line_read_finished_ = false;

  is_location_info_finished_   = false;
  is_perception_info_finished_ = false;
  is_control_info_finished_    = false;
  is_vcu_info_finished_        = false;

  stop_dis_ = 0.0;

  location_seq_   = 0;
  perception_seq_ = 0;
  control_seq_    = 0;
  cmd_seq_        = 0;
  vcu_seq_        = 0;

  location_mismatch_count_   = 0;
  perception_mismatch_count_ = 0;
  control_mismatch_count_    = 0;
  vcu_mismatch_count_        = 0;
}

void Decision::reset()
{
  is_ref_line_read_finished_ = false;

  stop_dis_ = 0.0;
}

//决策获取最终目标点
bool Decision::setGoal(CartesianPoint &goal)
{
  //根据参考线将CartesianPoint格式的goal转换为Frenet格式
  FrenetPoint fregoal;
  if (!cartesianToFrenet(goal, ref_line_, fregoal))
  {
    ROS_ERROR("Decision:Cartesian goal transform to Frenet failed.");
    return false;
  }

  if (fregoal.getS() + CAR_LENGTH * 0.5 > ref_line_.getReferenceLineMaxS())
  {
    ROS_ERROR("Decision:This goal will cause the car body exceed the reference line.");
    return false;
  }

  fregoal_ = fregoal;
  cargoal_ = goal;
  return true;
}

FrenetPoint Decision::getFreGoal() const
{
  return fregoal_;
}

CartesianPoint Decision::getCarGoal() const
{
  return cargoal_;
}

double Decision::getObstacleStopDistance(FrenetPoint &frelocation)
{
  double nearest_dis = std::numeric_limits< double >::max();

  for (Obstacle &obs : envi_info_.getObstacles())
  {
    //遍历所有的静态障碍物
    if (!obs.isStatic())
      continue;

    SLBoundary sl_boundary = obs.getSLBoundary();

    double s_related_start_l = sl_boundary.getSRelatedStartL();
    double s_related_end_l   = sl_boundary.getSRelatedEndL();
    double l_related_start_s = sl_boundary.getLRelatedStartS();
    double l_related_end_s   = sl_boundary.getLRelatedEndS();

    double start_s = sl_boundary.getStartS();
    double end_s   = sl_boundary.getEndS();
    double start_l = sl_boundary.getStartL();
    double end_l   = sl_boundary.getEndL();

    if (s_related_start_l > ref_line_.getReferenceLineMaxS() || s_related_end_l > ref_line_.getReferenceLineMaxS())
    {
      ROS_WARN("Decision:Obstacle out of ref line max s in get obstacle distance.");
      continue;
    }

    ReferenceLinePoint startl_point = ref_line_.getReferenceLinePointByS(s_related_start_l);
    ReferenceLinePoint endl_point   = ref_line_.getReferenceLinePointByS(s_related_end_l);

    double startl_s_right_width = startl_point.getdWidthRight();
    double endl_s_left_width    = endl_point.getdWidthLeft();

    ROS_INFO("Decision:This obstacle id = %d.", obs.getId());
    ROS_INFO("Test:(start_s,l_related_start_s)=(%f,%f),(end_s,l_related_end_s)=(%f,%f),(s_related_start_l,start_l)=(%f,"
             "%f),(s_related_end_l,end_l)=(%f,%f),startl_s_right_width=%f,endl_s_left_width=%f",
             start_s, l_related_start_s, end_s, l_related_end_s, s_related_start_l, start_l, s_related_end_l, end_l,
             startl_s_right_width, endl_s_left_width);

    if (start_s > ref_line_.getReferenceLinePointByIndex(ref_line_.getReferenceLinePointsSize() - 1).getS() ||
        end_s < frelocation.getS() || start_l > startl_s_right_width || end_l < -endl_s_left_width)
    {
      ROS_INFO("Test:out of ref line.");
      continue;
    }
    else if (-endl_s_left_width <= start_l && end_l <= startl_s_right_width)
    {
      if (nearest_dis > start_s - frelocation.getS() - CAR_LENGTH * 0.5 - SAFE_DISTANCE)
        nearest_dis = start_s - frelocation.getS() - CAR_LENGTH * 0.5 - SAFE_DISTANCE;

      ROS_INFO("Test:all obs in ref line.");
    }
    else
    {
      if (start_l < -endl_s_left_width && -endl_s_left_width <= l_related_start_s)
      {
        double s_min = start_s;

        if (nearest_dis > s_min - frelocation.getS() - CAR_LENGTH * 0.5 - SAFE_DISTANCE)
          nearest_dis = s_min - frelocation.getS() - CAR_LENGTH * 0.5 - SAFE_DISTANCE;
        ROS_INFO("Test:situation 1.");
      }

      if (l_related_start_s < -endl_s_left_width && -endl_s_left_width <= end_l)
      {
        double s_min = lerp(start_s, l_related_start_s, s_related_end_l, end_l, -endl_s_left_width);

        if (nearest_dis > s_min - frelocation.getS() - CAR_LENGTH * 0.5 - SAFE_DISTANCE)
          nearest_dis = s_min - frelocation.getS() - CAR_LENGTH * 0.5 - SAFE_DISTANCE;
        ROS_INFO("Test:situation 2.");
      }

      if (start_l < startl_s_right_width && startl_s_right_width <= l_related_start_s)
      {
        double s_min = lerp(s_related_start_l, start_l, start_s, l_related_start_s, startl_s_right_width);

        if (nearest_dis > s_min - frelocation.getS() - CAR_LENGTH * 0.5 - SAFE_DISTANCE)
          nearest_dis = s_min - frelocation.getS() - CAR_LENGTH * 0.5 - SAFE_DISTANCE;
        ROS_INFO("Test:situation 3.");
      }

      if (l_related_start_s < startl_s_right_width && startl_s_right_width <= end_l)
      {
        double s_min = start_s;

        if (nearest_dis > s_min - frelocation.getS() - CAR_LENGTH * 0.5 - SAFE_DISTANCE)
          nearest_dis = s_min - frelocation.getS() - CAR_LENGTH * 0.5 - SAFE_DISTANCE;

        ROS_INFO("Test:situation 4.");
      }
    }
  }

  return nearest_dis;
}

decision Decision::makeDecision(FrenetPoint &fre_location)
{
  decision deci;
  double v0 = fre_location.getdS();

  double fastest_stop_s = 0.5 * v0 * v0 / DEC_LIMIT;
  // double faster_stop_s = 0.5*v0*v0/(DEC_LIMIT*0.8);
  //这里计算停车距离，为什么不用COMFORT_DEC？假如速度为10m/s，COMFORT_DEC=0.3，停车距离大概150米，这超出了传感器的检测范围
  double comfort_stop_s = 0.5 * v0 * v0 / COMFORT_DEC;

  // 1.静止障碍物的停车
  double obs_stop_s = getObstacleStopDistance(fre_location);

  ROS_INFO("Decision:Obs stop dis=%f,fastest_stop_s=%f,faster_stop_s=%f", obs_stop_s, fastest_stop_s, comfort_stop_s);

  // if (obs_stop_s + PARKING_BUFFER < fastest_stop_s)

  // if (obs_stop_s < 15.0)
  // {
  //   deci = Emergency;
  //   ROS_INFO("Decision:Emergency Stopping for obstacle!");

  //   return deci;
  // }

  if (obs_stop_s < fastest_stop_s)
  {
    deci = Emergency;
    ROS_INFO("Decision:Emergency Stopping for obstacle!");

    return deci;
  }

  // 2.目标点停车
  // if s velocity of the current goal = 0 and distance from current goal < STOP_DISTANCE

  if (fregoal_.getS() - fre_location.getS() < fastest_stop_s && fregoal_.getS() - fre_location.getS() > 0.0)
  {
    deci = Emergency;
    ROS_INFO("Decision:Emergency Stopping for goal!");

    return deci;
  }

  // if (obs_stop_s + PARKING_BUFFER < comfort_stop_s)
  if (obs_stop_s < comfort_stop_s + 5.0)
  {
    deci      = Stop;
    stop_dis_ = obs_stop_s;

    ROS_INFO("Decision:Stopping plan for obstacle.");
  }

  if (fregoal_.getS() - fre_location.getS() < comfort_stop_s + 5.0 &&
      fregoal_.getS() - fre_location.getS() > 0.0) // now set STOP_DISTANCE 20
  {
    deci = Stop;
    if (fregoal_.getS() - fre_location.getS() < obs_stop_s)
    {
      stop_dis_ = fregoal_.getS() - fre_location.getS();
    }

    ROS_INFO("Decision:Stopping plan for goal.");
  }

  // 3.停车点停车

  // stop_dis_ = obs_stop_s;
  return deci;
}

bool Decision::getPlanningStopPoint(FrenetPoint &fre_planning_stop_point) const
{
  if (stop_dis_ <= 0.0)
  {
    ROS_ERROR("Decision:Stop distance is invalid,please make decision first.");
    return false;
  }

  FrenetPoint fre_point;
  if (!cartesianToFrenet(carlocation_, ref_line_, fre_point))
  {
    ROS_WARN("Decision:Cartesian location transform to Frenet failed in getPlanningStopPoint function.");
    return false;
  }

  fre_planning_stop_point = FrenetPoint(fre_point.getS() + stop_dis_, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  return true;
}

bool Decision::getPlanningStartPoint(FrenetPoint &fre_planning_start_point, Trajectory &last_trajectory) const
{
  if (last_trajectory.getTrajectorySize() == 0)
  {
    ROS_INFO("Decision:Last traj size is 0.");
    if (!cartesianToFrenet(carlocation_, ref_line_, fre_planning_start_point))
    {
      ROS_ERROR("Decision:Cartesian location transform to Frenet failed in getPlanningStartPoint function.");
      return false;
    }
  }
  else
  {
    // 若当前路径偏差和轨迹路径偏差较大，则选用当前车辆位置进行轨迹规划；
    if (!cartesianToFrenet(carlocation_, ref_line_, fre_planning_start_point))
    {
      ROS_ERROR("Decision:Cartesian location transform to Frenet failed.");
      return false;
    }
  }

  // if (last_trajectory.getTrajectorySize() == 0)
  // {
  //   ROS_INFO("Decision:Last traj size is 0.");
  //   if (!cartesianToFrenet(carlocation_, ref_line_, fre_planning_start_point))
  //   {
  //     ROS_ERROR("Decision:Cartesian location transform to Frenet failed in getPlanningStartPoint function.");
  //     return false;
  //   }
  // }

  // // 此处，需要增加对轨迹重规划的起点判断 20190522
  // // 1、若当前位置偏差和规划路径差别不大，则选用原轨迹上的点继续进行轨迹规划；
  // // 2、若当前路径偏差和轨迹路径偏差较大，则选用当前车辆位置进行轨迹规划；
  // // 3、若当前路径偏差和规划路径偏差很大，则触发故障，车辆急停。
  // else if (control_info_.getOffsetHeading() < 1 && control_info_.getOffsetSpeed() < 0.15 &&
  //          control_info_.getOffsetY() < 0.15)
  // {
  //   // 当前位置偏差和规划路径差别不大，则选用原轨迹上的点继续进行轨迹规划
  //   // 航行角偏差小于1°且 速度偏差小于0.15m/s 且 横向控制误差小于0.15m
  //   // 求 frelocation 在原轨迹上的映射点，用轨迹上求到的映射点作为 规划起点
  //   uint32_t dis_min_index = last_trajectory.getNearestCartesianPointIndex(carlocation_);

  //   CartesianPoint car_start_point = last_trajectory.getCartesianPointByIndex(dis_min_index);

  //   if (!cartesianToFrenet(car_start_point, ref_line_, fre_planning_start_point))
  //   {
  //     ROS_ERROR("Decision:Cartesian Start Point transform to Frenet failed.");
  //     return false;
  //   }
  // }
  // else if (control_info_.getOffsetHeading() > 5 || control_info_.getOffsetSpeed() > 0.5 ||
  //          control_info_.getOffsetY() > 0.5)
  // {
  //   // 偏差太大: 航行角偏差大于5°或 速度偏差大于0.5m/s 或 横向控制误差大于0.5m
  //   ROS_ERROR("Decision: Control Offset is too big to planning."); // 急停，给control下发急停信号
  //   return false;
  // }
  // else
  // {
  //   // 若当前路径偏差和轨迹路径偏差较大，则选用当前车辆位置进行轨迹规划；
  //   if (!cartesianToFrenet(carlocation_, ref_line_, fre_planning_start_point))
  //   {
  //     ROS_ERROR("Decision:Cartesian location transform to Frenet failed.");
  //     return false;
  //   }
  // }
  return true;
}

void Decision::refLineCallback(const map_msgs::REFPointArray::ConstPtr &ref_line_msg)
{
  ROS_INFO("ref line size=%d", ref_line_msg->REF_line_INFO.size());
  is_ref_line_read_finished_ = false;

  ref_line_.clearReferencePoints();
  ref_line_.clearMaxSpeedIntervalMap();

  double s_last = -100.0;

  for (uint32_t i = 0; i < ref_line_msg->REF_line_INFO.size(); ++i)
  {
    if (fabs(s_last - ref_line_msg->REF_line_INFO[i].rs) > EPSILON)
    {
      std::vector< LaneRange > lane_ranges;
      for (uint32_t j = 0; j < ref_line_msg->REF_line_INFO[i].lane_ranges.size(); ++j)
      {
        LaneRange lane_range;
        lane_range.left_boundary_  = ref_line_msg->REF_line_INFO[i].lane_ranges[j].left_boundary;
        lane_range.right_boundary_ = ref_line_msg->REF_line_INFO[i].lane_ranges[j].right_boundary;
        lane_ranges.emplace_back(lane_range);
      }
      // ReferenceLinePoint(double s,double x,double y,double theta,double kappa,double dkappa,double max_speed,double
      // width_left,double width_right);
      ReferenceLinePoint ref_point(
          ref_line_msg->REF_line_INFO[i].rs, ref_line_msg->REF_line_INFO[i].rx, ref_line_msg->REF_line_INFO[i].ry,
          angle2Radian(ref_line_msg->REF_line_INFO[i].rtheta), ref_line_msg->REF_line_INFO[i].rkappa,
          ref_line_msg->REF_line_INFO[i].rdkappa, ref_line_msg->REF_line_INFO[i].max_speed, lane_ranges);
      // ref_line_msg->REF_line_INFO[i].width_left,
      // ref_line_msg->REF_line_INFO[i].width_right);

      ROS_INFO("Test:ref_line(s=%f,x=%f,y=%f,theta=%f,kappa=%f,dkappa=%f,max_speed=%f,width_left=%f,width_right=%f)",
               ref_line_msg->REF_line_INFO[i].rs, ref_line_msg->REF_line_INFO[i].rx, ref_line_msg->REF_line_INFO[i].ry,
               angle2Radian(ref_line_msg->REF_line_INFO[i].rtheta), ref_line_msg->REF_line_INFO[i].rkappa,
               ref_line_msg->REF_line_INFO[i].rdkappa, ref_line_msg->REF_line_INFO[i].max_speed,
               ref_point.getdWidthLeft(), ref_point.getdWidthRight());

      ref_line_.addReferencePoint(ref_point);
    }
    s_last = ref_line_msg->REF_line_INFO[i].rs;
  }

  ref_line_.CalculateMaxSpeedInterval();

  is_ref_line_read_finished_ = true;
}

void Decision::perceptionCallback(const perception_msgs::FusionDataInfo::ConstPtr &perception_msg)
{
  ROS_INFO("Test:I read a per msg.");
  uint32_t seq = perception_msg->header.seq;

  if (perception_seq_ + 1 != seq)
  {
    perception_mismatch_count_++;
    if (perception_mismatch_count_ > 5)
    {
      perception_mismatch_count_ = 0;
      // ERROR
      ROS_ERROR("Decision:Perception sequence mismatch.");
      return;
    }
  }
  else
  {
    perception_mismatch_count_ = 0;
    // ROS_INFO("I received a perception_msg.");
    envi_info_.clearObstacles();

    for (common_msgs::ObstacleInfo obs_info : perception_msg->obstacles)
    {

      ROS_INFO("Test:peak1(x=%f,y=%f),peak2(x=%f,y=%f),peak3(x=%f,y=%f),peak4(x=%f,y=%f),vel=%f,heading=%f",
               obs_info.peak[0].x, obs_info.peak[0].y, obs_info.peak[1].x, obs_info.peak[1].y, obs_info.peak[2].x,
               obs_info.peak[2].y, obs_info.peak[3].x, obs_info.peak[3].y, obs_info.velocity,
               angle2Radian(obs_info.theta));
      Obstacle obstacle(
          obs_info.id, Eigen::Vector2d(obs_info.peak[0].x, obs_info.peak[0].y),
          Eigen::Vector2d(obs_info.peak[1].x, obs_info.peak[1].y),
          Eigen::Vector2d(obs_info.peak[2].x, obs_info.peak[2].y),
          Eigen::Vector2d(obs_info.peak[3].x, obs_info.peak[3].y),
          // Eigen::Vector2d(millimeter2Meter(obs_info.velocity.data)*cos(obs_info.theta.data*0.1),millimeter2Meter(obs_info.velocity.data)*sin(obs_info.theta.data*0.1))
          obs_info.velocity, angle2Radian(obs_info.theta));

      if (!is_ref_line_read_finished_ || !is_location_info_finished_)
      {
        ROS_WARN("Decision:Reference line or location is not ready,obstacle can't be transformed.");
        return;
      }

      if (obstacle.init(ref_line_))
        envi_info_.addObstacle(obstacle);
      else
        ROS_WARN("Decision:All this obstacle cornor transform failed,drop it.");
    }
    is_perception_info_finished_ = true;
  }
  perception_seq_ = seq;
}

void Decision::vcuCallback(const control_msgs::AGVStatus::ConstPtr &vcu_msg)
{
  uint32_t seq = vcu_msg->header.seq;

  if (vcu_seq_ + 1 != seq)
  {
    vcu_mismatch_count_++;
    if (vcu_mismatch_count_ > 5)
    {
      vcu_mismatch_count_ = 0;
      // ERROR
      ROS_ERROR("Decision:Vcu sequence mismatch.");
      return;
    }
  }
  else
  {
    vcu_mismatch_count_ = 0;

    // VCUStatus(uint8_t VEHMode,uint8_t VEHFlt,uint8_t SOC,bool HVStatus,bool estop_status,uint8_t lift_status,uint8_t
    // rolling_counter)
    vcu_info_ = VCUStatus(vcu_msg->VEHMode, vcu_msg->VEHFlt, vcu_msg->SOC, vcu_msg->HVStatus, vcu_msg->EStopStatus,
                          vcu_msg->LiftStatus, vcu_msg->Rolling_Counter);
    // vcu_info_ =
    // VCUStatus(vcu_msg->VEHMode,vcu_msg->VEHFlt,vcu_msg->SOC,vcu_msg->HVStatus,vcu_msg->LiftStatus,vcu_msg->Rolling_Counter);

    is_vcu_info_finished_ = true;
  }
  vcu_seq_ = seq;
}

void Decision::locationCallback(const location_msgs::FusionDataInfo::ConstPtr &location_msg)
{
  // ROS_INFO("Test:I read a per msg.");
  // ROS_INFO("I received a location_msg.");
  uint32_t seq = location_msg->header.seq;

  //判断序列号是否合理
  if (location_seq_ + 1 != seq)
  {
    location_mismatch_count_++;
    if (location_mismatch_count_ > 5)
    {
      // ERROR
      location_mismatch_count_ = 0;
      ROS_ERROR("Decision:Locatiion sequence mismatch.");
      return;
    }
  }
  else
  {
    location_mismatch_count_ = 0;
    /*
if (is_first_location_ == true)
{
  ROS_INFO("Location:This is first location flag.");
  carlocation_.setX(location_msg->pose.x);
  carlocation_.setY(location_msg->pose.y);
  carlocation_.setTheta(angle2Radian(location_msg->yaw));
  carlocation_.setVel(sqrt(pow(location_msg->velocity.linear.x, 2) + pow(location_msg->velocity.linear.y, 2)));
  // carlocation_.k = location_msg->;
  // carlocation_.acc = location_msg->

  carlocation_.setKappa(0.0);
  carlocation_.setAcc(sqrt(pow(location_msg->accel.linear.x, 2) + pow(location_msg->accel.linear.y, 2)));

  last_carlocation_ = carlocation_;
    is_first_location_ = false;
}
//增加位置跳变判断，若当前帧定位坐标与上一帧坐标距离大于5，则报错 2019-5-10
else//第一帧定位数据不做判断
{
  ROS_INFO("Location:location x=%f,y=%f,last_location
x=%f,y=%f",location_msg->pose.x,location_msg->pose.y,last_carlocation_.getX(),last_carlocation_.getY());
  if (sqrt(pow(location_msg->pose.x - last_carlocation_.getX(), 2) + pow(location_msg->pose.y -
last_carlocation_.getY(), 2)) > 5) //上一帧定位与当前定位距离大于5
  {
    ROS_ERROR("Decision:Location jump!!!");
    return;
  }
}

if (location_msg->pose.x < 0 && location_msg->pose.y < 0)
{
  ROS_ERROR("Decision:Location xy invalid!!!");
  return;
}

//车辆是否出车道,已添加 2019-5-28
ReferenceLine ref_line;
if (getReferenceLineOnce(ref_line))
{
  // CartesianPoint(double x,double y,double theta,double kappa,double vel,double acc,double relative_time);

  CartesianPoint
car_point(location_msg->pose.x,location_msg->pose.y,angle2Radian(location_msg->yaw),0.0,sqrt(pow(location_msg->velocity.linear.x,
2) + pow(location_msg->velocity.linear.y, 2)),
                           sqrt(pow(location_msg->accel.linear.x, 2) + pow(location_msg->accel.linear.y, 2)),0.0);

  std::vector<Vect> boundary = getBoundaryByCartesianLocation(car_point);

  std::vector<FrenetPoint> fre_points =
getFrenetPointsBoundary(boundary,car_point,ref_line_);//该方法是common/get_boundary_by_cartesian_location.h头文件中的静态方法，直接调用，返回四个该CartesianPoint的四个顶点的Frenet坐标

  if (fre_points.size() < 4)
  {
    ROS_WARN("Decision:Location foot print transform failed.");
  }
    else
    {
      for (FrenetPoint fre_point : fre_points)
    {
      ReferenceLinePoint ref_line_point =
ref_line.getReferenceLinePointByS(fre_point.getS());//由每个frenet顶点坐标获取其对应的参考点，并判断其是否出道路边界
      if (fre_point.getL() > ref_line_point.getdWidthRight() || fre_point.getL() < -ref_line_point.getdWidthLeft())
      {
        ROS_ERROR("Decision:Location out of road!");
        return;
      }
    }

    carlocation_.setX(location_msg->pose.x);
    carlocation_.setY(location_msg->pose.y);
    carlocation_.setTheta(angle2Radian(location_msg->yaw));
    carlocation_.setVel(sqrt(pow(location_msg->velocity.linear.x, 2) + pow(location_msg->velocity.linear.y, 2)));
    // carlocation_.k = location_msg->;
    // carlocation_.acc = location_msg->

    carlocation_.setKappa(0.0);
    carlocation_.setAcc(sqrt(pow(location_msg->accel.linear.x, 2) + pow(location_msg->accel.linear.y, 2)));

    last_carlocation_ = carlocation_;
    is_location_info_finished_ = true;
  }
}
else
 {
      carlocation_.setX(location_msg->pose.x);
    carlocation_.setY(location_msg->pose.y);
    carlocation_.setTheta(angle2Radian(location_msg->yaw));
    carlocation_.setVel(sqrt(pow(location_msg->velocity.linear.x, 2) + pow(location_msg->velocity.linear.y, 2)));
    // carlocation_.k = location_msg->;
    // carlocation_.acc = location_msg->

    carlocation_.setKappa(0.0);
    carlocation_.setAcc(sqrt(pow(location_msg->accel.linear.x, 2) + pow(location_msg->accel.linear.y, 2)));

    last_carlocation_ = carlocation_;
    is_location_info_finished_ = true;
}
*/

    carlocation_.setX(location_msg->pose.x);
    carlocation_.setY(location_msg->pose.y);
    carlocation_.setTheta(angle2Radian(location_msg->yaw));
    carlocation_.setVel(sqrt(pow(location_msg->velocity.linear.x, 2) + pow(location_msg->velocity.linear.y, 2)));
    // carlocation_.k = location_msg->;
    // carlocation_.acc = location_msg->

    carlocation_.setKappa(0.0);
    // carlocation_.setAcc(sqrt(pow(location_msg->accel.linear.x, 2) + pow(location_msg->accel.linear.y, 2)));
    carlocation_.setAcc(0.0);
    last_carlocation_          = carlocation_;
    is_location_info_finished_ = true;
  }

  location_seq_ = seq;
}

void Decision::controlCallback(const control_msgs::AGVRunningStatus::ConstPtr &control_msg)
{
  uint32_t seq = control_msg->header.seq;

  //判断序列号是否合理
  if (control_seq_ + 1 != seq)
  {
    control_mismatch_count_++;
    if (control_mismatch_count_ > 5)
    {
      // ERROR
      control_mismatch_count_ = 0;
      ROS_ERROR("Decision:Control sequence mismatch.");
      return;
    }
  }
  else
  {
    // ControlInfo(uint8_t fault_status,int8_t running_status,double offset_y, double offset_heading, double
    // offset_speed);
    control_info_ = ControlInfo(control_msg->fault_status, control_msg->running_status, control_msg->offset_y,
                                control_msg->offset_heading, control_msg->offset_speed);
    is_control_info_finished_ = true;
  }

  control_seq_ = seq;
}

// change in 2019-7-4,return bool -> return uint_8,
// 0 error,quit task
// 1 warn,no info,skip current circle
// 2 correct
uint8_t Decision::getPerception(PerceptionInfo &per_info) const
{
  if (!is_perception_info_finished_)
  {
    ROS_WARN("Decision:Perception info has not been received yet.");

    return 1;
  }

  per_info = envi_info_;

  return 2;
}

uint8_t Decision::getControl(ControlInfo &control_info) const
{
  if (!is_control_info_finished_)
  {
    ROS_WARN("Decision:Control info has not been received yet.");
    return 1;
  }

  if (control_info_.getFaultStatus() > 0)
  {
    ROS_ERROR("Decision::Car is in fault status.");
    return 0;
  }

  if (control_info_.getRunningStatus() < 0)
  {
    ROS_ERROR("Decision::Car is in unknown running status.");
    return 0;
  }

  control_info = control_info_;
  return 2;
}

uint8_t Decision::getReferenceLine(ReferenceLine &ref_line) const
{
  //    ros::Rate loop_rate(10);
  //    while (!is_ref_line_read_finished_ && n_.ok())
  //    {
  //        loop_rate.sleep();
  //    }

  if (!is_ref_line_read_finished_)
  {
    ROS_WARN("Decision:Reference line info has not been received yet.");
    return 1;
  }

  if (ref_line_.getReferenceLinePointsSize() == 0)
  {
    ROS_ERROR("Decision:Reference line size = 0.");
    return 0;
  }

  ref_line = ref_line_;
  return 2;
}

// bool Decision::getReferenceLineOnce(ReferenceLine& ref_line) const
//{
//    if (!is_ref_line_read_finished_)
//    {
//        ROS_WARN("Decision:Reference line info has not been received yet.");
//        return 1;
//    }

//    if (ref_line_.getReferenceLinePointsSize() == 0)
//    {
//        ROS_ERROR("Decision:Reference line size = 0.");
//        return 0;
//    }

//    ref_line = ref_line_;
//    return 2;
//}

uint8_t Decision::getCarLocation(CartesianPoint &car_point) const
{
  if (!is_location_info_finished_)
  {
    ROS_WARN("Decision:Location info has not been received yet.");
    return 1;
  }

  car_point = carlocation_;
  return 2;
}

uint8_t Decision::getFreLocation(FrenetPoint &fre_location) const
{
  if (!is_location_info_finished_)
  {
    ROS_WARN("Decision:Location info has not been received yet.");
    return 1;
  }

  if (!cartesianToFrenet(carlocation_, ref_line_, fre_location))
  {
    ROS_ERROR("Decision:Cartesian location transform to Frenet failed.");
    return 0;
  }
  // fre_location_ = fre_location;
  return 2;
}

uint8_t Decision::checkVCU()
{
  if (is_vcu_info_finished_)
  {
    ROS_WARN("Decision:VCU info has not been received yet.");
    return 1;
  }

  if (vcu_info_.getVEHMode() != 2) //车辆在非自动驾驶模式
  {
    ROS_ERROR("Decision:VCU VEH mode is %d.", vcu_info_.getVEHMode());
    return 0;
  }

  if (vcu_info_.getVEHFlt() != 0) //车辆存在故障
  {
    ROS_ERROR("Decision:VCU VEH flt is %d.", vcu_info_.getVEHFlt());
    return 0;
  }

  if (vcu_info_.getSOC() == 3) //电量不足25%
  {
    ROS_ERROR("Decision:VCU SOC is %d.", vcu_info_.getSOC());
    return 0;
  }

  if (vcu_info_.getHVStatus() == 0) //车辆未启动高压
  {
    ROS_ERROR("Decision:VCU HV status is %d.", vcu_info_.getHVStatus());
    return 0;
  }

  if (vcu_info_.getLiftStatus() != 2) //顶升未在下降状态
  {
    ROS_ERROR("Decision:VUC lift status is %d.", vcu_info_.getLiftStatus());
    return 0;
  }

  return 2;
}

VCUStatus Decision::getVCUInfo()
{
  return vcu_info_;
}
}
