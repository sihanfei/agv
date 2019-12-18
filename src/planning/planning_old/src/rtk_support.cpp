#include "rtk_support.h"
#include <boost/thread/thread.hpp>

namespace planning
{

RTKSupport::RTKSupport(ros::NodeHandle &n) : n_(n)
{
  // planning模块由该构造函数启动，n为planning节点的NodeHandle对象，private_nh1，private_nh2接收调试数据
  //由决策类decision收集、处理话题数据，除调度命令外所有输入消息回调都封装decision类中，lattice_planner类做路径规划，decision类为lattice_planner类提供参数

  //接收感知topic
  perception_sub_ = n_.subscribe< perception_msgs::FusionDataInfo >(
      "/perception/obstacle_info", 10, &RTKSupport::perceptionCallback, this); // perception topic

  location_sub_ = n_.subscribe< location_msgs::FusionDataInfo >(
      "/localization/fusion_msg", 10, &RTKSupport::locationCallback, this); // perception topic

  //接收参考线topic
  ref_line_sub_ =
      n_.subscribe< map_msgs::REFPointArray >("/map/ref_point_info", 10, &RTKSupport::refLineCallback, this);

  //接收vcu
  vcu_sub_ = n_.subscribe< control_msgs::AGVStatus >("/drivers/com2agv/agv_status", 10, &RTKSupport::vcuCallback, this);

  //发布轨迹
  decision_pub_ = n_.advertise< plan_msgs::DecisionInfo >("/plan/decision_info", 10); // planning information to control

  //等待规划必要的输入
  while (n_.ok())
  {
    sleep(1);
    if (perception_sub_.getNumPublishers() == 0 || location_sub_.getNumPublishers() == 0)
    {
      ROS_ERROR("RTKSupport::Waiting for location and perception publisher...");
      // publishErrorStatusMsg("E0301001");
    }
    else
      break;
  }
  ROS_INFO("RTKSupport:Connected Location and Perception.");

  //创建执行任务的thread
  boost::thread start_planning_thrd(boost::bind(&RTKSupport::startRTKSupport, this));

  ROS_INFO("RTKSupport:RTK support start.");
}

// void Planning::statusPublisher()
//{
//  ros::Rate loop_rate(10);
//  while(n_.ok())
//  {
//    node_status_.header.stamp = ros::Time::now();

//    node_status_.node_name = node_file_name_;
//    node_status_.node_pid = node_pid_;

//    status_msgs::SafetyStatus safety_status;

//    safety_status.message_code = "I0301008";
//    safety_status.counter = counter_++;
//    safety_status.hardware_id = 0;

//    common_msgs::KeyValue planning_task_id_key_value;
//    planning_task_id_key_value.key = "Planning_Task_ID"; //1成功0失败
//    planning_task_id_key_value.valuetype = 2;
//    planning_task_id_key_value.value = planning_task_ID_;
//    safety_status.values.push_back(planning_task_id_key_value);

//    common_msgs::KeyValue planning_task_status_key_value;
//    planning_task_status_key_value.key = "Planning_Task_Status"; //1成功0失败a
//    planning_task_status_key_value.valuetype = 2;
//    planning_task_status_key_value.value = planning_task_status_;
//    safety_status.values.push_back(planning_task_status_key_value);

//    safety_status.value_num = safety_status.values.size();

//    node_status_.status.push_back(safety_status);
//    node_status_.state_num = node_status_.status.size();

//    node_status_pub_.publish(node_status_);
//    loop_rate.sleep();
//  }
//}

void RTKSupport::vcuCallback(const control_msgs::AGVStatus::ConstPtr &vcu_msg)
{
  uint32_t seq = vcu_msg->header.seq;

  if (vcu_seq_ + 1 != seq)
  {
    vcu_mismatch_count_++;
    if (vcu_mismatch_count_ > 5)
    {
      vcu_mismatch_count_ = 0;
      // ERROR
      ROS_ERROR("RTKSupport:Vcu sequence mismatch.");
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

void RTKSupport::refLineCallback(const map_msgs::REFPointArray::ConstPtr &ref_line_msg)
{
  ROS_INFO("RTKSupport:Ref line size=%d", ref_line_msg->REF_line_INFO.size());
  is_ref_line_read_finished_ = false;

  ref_line_.clearReferencePoints();

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

      // ROS_INFO("Test:ref_line(s=%f,x=%f,y=%f,theta=%f,kappa=%f,dkappa=%f,max_speed=%f,width_left=%f,width_right=%f)",
      //         ref_line_msg->REF_line_INFO[i].rs,ref_line_msg->REF_line_INFO[i].rx,ref_line_msg->REF_line_INFO[i].ry,
      //         angle2Radian(ref_line_msg->REF_line_INFO[i].rtheta),ref_line_msg->REF_line_INFO[i].rkappa,
      //         ref_line_msg->REF_line_INFO[i].rdkappa,
      //         ref_line_msg->REF_line_INFO[i].max_speed, ref_line_msg->REF_line_INFO[i].width_left,
      //         ref_line_msg->REF_line_INFO[i].width_right);

      ref_line_.addReferencePoint(ref_point);
    }
    s_last = ref_line_msg->REF_line_INFO[i].rs;
  }

  is_ref_line_read_finished_ = true;
}

void RTKSupport::perceptionCallback(const perception_msgs::FusionDataInfo::ConstPtr &perception_msg)
{
  // ROS_INFO("Test:I read a per msg.");
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

void RTKSupport::locationCallback(const location_msgs::FusionDataInfo::ConstPtr &location_msg)
{
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
      ROS_ERROR("RTKSupport:Locatiion sequence mismatch.");
      return;
    }
  }
  else
  {
    location_mismatch_count_ = 0;

    carlocation_.setX(location_msg->pose.x);
    carlocation_.setY(location_msg->pose.y);
    carlocation_.setTheta(angle2Radian(location_msg->yaw));
    carlocation_.setVel(sqrt(pow(location_msg->velocity.linear.x, 2) + pow(location_msg->velocity.linear.y, 2)));
    // carlocation_.k = location_msg->;
    // carlocation_.acc = location_msg->

    carlocation_.setKappa(0.0);
    carlocation_.setAcc(sqrt(pow(location_msg->accel.linear.x, 2) + pow(location_msg->accel.linear.y, 2)));

    last_carlocation_          = carlocation_;
    is_location_info_finished_ = true;
  }

  location_seq_ = seq;
}

void RTKSupport::startRTKSupport()
{
  ros::Rate loop_rate(10);
  // planning节点启动后，若感知与定位发布正常，则开启该线程，进行局部规划
  //待收到目标点后，规划任务开启
  //规划完毕后，若命令为cmd_type=1，则发布该轨迹，若cmd_type为0，则只计算不发布

  while (n_.ok())
  {
    //获取定位
    //坐标变换

    if (!is_location_info_finished_ || !is_perception_info_finished_ || !is_ref_line_read_finished_ ||
        !is_vcu_info_finished_)
      continue;

    // bool cartesianToFrenet(const CartesianPoint p0, const ReferenceLine& ref_line, FrenetPoint& frenet_point);
    FrenetPoint fre_location;
    if (!cartesianToFrenet(carlocation_, ref_line_, fre_location))
    {
      ROS_INFO("RTKSupport:Cartesian location transform to frenet location failed.");
    }

    double nearest_dis = std::numeric_limits< double >::max();

    for (Obstacle &obs : envi_info_.getObstacles())
    {
      SLBoundary sl_boundary = obs.getSLBoundary();

      double s_related_start_l = sl_boundary.getSRelatedStartL();
      double s_related_end_l   = sl_boundary.getSRelatedEndL();
      double l_related_start_s = sl_boundary.getLRelatedStartS();
      double l_related_end_s   = sl_boundary.getLRelatedEndS();

      double start_s = sl_boundary.getStartS();
      double end_s   = sl_boundary.getEndS();
      double start_l = sl_boundary.getStartL();
      double end_l   = sl_boundary.getEndL();

      ReferenceLinePoint startl_point = ref_line_.getReferenceLinePointByS(s_related_start_l);
      ReferenceLinePoint endl_point   = ref_line_.getReferenceLinePointByS(s_related_end_l);

      double startl_s_right_width = startl_point.getdWidthRight();
      double endl_s_left_width    = endl_point.getdWidthLeft();

      ROS_INFO("RTKSupport:This obstacle id = %d.", obs.getId());
      ROS_INFO("Test:(start_s,l_related_start_s)=(%f,%f),(end_s,l_related_end_s)=(%f,%f),(s_related_start_l,start_l)=(%"
               "f,%f),(s_related_end_l,end_l)=(%f,%f),startl_s_right_width=%f,endl_s_left_width=%f",
               start_s, l_related_start_s, end_s, l_related_end_s, s_related_start_l, start_l, s_related_end_l, end_l,
               startl_s_right_width, endl_s_left_width);

      if (start_s > ref_line_.getReferenceLinePointByIndex(ref_line_.getReferenceLinePointsSize() - 1).getS() ||
          end_s < fre_location.getS() || start_l > startl_s_right_width || end_l < -endl_s_left_width)
      {
        ROS_INFO("Test:out of ref line.");
        continue;
      }
      else if (-endl_s_left_width <= start_l && end_l <= startl_s_right_width)
      {
        if (nearest_dis > start_s - fre_location.getS() - CAR_LENGTH * 0.5 - SAFE_DISTANCE)
          nearest_dis = start_s - fre_location.getS() - CAR_LENGTH * 0.5 - SAFE_DISTANCE;

        ROS_INFO("Test:all obs in ref line.");
      }
      else
      {
        if (start_l < -endl_s_left_width && -endl_s_left_width <= l_related_start_s)
        {
          double s_min = start_s;

          if (nearest_dis > s_min - fre_location.getS() - CAR_LENGTH * 0.5 - SAFE_DISTANCE)
            nearest_dis = s_min - fre_location.getS() - CAR_LENGTH * 0.5 - SAFE_DISTANCE;
          ROS_INFO("Test:situation 1.");
        }

        if (l_related_start_s < -endl_s_left_width && -endl_s_left_width <= end_l)
        {
          double s_min = lerp(start_s, l_related_start_s, s_related_end_l, end_l, -endl_s_left_width);

          if (nearest_dis > s_min - fre_location.getS() - CAR_LENGTH * 0.5 - SAFE_DISTANCE)
            nearest_dis = s_min - fre_location.getS() - CAR_LENGTH * 0.5 - SAFE_DISTANCE;
          ROS_INFO("Test:situation 2.");
        }

        if (start_l < startl_s_right_width && startl_s_right_width <= l_related_start_s)
        {
          double s_min = lerp(s_related_start_l, start_l, start_s, l_related_start_s, startl_s_right_width);

          if (nearest_dis > s_min - fre_location.getS() - CAR_LENGTH * 0.5 - SAFE_DISTANCE)
            nearest_dis = s_min - fre_location.getS() - CAR_LENGTH * 0.5 - SAFE_DISTANCE;
          ROS_INFO("Test:situation 3.");
        }

        if (l_related_start_s < startl_s_right_width && startl_s_right_width <= end_l)
        {
          double s_min = start_s;

          if (nearest_dis > s_min - fre_location.getS() - CAR_LENGTH * 0.5 - SAFE_DISTANCE)
            nearest_dis = s_min - fre_location.getS() - CAR_LENGTH * 0.5 - SAFE_DISTANCE;

          ROS_INFO("Test:situation 4.");
        }
      }
    }

    if (nearest_dis < 10.0)
    {
      ROS_INFO("RTKSupport:One obstacle is too close.");
      publishEstopPlanningMsg();
    }
  }

  //获取感知
  //坐标变换

  // double coin = best_trajectory.CalculateCartesianTrajectoryCoincidence(last_best_trajectory_);
  // ROS_INFO("Planning:Coin = %f",coin);

  loop_rate.sleep();
}

// void RTKSupport::publishErrorStatusMsg(std::string number)
//{
//    //status_msgs::NodeStatus node_status;
//    //node_status.header.stamp = ros::Time::now();

//    //node_status.node_name = node_file_name_;
//    //node_status.node_pid = node_pid_;

//    status_msgs::SafetyStatus safety_status;
//    safety_status.message_code = number;
//    safety_status.counter = counter_++;
//    safety_status.hardware_id = 0;
//    safety_status.value_num = 0;

//    node_status_.status.push_back(safety_status);

//}

void RTKSupport::publishEstopPlanningMsg()
{
  plan_msgs::DecisionInfo decision_info;

  decision_info.CMD_estop       = 1;
  decision_info.CMD_gear        = 0;
  decision_info.CMD_hand_brake  = 0;
  decision_info.CMD_lift        = 0;
  decision_info.path_plan_valid = 0;
  decision_info.move_permit     = 1;
  decision_info.path_mode       = 2; // rtk

  ros::Rate loop_rate(100);

  while (n_.ok() && !vcu_info_.getEstopStatus())
  {
    decision_pub_.publish(decision_info);
    loop_rate.sleep();
  }
}

} // end namespace
