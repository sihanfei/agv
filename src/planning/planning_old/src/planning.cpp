#include "planning.h"

#include "math/cartesian_frenet_converter.h"
#include <boost/thread/thread.hpp>

namespace planning
{

Planning::Planning(ros::NodeHandle &n, ros::NodeHandle &private_nh1, ros::NodeHandle &private_nh2)
    : n_(n),
      // as_(NULL),
      lp_ptr_(NULL), decision_ptr_(NULL)
{
  // planning模块由该构造函数启动，n为planning节点的NodeHandle对象，private_nh1，private_nh2接收调试数据
  //由决策类decision收集、处理话题数据，除调度命令外所有输入消息回调都封装decision类中，lattice_planner类做路径规划，decision类为lattice_planner类提供参数

  // lattice_planner规划类指针
  lp_ptr_ = new LatticePlanner();

  // decision决策类指针
  decision_ptr_ = new Decision(n);

  //感知数据处理类，现在暂时用不到
  // per_env_ = new PerceptionEnvironment();

  //接收调度topic，选用重规划或实时规划逻辑
  vms_info_sub_ = n_.subscribe< hmi_msgs::VMSControlAD >("/map/vms_control_info", 10, &Planning::executeCb, this);

  //接收感知topic
  perception_sub_ = n_.subscribe< perception_msgs::FusionDataInfo >(
      "/perception/obstacle_info", 10, &Decision::perceptionCallback, decision_ptr_); // perception topic

  //接收定位topic
  location_sub_ = n_.subscribe< location_msgs::FusionDataInfo >(
      "/localization/fusion_msg", 10, &Decision::locationCallback, decision_ptr_); // location topic

  //接收控制topic
  control_sub_ = n_.subscribe< control_msgs::AGVRunningStatus >("/control/agv_running_status", 10,
                                                                &Decision::controlCallback, decision_ptr_);

  //接收参考线topic
  ref_line_sub_ =
      n_.subscribe< map_msgs::REFPointArray >("/map/ref_point_info", 10, &Decision::refLineCallback, decision_ptr_);

  //接收vcu
  vcu_sub_ =
      n_.subscribe< control_msgs::AGVStatus >("/drivers/com2agv/agv_status", 10, &Decision::vcuCallback, decision_ptr_);

  //发布轨迹
  decision_pub_ = n_.advertise< plan_msgs::DecisionInfo >("/plan/decision_info", 10); // planning information to control

  //发布车辆状态
  // adstatus_pub_ = n_.advertise< status_msgs::NodeStatus >("/planning/planning_status", 10); // ad status to hmi

  //发布节点状态给monitor
  node_status_pub_ = n_.advertise< status_msgs::NodeStatus >("/node/node_status", 10);

  // for 仿真debug
  rviz_path_decision_pub_ = n_.advertise< visualization_msgs::MarkerArray >("/monitor/rviz_path_decision_info", 10);

  //配置动态参数服务，仿真调试用 add by 林海 2019-5-22
  srv1_ = boost::make_shared< dynamic_reconfigure::Server< planning::params_1_Config > >(private_nh1);
  dynamic_reconfigure::Server< planning::params_1_Config >::CallbackType f1;
  f1 = boost::bind(&Planning::Paramscallback_1, this, _1, _2);
  srv1_->setCallback(f1);

  srv2_ = boost::make_shared< dynamic_reconfigure::Server< planning::params_2_Config > >(private_nh2);
  dynamic_reconfigure::Server< planning::params_2_Config >::CallbackType f2;
  f2 = boost::bind(&Planning::Paramscallback_2, this, _1, _2);
  srv2_->setCallback(f2);

  // map
  //调用map service，获取map，暂时不用
  // ros::ServiceClient client = n.serviceClient<>();
  // get a map from a service
  // client.call(srv);
  // map_ = srv.response.map;

  node_file_name_ = ros::this_node::getName();
  node_pid_       = getpid();

  planning_task_ID_     = "0";
  planning_task_status_ = "0";

  // planning_task_ID
  //决策提供：FSM需要
  //决策节点返回当前执行或者最后执行的任务ID。

  // planning_task_status
  //决策提供：FSM需要
  //决策节点返回当前执行或者最后执行的任务状态。
  // 0– 未执行
  // 1– 执行任务中
  // 2– 等待任务执行结果
  // 3– 任务结束，成功，离开当前状态
  // 4– 任务结束，失败，离开当前状态
  // 5– 任务结束，故障，切换到Exception
  // 6– 任务中止，按照命令
  // 7– 任务中止，自动判断

  counter_ = 1;

  //创建发布状态到monitor的thread
  // boost::thread status_publish_thrd(boost::bind(&Planning::statusPublisher,this));

  //等待规划必要的输入
  while (n_.ok())
  {
    sleep(1);
    if (perception_sub_.getNumPublishers() == 0 || location_sub_.getNumPublishers() == 0)
    {
      ROS_ERROR("Planning::Waiting for location and perception publisher...");
      publishErrorStatusMsg("E0301001");
    }
    else
      break;
  }
  ROS_INFO("Planning:Connected Location and Perception.");
  // boost::thread thrd(boost::bind(&Planning::publisherChecker,this));

  //  task_ID_        = 0;
  //  task_status_    = 0;
  //  running_status_ = 0;

  //开始任务标志位
  start_planning_flag_ = false;

  //任务类型
  cmd_type_ = 0;

  //正在执行任务退出标志位
  running_task_end_flag_ = true;

  // CartesianPoint(double x,double y,double theta,double kappa,double vel,double acc,double relative_time);
  last_cargoal_ = CartesianPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  //创建执行任务的thread
  boost::thread start_planning_thrd(boost::bind(&Planning::startPlanning, this));

  ROS_INFO("Planning:Planning start.");
}

Planning::~Planning()
{
  if (lp_ptr_ != NULL)
    delete lp_ptr_;

  if (decision_ptr_ != NULL)
    delete decision_ptr_;
}

void Planning::statusPublisher()
{
  ros::Rate loop_rate(10);
  while (n_.ok())
  {
    // node_status_
    /*
        node_status_.status.clear();

        node_status_.header.stamp = ros::Time::now();

        node_status_.node_name = node_file_name_;
        node_status_.node_pid = node_pid_;


        status_msgs::SafetyStatus safety_status;

        safety_status.message_code = "I0301008";
        safety_status.counter = counter_++;
        safety_status.hardware_id = 0;

        common_msgs::KeyValue planning_task_id_key_value;
        planning_task_id_key_value.key = "Planning_Task_ID"; //1成功0失败
        planning_task_id_key_value.valuetype = 2;
        planning_task_id_key_value.value = planning_task_ID_;
        safety_status.values.push_back(planning_task_id_key_value);


        common_msgs::KeyValue planning_task_status_key_value;
        planning_task_status_key_value.key = "Planning_Task_Status"; //1成功0失败a
        planning_task_status_key_value.valuetype = 2;
        planning_task_status_key_value.value = planning_task_status_;
        safety_status.values.push_back(planning_task_status_key_value);


        safety_status.value_num = safety_status.values.size();

        node_status_.status.push_back(safety_status);
        node_status_.state_num = node_status_.status.size();

    */
    node_status_pub_.publish(node_status_);
    loop_rate.sleep();
  }
}

void Planning::startPlanning()
{
  ros::Rate loop_rate(10);
  // planning节点启动后，若感知与定位发布正常，则开启该线程，进行局部规划
  //待收到目标点后，规划任务开启
  //规划完毕后，若命令为cmd_type=1，则发布该轨迹，若cmd_type为0，则只计算不发布

  while (n_.ok())
  {
    // ROS_INFO("Test:Big while...");
    // ROS_INFO("Test:start_planning_flag_=%d", start_planning_flag_);

    loop_rate.sleep();

    //等待目标点到达
    while (n_.ok() && !start_planning_flag_)
    {
      // ROS_INFO("waiting for start planning flag...");
      loop_rate.sleep();
    }

    //获取参考线，若获取状态值为0，则表示获取过程中发生计算错误，报错退出本次任务
    //若获取状态值为1，则表示该数据还没有完全获得到，跳出本次循环，待下次循环继续判断数据是否获得
    //关于状态值的判断准则后续代码中都一样

    planning_task_status_ = "1";

    // ROS_INFO("Test:11111...");
    ReferenceLine ref_line;
    uint8_t ref_line_state = decision_ptr_->getReferenceLine(ref_line);
    if (ref_line_state == 0)
    {
      ROS_ERROR("Planning:Get reference line failed.");
      publishEstopPlanningMsg();
      publishErrorStatusMsg("E0301002");

      planning_task_status_ = "5";
      start_planning_flag_  = false;

      continue;
    }

    if (ref_line_state == 1)
    {
      continue;
    }

    // ROS_INFO("Test:222222...");
    //将该最终目标点给决策类，决策类保存目标点的笛卡尔坐标，并计算frenet坐标，供下面使用，计算错误则报错
    if (!decision_ptr_->setGoal(cargoal_))
    {
      ROS_ERROR("Planning:Set goal failed.");
      publishEstopPlanningMsg();
      publishErrorStatusMsg("E0301003");

      planning_task_status_ = "5";

      start_planning_flag_ = false;
      continue;
    }

    //将该最终目标点的frenet坐标给规划类
    FrenetPoint fregoal = decision_ptr_->getFreGoal();
    ROS_INFO("Planning:fregoal(s=%f,ds=%f,dds=%f,l=%f,dl=%f,ddl=%f)", fregoal.getS(), fregoal.getdS(), fregoal.getddS(),
             fregoal.getL(), fregoal.getdL(), fregoal.getddL());
    lp_ptr_->setGoal(fregoal);

    //进入循环前给该值赋值false，在执行任务循环中，每个循环会判断一次该标志位，若为true则退出循环，即退出本次任务
    running_task_end_flag_ = false;

    while (n_.ok())
    {
      //若为true则退出循环，即退出本次任务
      if (running_task_end_flag_)
      {
        break;
      }

      //计时的
      clock_t start, end;
      double dur;
      start = clock();

      // ROS_INFO("Planning:fregoal(s,ds,dds,l,dl,ddl)=(%f,%f,%f,%f,%f,%f)",fregoal.getS(),fregoal.getdS(),fregoal.getddS(),fregoal.getL(),fregoal.getdL(),fregoal.getddL());

      CartesianPoint carpoint;
      decision_ptr_->getCarLocation(carpoint);
      ROS_INFO("Test:x=%f,y=%f,theta=%f,vel=%f,acc=%f,kappa=%f", carpoint.getX(), carpoint.getY(), carpoint.getTheta(),
               carpoint.getVel(), carpoint.getAcc(), carpoint.getKappa());

      //从决策中获取frenet定位
      FrenetPoint frelocation;
      uint8_t location_state = decision_ptr_->getFreLocation(frelocation);
      if (location_state == 0)
      {
        ROS_WARN("Planning:Get frenet location failed.");
        //publishEstopPlanningMsg();
        publishErrorStatusMsg("E0301004");

        planning_task_status_ = "5";

        start_planning_flag_ = false;
        // break;
        continue;
      }

      if (location_state == 1)
        continue;

      ROS_INFO("Planning:frelocation(s,ds,dds,l,dl,ddl)=(%f,%f,%f,%f,%f,%f)", frelocation.getS(), frelocation.getdS(),
               frelocation.getddS(), frelocation.getL(), frelocation.getdL(), frelocation.getddL());

      //规划前，先根据定位判断是否到达目标点,在frenet坐标系下判断,若到达，则退出规划的循环，本次规划结束
      if (lp_ptr_->isGoalReached(frelocation))
      // if (lp_ptr_->isGoalReached(frelocation))
      {
        ROS_INFO("Planning:Goal reached.");

        planning_task_status_ = "3";

        start_planning_flag_ = false;
        decision_ptr_->reset();
        break;
      }

      //从决策中获取控制中的车辆状态
      // ControlInfo::ControlInfo(uint8_t fault_status, int8_t running_status, double offset_y, double offset_heading,
      // double offset_speed)
      ControlInfo control(0.0, 0.0, 0.0, 0.0, 0.0);

      //      uint8_t control_state = decision_ptr_->getControl(control);
      //      if (control_state == 0)
      //      {
      //        ROS_ERROR("Planning:Car status is incorrect.");
      //        // publishEstopPlanningMsg();
      //        publishErrorStatusMsg("E0301005");

      //        planning_task_status_ = "5";

      //        start_planning_flag_ = false;
      //        break;
      //      }

      //      if (control_state == 1)
      //        continue;

      ROS_INFO("Planning:control(offsety,offsetspeed,offsetheading)=(%f,%f,%f)", control.getOffsetY(),
               control.getOffsetSpeed(), control.getOffsetHeading());

      //      uint8_t vcu_state = decision_ptr_->checkVCU();
      //      if (vcu_state == 0)
      //      {
      //        ROS_ERROR("Planning:Vcu status is incorrect.");
      //        // publishEstopPlanningMsg();
      //        //                publishErrorStatusMsg("E0301005");

      //        //                planning_task_status_ = "5";

      //        start_planning_flag_ = false;
      //        break;
      //      }

      //      if (vcu_state == 1)
      //        continue;

      Trajectory best_trajectory;

      //判断是否需要进行下一次规划
      if (lp_ptr_->isStartReplan(control, last_best_trajectory_))
      {
        ROS_INFO("Planning:Start replan.");

        //获取感知
        PerceptionInfo per_info;
        uint8_t per_state = decision_ptr_->getPerception(per_info);
        if (per_state == 1)
          continue;

        //做出决策
        decision deci = decision_ptr_->makeDecision(frelocation);
        if (deci == Emergency)
        {
          ROS_INFO("Planning:Emergency,estop.");
          publishEstopPlanningMsg();
          continue;
        }

        //从决策中获取规划起点，失败则急停
        FrenetPoint fre_start_plan_point;
        if (!decision_ptr_->getPlanningStartPoint(fre_start_plan_point, last_best_trajectory_))
        {
          ROS_ERROR("Planning:Get start point failed.");
          publishEstopPlanningMsg();
          publishErrorStatusMsg("E0301006");

          planning_task_status_ = "5";

          start_planning_flag_ = false;
          break;
        }

        //从决策中获取规划停止点，失败则急停
        FrenetPoint fre_stop_plan_point;
        if (deci == Stop)
        {
          ROS_INFO("Planning:Start stop plan.");
          if (!decision_ptr_->getPlanningStopPoint(fre_stop_plan_point))
          {
            ROS_ERROR("Planning:Get stop point failed.");
            publishEstopPlanningMsg();
            publishErrorStatusMsg("E0301007");

            planning_task_status_ = "5";

            start_planning_flag_ = false;
            break;
          }
        }

        ROS_INFO("Planning:Before "
                 "planning,fre_start_plan_point(s=%f,ds=%f,dds=%f,l=%f,dl=%f,ddl=%f),fre_stop_plan_point(s=%f,ds=%f,"
                 "dds=%f,l=%f,dl=%f,ddl=%f)",
                 fre_start_plan_point.getS(), fre_start_plan_point.getdS(), fre_start_plan_point.getddS(),
                 fre_start_plan_point.getL(), fre_start_plan_point.getdL(), fre_start_plan_point.getddL(),
                 fre_stop_plan_point.getS(), fre_stop_plan_point.getdS(), fre_stop_plan_point.getddS(),
                 fre_stop_plan_point.getL(), fre_stop_plan_point.getdL(), fre_stop_plan_point.getddL());

        //开始lattice
        // plan，将规划开始点，规划目标点（非停车规划则用默认值，计算中不用），参考线，感知，决策结果作为输入，计算最终路径

        if (lp_ptr_->planning(fre_start_plan_point, fre_stop_plan_point, ref_line, per_info, deci, best_trajectory))
        {
          ROS_INFO("Planning:Plannning success.");

          if (last_best_trajectory_.getTrajectorySize() == 0)
          {
            publishPlanningMsg(best_trajectory);
          }
          else
          {
            // double coin = best_trajectory.CalculateCartesianTrajectoryCoincidence(last_best_trajectory_);
            // ROS_INFO("Planning:Coin = %f",coin);

            if (cmd_type_ == 1)
            // if (cmd_type_ == 1 && coin > 2.0)
            // if (cmd_type_ == 0)
            {
              publishPlanningMsg(best_trajectory);
              /*
              if (USE_PUBLISH_ALL_DEBUG)
              {
                  //for debug============================add 2019-5-30
                  visualization_msgs::MarkerArray decision_markerArray_;
                  visualization_msgs::Marker decision_marker_;
                  //ROS_INFO("%d", msg->path_data_REF.size());
                  int marker_id_ = 1;
                  for (Trajectory traj :all_traj_for_show) {
                      for (CartesianPoint point:traj.getCartesianTrajectory())
                      {
                          decision_marker_.header.frame_id = "/odom";
                          decision_marker_.header.stamp = ros::Time::now();
                          decision_marker_.action = visualization_msgs::Marker::ADD;
                          decision_marker_.pose.orientation.w = 1.0;

                          decision_marker_.ns = "Decision_point";
                          decision_marker_.id = marker_id_;
                          decision_marker_.type = visualization_msgs::Marker::CUBE;
                          decision_marker_.scale.x = 0.1;
                          decision_marker_.scale.y = 0.1;
                          decision_marker_.scale.z = 0.1;
                          decision_marker_.color.r = 0.0;
                          decision_marker_.color.g = 0.0;
                          decision_marker_.color.b = 1.0;
                          decision_marker_.color.a = 1.0;
                          decision_marker_.pose.position.x = point.getX();
                          decision_marker_.pose.position.y = point.getY();
                          decision_marker_.pose.position.z = 0.5;

                          decision_markerArray_.markers.emplace_back(decision_marker_);
                          marker_id_++;
                      }
                  }
                          all_traj_for_show.clear();

                  rviz_path_decision_pub_.publish(decision_markerArray_);
                  //==========================================================
              }
              */
            }
            else
              ROS_INFO("Planning:Get the best trajectory,but don't publish it.");
          }
        }
        else
        {
          ROS_INFO("Planning:No path,estop");
          // publishEstopPlanningMsg();
          // publishErrorStatusMsg("E0301008");
        }
      }
      else
      {
        // ROS_INFO("Planning:No need to replan.");
      }

      loop_rate.sleep();

      //这里保证每个规划循环为100ms，若不足100ms，则等待至100ms，若超过，则按超过时间
      end = clock();
      dur = ( double )(end - start) / CLOCKS_PER_SEC * 1000;
      //            while (dur < 100)
      //            {
      //                end = clock();
      //                dur = (double)(end - start)/CLOCKS_PER_SEC*1000;
      //            }
      ROS_INFO("Planning:This loop time is %fms", ( double )(end - start) / CLOCKS_PER_SEC * 1000);
    }
  }
}

void Planning::executeCb(const hmi_msgs::VMSControlAD::ConstPtr &cmd_msg)
{
  ROS_INFO("Planning:I received a cmd.");

  // task_ID_ = cmd_msg->task_ID;

  //加一个对目标点的判断

  ROS_INFO("Planning:cargoal(x=%f,y=%f,heading=%f)", cmd_msg->target_position.x, cmd_msg->target_position.y,
           angle2Radian(cmd_msg->target_position.heading));
  cargoal_ = CartesianPoint(cmd_msg->target_position.x, cmd_msg->target_position.y,
                            angle2Radian(cmd_msg->target_position.heading), 0, 0, 0, 0);

  if (fabs(cargoal_.getX() - last_cargoal_.getX()) < EPSILON &&
      fabs(cargoal_.getY() - last_cargoal_.getY()) < EPSILON &&
      fabs(cargoal_.getTheta() - last_cargoal_.getTheta()) < EPSILON)
  {
    ROS_INFO("Planning:received a same goal,don't cancel current task.");
    cmd_type_         = cmd_msg->CMD_Type;
    planning_task_ID_ = std::to_string(cmd_msg->task_ID);
  }
  else
  {
    ROS_INFO("Planning:received a new goal,cancel current task and do new one.");
    cmd_type_         = cmd_msg->CMD_Type;
    planning_task_ID_ = std::to_string(cmd_msg->task_ID);

    start_planning_flag_   = true;
    running_task_end_flag_ = true;

    decision_ptr_->reset();
    last_cargoal_ = cargoal_;
  }
}

void Planning::publishEstopPlanningMsg()
{
  plan_msgs::DecisionInfo decision_info;

  decision_info.CMD_estop       = 1;
  decision_info.CMD_gear        = 0;
  decision_info.CMD_hand_brake  = 0;
  decision_info.CMD_lift        = 0;
  decision_info.path_plan_valid = 0;
  decision_info.move_permit     = 1;
  decision_info.path_mode       = 1; // lattice

  ros::Rate loop_rate(100);

  // while (n_.ok() && !decision_ptr_->getVCUInfo().getEstopStatus()) // in case , publish 10 times
  // {
  //   decision_pub_.publish(decision_info);
  //   loop_rate.sleep();
  // }
  uint32_t num = 10;
  while (n_.ok() && num > 0) // in case , publish 10 times
  {
    num--;
    decision_pub_.publish(decision_info);
    loop_rate.sleep();
  }
}

void Planning::publishPlanningMsg(const Trajectory &best_trajectory)
{
  plan_msgs::DecisionInfo decision_info;

  for (uint32_t i = 1; i < best_trajectory.getTrajectorySize(); ++i)
  // for (uint32_t i = 0; i < best_trajectory.getTrajectorySize(); ++i)
  {
    CartesianPoint path_point = best_trajectory.getCartesianPointByIndex(i);
    common_msgs::PathPoint path_data;
    path_data.x = path_point.getX();
    path_data.y = path_point.getY();

    path_data.theta = radian2Angle(path_point.getTheta());

    // path_data.w=
    path_data.v = path_point.getVel();

    decision_info.path_data_REF.emplace_back(path_data);

    last_best_trajectory_ = best_trajectory;
    ROS_INFO("Planning:Best traj(x,y,theta,v)=(%f,%f,%f,%f)", path_data.x, path_data.y, path_data.theta, path_data.v);
  }

  decision_info.CMD_estop       = 0;
  decision_info.CMD_gear        = 0;
  decision_info.CMD_hand_brake  = 0;
  decision_info.CMD_lift        = 0;
  decision_info.path_plan_valid = 1;
  decision_info.move_permit     = 1; // default
  decision_info.path_mode       = 1; // lattice

  decision_pub_.publish(decision_info);
}

void Planning::publishErrorStatusMsg(std::string number)
{
  // status_msgs::NodeStatus node_status;
  // node_status.header.stamp = ros::Time::now();

  // node_status.node_name = node_file_name_;
  // node_status.node_pid = node_pid_;
  /*
      status_msgs::SafetyStatus safety_status;
      safety_status.message_code = number;
      safety_status.counter = counter_++;
      safety_status.hardware_id = 0;
      safety_status.value_num = 0;

      node_status_.status.push_back(safety_status);
  */
}

// void Planning::publisherChecker()
//{
//  while (n_.ok())
//  {
//    if (perception_sub_.getNumPublishers()==0 || location_sub_.getNumPublishers()==0)
//    {
//      ROS_("Planning:No publisher!!!,stop and waiting publisher...");

//      //1. stop car
//      //2. maybe exit()?
//    }
//    usleep(100000);
//  }
//}

//第一组参数的回调函数 仿真调试用 add by 林海 2019-5-22
void Planning::Paramscallback_1(planning::params_1_Config &config, uint32_t level)
{
  // ROS_INFO("Reconfigure 1: %f %f %f ", config.WEIGHT_LON_OBJECTIVE, config.WEIGHT_LON_COMFORT,
  //         config.WEIGHT_LON_COLLISION);
  WEIGHT_TARGET_SPEED   = config.WEIGHT_TARGET_SPEED;   //速度权重
  WEIGHT_DIST_TRAVELLED = config.WEIGHT_DIST_TRAVELLED; //距离权重
  // const double WEIGHT_TIME_LEN = 5.0;
  // longitudinal jerk
  LONGITUDINAL_JERK_UPPER_BOUND = config.LONGITUDINAL_JERK_UPPER_BOUND;

  // longitudinal collision
  LON_COLLISION_COST_STD        = config.LON_COLLISION_COST_STD;
  LON_COLLISION_YIELD_BUFFER    = config.LON_COLLISION_COST_STD;
  LON_COLLISION_OVERTAKE_BUFFER = config.LON_COLLISION_COST_STD;
  // centripetal acceleration

  // the offset of the center line of the lane
  LAT_OFFSET_BOUND            = config.LON_COLLISION_COST_STD;
  WEIGHT_OPPOSITE_SIDE_OFFSET = config.WEIGHT_OPPOSITE_SIDE_OFFSET;
  WEIGHT_SAME_SIDE_OFFSET     = config.WEIGHT_SAME_SIDE_OFFSET;

  // lateral acceleration

  // weight of every cost
  WEIGHT_LON_OBJECTIVE            = config.WEIGHT_LON_OBJECTIVE;
  WEIGHT_LON_COMFORT              = config.WEIGHT_LON_OBJECTIVE;
  WEIGHT_LON_COLLISION            = config.WEIGHT_LON_COLLISION;
  WEIGHT_LAT_OFFSET               = config.WEIGHT_LAT_OFFSET;
  WEIGHT_LAT_COMFORT              = config.WEIGHT_LAT_COMFORT;
  WEIGHT_CENTRIPETAL_ACCELERATION = config.WEIGHT_CENTRIPETAL_ACCELERATION;
}
//第二组参数的回调函数 仿真调试用 add by 林海 2019-5-22
void Planning::Paramscallback_2(planning::params_2_Config &config, uint32_t level)
{
  // ROS_INFO("Reconfigure 2: %f %f %f ", config.WEIGHT_LAT_KJ, config.WEIGHT_LAT_KS, config.WEIGHT_LAT_KD);
  WEIGHT_LAT_JERK     = config.WEIGHT_LAT_JERK;     // 横向规划时，Jerk的权重系数
  WEIGHT_LAT_S_LEN    = config.WEIGHT_LAT_S_LEN;    // 横向规划时，S的权重系数
  WEIGHT_LAT_L_OFFSET = config.WEIGHT_LAT_L_OFFSET; // 横向规划时，D的权重系数

  WEIGHT_LON_JERK   = config.WEIGHT_LON_JERK;   // 纵向规划时，Jerk的权重系数
  WEIGHT_LON_T_SPEN = config.WEIGHT_LON_T_SPEN; // 纵向规划时，T的权重系数
  WEIGHT_LON_V_DIF  = config.WEIGHT_LON_V_DIF;  // 纵向规划时，D或V的权重系数

  WEIGHT_LON_STOP_DIS_TO_GOAL = config.WEIGHT_LON_STOP_DIS_TO_GOAL;
  WEIGHT_LON_STOP_END_V       = config.WEIGHT_LON_STOP_END_V;

  WEIGHT_TOTAL_LAT = config.WEIGHT_TOTAL_LAT; // 横向轨迹的权重系数
  WEIGHT_TOTAL_LON = config.WEIGHT_TOTAL_LON; // 纵向轨迹的权重系数
}

} // end namespace
