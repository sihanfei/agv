#include "lattice/lattice_planner.h"

namespace planning
{

LatticePlanner::LatticePlanner()
{
  //控制偏差超过阈值的计数器
  offset_count_ = 0;
  //周期计数器，到达阈值则重规划
  replan_count_ = 100; // to start first plan in isStartReplan()
}

void LatticePlanner::setGoal(const FrenetPoint fregoal)
{
  goal_ = fregoal;
}

bool LatticePlanner::isGoalReached(const FrenetPoint &frelocation)
{
  // cmp location and goal
  // current goal is a stop point,judging by s distance and velocity

  //完全到达目标点的条件，速度，位置都到达指定范围
  if (fabs(goal_.getS() - frelocation.getS()) <= STOP_DISTANCE && frelocation.getdS() <= STOP_VEL &&
      fabs(goal_.getL() - frelocation.getL()) <= STOP_DISTANCE * 0.5)
  {
    return true;
  }

  // S值与速度到达目标点范围但L值未到达
  if (fabs(goal_.getS() - frelocation.getS()) <= STOP_DISTANCE && frelocation.getdS() <= STOP_VEL)
  {
    ROS_WARN("LatticePlanner:S distance reached goal but L error = %f.", fabs(goal_.getL() - frelocation.getL()));
    return true;
  }

  //只有S值越过目标点，L值与速度均未达到停车标准
  if (goal_.getS() - frelocation.getS() <= STOP_DISTANCE)
  {
    ROS_WARN("LatticePlanner:S distance reached goal but vel error = %f and L error = %f.", frelocation.getdS(),
             fabs(goal_.getL() - frelocation.getL()));
    return true;
  }

  return false;
}

bool LatticePlanner::isStartReplan(const ControlInfo control_info, const Trajectory &last_best_trajectory)
{
  bool detemined_stop_plan_flag = false;

  //上条轨迹的末点状态满足最终目标点需求，无需再规划新轨迹
  if (last_best_trajectory.getTrajectorySize() != 0)
  {
    FrenetPoint last_fre_point =
        last_best_trajectory.getFrenetPointByIndex(last_best_trajectory.getTrajectorySize() - 1);
    if (fabs(goal_.getS() - last_fre_point.getS()) <= STOP_DISTANCE &&
        fabs(last_fre_point.getdS() - goal_.getdS()) <= STOP_VEL)
    {
      ROS_INFO("LatticePlanner:Last trajectory can complete the stop task,no need to plan new one.");
      detemined_stop_plan_flag = true;
    }
  }

  //按照循环频率规划，目前设置值为5个循环，时间大概500ms
  if (!detemined_stop_plan_flag)
  {
    replan_count_++;
    if (replan_count_ >= REPLAN_COUNT_THRESHOLD)
    {
      offset_count_ = 0;
      replan_count_ = 0;
      ROS_INFO("LatticePlanner:It is time to replan.");
      return true;
    }
  }
  // ROS_INFO("offset_count_ = %d,replan_count_ = %d",offset_count_,replan_count_);

  //判断控制偏差是否超出阈值，若超出次数超过5次，则重规划
  if (control_info.getOffsetHeading() > OFFSET_HEADING_THRESHOLD ||
      control_info.getOffsetSpeed() > OFFSET_SPEED_THRESHOLD || control_info.getOffsetY() > OFFSET_Y_THRESHOLD)
  {
    offset_count_++;

    if (offset_count_ >= OFFSET_COUNT_THRESHOLD)
    {
      offset_count_ = 0;
      replan_count_ = 0;
      ROS_INFO("LatticePlanner:offset too large.");
      return true;
    }
  }
  else
  {
    offset_count_ = 0;
  }

  //判断上一条轨迹是否会碰撞，若会则重规划
  if (envi_info_.getObstaclesSize() == 0)
  {
    // ROS_INFO("isStartReplan:no obs!!!");
    return false;
  }
  else
  {
    double t = 0.0;
    for (uint32_t i = 0; i < last_best_trajectory.getTrajectorySize(); ++i)
    {
      FrenetPoint fre_point = last_best_trajectory.getFrenetPointByIndex(i);
      if (fre_point.getS() > current_location_.getS())
      {
        t = t + PLAN_TIME_RESOLUTION;

        CartesianPoint car_point = last_best_trajectory.getCartesianPointByIndex(i);
        // calculate the footprint of every traj point
        double car_left  = car_point.getX() - CAR_WIDTH / 2;
        double car_upper = car_point.getY() + CAR_LENGTH / 2;

        double car_right = car_point.getX() + CAR_WIDTH / 2;
        double car_lower = car_point.getY() - CAR_LENGTH / 2;

        // rectangle rotation formula according to center point
        // x2 = (x1 - x0) * cosa - (y1 - y0) * sina + x0
        // y2 = (y1 - y0) * cosa + (x1 - x0) * sina + y0

        double upper_left_corner_x = (car_left - car_point.getX()) * cos(car_point.getTheta()) -
                                     (car_upper - car_point.getY()) * sin(car_point.getTheta()) + car_point.getX();

        double upper_left_corner_y = (car_upper - car_point.getY()) * cos(car_point.getTheta()) +
                                     (car_left - car_point.getX()) * sin(car_point.getTheta()) + car_point.getY();

        double lower_left_corner_x = (car_left - car_point.getX()) * cos(car_point.getTheta()) -
                                     (car_lower - car_point.getY()) * sin(car_point.getTheta()) + car_point.getX();

        double lower_left_corner_y = (car_lower - car_point.getY()) * cos(car_point.getTheta()) +
                                     (car_left - car_point.getX()) * sin(car_point.getTheta()) + car_point.getY();

        double lower_right_corner_x = (car_right - car_point.getX()) * cos(car_point.getTheta()) -
                                      (car_lower - car_point.getY()) * sin(car_point.getTheta()) + car_point.getX();

        double lower_right_corner_y = (car_lower - car_point.getY()) * cos(car_point.getTheta()) +
                                      (car_right - car_point.getX()) * sin(car_point.getTheta()) + car_point.getY();

        double upper_right_corner_x = (car_right - car_point.getX()) * cos(car_point.getTheta()) -
                                      (car_upper - car_point.getY()) * sin(car_point.getTheta()) + car_point.getX();

        double upper_right_corner_y = (car_upper - car_point.getY()) * cos(car_point.getTheta()) +
                                      (car_right - car_point.getX()) * sin(car_point.getTheta()) + car_point.getY();

        for (Obstacle obstacle : envi_info_.getObstacles())
        {
          // calculate the foot print of every obstacle

          Box2d box1(Vect(upper_left_corner_x, upper_left_corner_y), Vect(lower_left_corner_x, lower_left_corner_y),
                     Vect(lower_right_corner_x, lower_right_corner_y),
                     Vect(upper_right_corner_x, upper_right_corner_y));

          Box2d box2;
          if (obstacle.isStatic())
          {
            box2 = Box2d(Vect(obstacle.getCorner1X(), obstacle.getCorner1Y()),
                         Vect(obstacle.getCorner2X(), obstacle.getCorner2Y()),
                         Vect(obstacle.getCorner3X(), obstacle.getCorner3Y()),
                         Vect(obstacle.getCorner4X(), obstacle.getCorner4Y()));
          }
          else
          {
            box2 = Box2d(Vect(obstacle.getCorner1XByTime(t), obstacle.getCorner1YByTime(t)),
                         Vect(obstacle.getCorner2XByTime(t), obstacle.getCorner2YByTime(t)),
                         Vect(obstacle.getCorner3XByTime(t), obstacle.getCorner3YByTime(t)),
                         Vect(obstacle.getCorner4XByTime(t), obstacle.getCorner4YByTime(t)));
          }

          if (box1.HasOverlap(box2))
          {
            ROS_INFO("LatticePlanner:Last trajectory would crash obstacle,need replan.");
            return true;
          }
        }
      }
    }
  }
  return false;
}

bool LatticePlanner::planning(FrenetPoint &fre_start_plan_point, FrenetPoint &fre_stop_plan_point,
                              ReferenceLine &ref_line, PerceptionInfo perception_info, decision deci,
                              Trajectory &best_trajectory)
{
  // specific planning logic in TrajectoryGenerator
  TrajectoryGenerator traj_gen(fre_start_plan_point, fre_stop_plan_point, perception_info, ref_line, deci);

  // get best trajectory
  return traj_gen.getBestTrajectory(best_trajectory);
}

// bool LatticePlanner::fastStopPlanning(ReferenceLine& ref_line,Trajectory& best_trajectory)
//{
//  double v0 = current_location_.getdS();

//  //CartesianPoint(double x,double y,double theta,double kappa,double vel,double acc,double relative_time)

//  double stop_s = 0.5*v0*v0/DEC_LIMIT + PARKING_BUFFER;
//  //FrenetPoint(double s,double ds,double dds,double l,double dl,double ddl,double relative_time);
//  FrenetPoint current_goal_for_stop(current_location_.getS()+stop_s,0.0,0.0,0.0,0.0,0.0,0.0);

//  TrajectoryGenerator traj_gen(current_location_,current_goal_for_stop,envi_info_,ref_line,Stop);

//  //get best trajectory
//  return traj_gen.getBestTrajectory(best_trajectory);
//}

// bool LatticePlanner::slowStopPlanning(ReferenceLine& ref_line,Trajectory& best_trajectory)
//{
//  //bool stop_flag = true;
//  double v0 = current_location_.getdS();

//  double stop_s = 0.5*v0*v0/COMFORT_DEC + PARKING_BUFFER;
//  FrenetPoint current_goal_for_stop(current_location_.getS()+stop_s,0.0,0.0,0.0,0.0,0.0,0.0);

//  TrajectoryGenerator traj_gen(current_location_,current_goal_for_stop,envi_info_,ref_line,Stop);

//  //get best trajectory
//  return traj_gen.getBestTrajectory(best_trajectory);
//}

} // end namespace
