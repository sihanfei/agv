
#include "lattice/trajectory_scorer.h"

#include <omp.h>

namespace planning
{

bool pairSortCost(TrajectoryPair &p1, TrajectoryPair &p2)
{
  return p1.getCost() < p2.getCost();
}

TrajectoryScorer::TrajectoryScorer(std::vector< std::shared_ptr< Curve1d > > &all_st_paths,
                                   std::vector< std::shared_ptr< Curve1d > > &all_ls_paths, FrenetPoint &goal,
                                   bool stop_flag)
    : goal_(goal), stop_flag_(stop_flag)

// path_time_graph_(path_time_graph)
{
  // Pairing all the ls and st paths, and load them into the traj_pair_vec_

  // path_time_intervals_ = pt_graph->getPathBlockingIntervals(0,PLAN_TIME,PLAN_TIME_RESOLUTION);

  ROS_INFO("Test:all_st_paths size = %d", all_st_paths.size());
  ROS_INFO("Test:all_ls_paths size = %d", all_ls_paths.size());

  for (std::shared_ptr< Curve1d > st_path : all_st_paths)
  {
    // first,check every st path
    if (!IsLonValid(st_path))
    {
      ROS_INFO("TrajectoryScorer:this st_path t_max=%f,s_max=%f is not valid.", st_path->paramMax(), st_path->getx1());
      continue;
    }

    for (std::shared_ptr< Curve1d > ls_path : all_ls_paths)
    {
      // second,check every ls path
      if (!IsLatValid(ls_path))
      {
        ROS_INFO("TrajectoryScorer:this ls_path s_max=%f,l_max=%f is not valid.", ls_path->paramMax(),
                 ls_path->getx1());
        continue;
      }

      ls_path_.emplace_back(ls_path); // for ls_path

      TrajectoryPair traj_pair(ls_path.get(), st_path.get());

      traj_pair_vec_.emplace_back(traj_pair);
    }
  }

  // ROS_INFO("TrajectoryScorer:Score param
  // WEIGHT_LAT_JERK=%f,WEIGHT_LAT_S_LEN=%f,WEIGHT_LAT_L_OFFSET=%f,WEIGHT_LON_JERK=%f,WEIGHT_LON_T_SPEN=%f,WEIGHT_LON_V_DIF=%f,WEIGHT_LON_STOP_DIS_TO_GOAL=%f,WEIGHT_LON_STOP_END_V=%f,WEIGHT_TOTAL_LAT=%f,WEIGHT_TOTAL_LON=%f",
  // WEIGHT_LAT_JERK,WEIGHT_LAT_S_LEN,WEIGHT_LAT_L_OFFSET,
  // WEIGHT_LON_JERK,WEIGHT_LON_T_SPEN,WEIGHT_LON_V_DIF,WEIGHT_LON_STOP_DIS_TO_GOAL,WEIGHT_LON_STOP_END_V,
  // WEIGHT_TOTAL_LAT,WEIGHT_TOTAL_LON);
}

std::vector< TrajectoryPair > &TrajectoryScorer::scorePairs()
{
  //#pragma omp parallel for num_threads(4)

  for (uint32_t i = 0; i < traj_pair_vec_.size(); ++i)
  {
    TrajectoryPair &traj_pair = traj_pair_vec_[i];

    // ROS_INFO("TrajectoryScorer:traj pair st_path length =%f,st_path_dx1=%f,start point(t,s)=(%f,%f),end
    // point(t,s)=(%f,%f);ls_path length =%f,start point(s,l)=(%f,%f),end point(s,l)=(%f,%f)",
    //         traj_pair.getSTPath()->paramLength(),traj_pair.getSTPath()->getdx1(),traj_pair.getSTPath()->paramMin(),traj_pair.getSTPath()->getx0(),traj_pair.getSTPath()->paramMax(),traj_pair.getSTPath()->getx1(),
    //         traj_pair.getLSPath()->paramLength(),traj_pair.getLSPath()->paramMin(),traj_pair.getLSPath()->getx0(),traj_pair.getLSPath()->paramMax(),traj_pair.getLSPath()->getx1());

    traj_pair.setCost(calTrajectoryCostPair(traj_pair));
    // traj_pair.setCost(traj_pair.getCost() + calTrajectoryCostPair(*this));

    // ROS_INFO("TrajectoryScorer:this traj score = %f",traj_pair.getCost());
    // ROS_INFO("next...");
  }
  std::sort(traj_pair_vec_.begin(), traj_pair_vec_.end(), pairSortCost);
  // ROS_INFO("next.");
  return traj_pair_vec_;
}

// ST积分
double TrajectoryScorer::calJerkIntST(const TrajectoryPair &traj_pair)
{
  // 计算五次曲线的jrek积分[t0,t1]
  // a[6]是五次多项式的六个系数，t1为终止状态参数值，t0为初始状态参数值
  // p = a[0] + a[1]*t + a[2]*t^2 + a[3]*t^3 + a[4]*t^4 + a[5]*t^5;

  double jerkMin = traj_pair.getSTPath()->evaluateNormalization(4, traj_pair.getSTPath()->paramMin());
  double jerkMax = traj_pair.getSTPath()->evaluateNormalization(4, traj_pair.getSTPath()->paramMax());

  return jerkMax - jerkMin;
}

//纵向评分 巡航
double TrajectoryScorer::calTrajectoryCostLonCruise(const TrajectoryPair &traj_pair)
{
  // 计算纵向轨迹的损失值（巡航）
  // a[5]是四次多项式的五个系数，t1为终止状态t值（t的采样值），t0为初始状态t值，v1为终止状态v值（v方向撒点值），v0为初始状态v值
  double dx_cost  = 0.0;
  //double v_offset = traj_pair.getSTPath()->getdx1() - traj_pair.getSTPath()->getdx0();
  double v_offset = traj_pair.getSTPath()->getdx1() - VEL_LIMIT;

//  if (v_offset > 0.0)
//    dx_cost = 1.0 / (v_offset * v_offset + EPSILON + 1.0);
//  else
//    dx_cost = v_offset * v_offset + 1.0;
  dx_cost = v_offset*v_offset;

  double jerk_cost   = fabs(calJerkIntST(traj_pair));
  double length_cost = traj_pair.getSTPath()->paramLength();

  // ROS_INFO("TrajectoryScorer:Lon Cruise part 1=%f,part 2=%f,part 3=%f,total
  // cost=%f",WEIGHT_LON_JERK*jerk_cost,WEIGHT_LON_T_SPEN*length_cost,WEIGHT_LON_V_DIF *
  // dx_cost,WEIGHT_LON_JERK*jerk_cost + WEIGHT_LON_T_SPEN*length_cost + WEIGHT_LON_V_DIF * dx_cost);
  return WEIGHT_LON_JERK * jerk_cost + WEIGHT_LON_T_SPEN * length_cost + WEIGHT_LON_V_DIF * dx_cost;
}

//纵向评分 停车
double TrajectoryScorer::calTrajectoryCostLonStopingMerging(const TrajectoryPair &traj_pair)
{
  // 计算纵向轨迹的损失值（混行或停车）
  // a[6]是五次多项式的六个系数，t1为终止状态t值（t的采样值），t0为初始状态t值，s1为终止状态d值（S方向撒点值）,s0为初始状态s值
  double s_length = goal_.getS() - traj_pair.getSTPath()->getx1();
  double s_cost   = s_length * s_length;

  double dx_cost = traj_pair.getSTPath()->getdx1();

  double jerk_cost = fabs(calJerkIntST(traj_pair));
  // double length_cost = traj_pair.getSTPath()->paramLength();

  // ROS_INFO("TrajectoryScorer:s_length=%f,s_cost=%f,dx_cost=%f,jerk_cost=%f",s_length,s_cost,dx_cost,jerk_cost);

  // ROS_INFO("TrajectoryScorer:Lon Stoping Merging part 1=%f,part 2=%f,part 3=%f,total
  // cost=%f,s_length=%f,goal_s=%f,st_path_x1=%f,dx_cost=%f",WEIGHT_LON_JERK*jerk_cost,WEIGHT_LON_STOP_END_V*dx_cost,WEIGHT_LON_STOP_DIS_TO_GOAL*s_cost,WEIGHT_LON_JERK*jerk_cost
  // + WEIGHT_LON_STOP_END_V*dx_cost +
  // WEIGHT_LON_STOP_DIS_TO_GOAL*s_cost,s_length,goal_.getS(),traj_pair.getSTPath()->getx1(),dx_cost);
  return WEIGHT_LON_JERK * jerk_cost + WEIGHT_LON_STOP_END_V * dx_cost + WEIGHT_LON_STOP_DIS_TO_GOAL * s_cost;
}

// LS积分
double TrajectoryScorer::calJerkIntLS(const TrajectoryPair &traj_pair)
{
  // 计算五次曲线的jrek积分[t0,t1]
  // a[6]是五次多项式的六个系数，t1为终止状态参数值，t0为初始状态参数值
  // p = a[0] + a[1]*t + a[2]*t^2 + a[3]*t^3 + a[4]*t^4 + a[5]*t^5;

  double jerkMin = traj_pair.getLSPath()->evaluateNormalization(4, traj_pair.getLSPath()->paramMin());
  double jerkMax = traj_pair.getLSPath()->evaluateNormalization(4, traj_pair.getLSPath()->paramMax());

  return jerkMax - jerkMin;
}

//横向评分
double TrajectoryScorer::calTrajectoryCostLat(const TrajectoryPair &traj_pair)
{
  // 计算横向轨迹的损失值
  // a[6]是五次多项式的六个系数，s1为终止状态s值（S方向撒点值），s0为初始状态s值，l1为终止状态d值（L方向撒点值）
  double l1            = traj_pair.getLSPath()->getx1();
  double l_offset_cost = l1 * l1;

  double jerk_cost = fabs(calJerkIntLS(traj_pair));

  double length      = traj_pair.getLSPath()->paramLength();
  double length_cost = 1.0 / (length + EPSILON + 1.0);

  // ROS_INFO("TrajectoryScorer:Lat part 1=%f,part 2=%f,part 3=%f,total
  // cost=%f",WEIGHT_LAT_JERK*jerk_cost,WEIGHT_LAT_S_LEN*length_cost,WEIGHT_LAT_L_OFFSET*l_offset_cost,WEIGHT_LAT_JERK*jerk_cost
  // + WEIGHT_LAT_S_LEN*length_cost + WEIGHT_LAT_L_OFFSET*l_offset_cost);

  return WEIGHT_LAT_JERK * jerk_cost + WEIGHT_LAT_S_LEN * length_cost + WEIGHT_LAT_L_OFFSET * l_offset_cost;
}

//横纵向综合评分
double TrajectoryScorer::calTrajectoryCostPair(const TrajectoryPair &traj_pair)
{
  // 计算横向轨迹和纵向轨迹的损失值
  if (stop_flag_) // 若停车或者混行
  {
    return WEIGHT_TOTAL_LAT * calTrajectoryCostLat(traj_pair) +
           WEIGHT_TOTAL_LON * calTrajectoryCostLonStopingMerging(traj_pair);
  }
  else // 若巡航
  {
    return WEIGHT_TOTAL_LAT * calTrajectoryCostLat(traj_pair) +
           WEIGHT_TOTAL_LON * calTrajectoryCostLonCruise(traj_pair);
  }
}

bool TrajectoryScorer::IsLonValid(const std::shared_ptr< Curve1d > st_path)
{
  for (double sample = st_path->paramMin(); sample <= st_path->paramMax(); sample = sample + SCORE_TIME_RESOLUTION)
  {
    // ROS_INFO("Test:vel = %f,acc = %f", st_path->evaluate(1, sample), st_path->evaluate(2, sample));
    if (st_path->evaluate(1, sample) > VEL_LIMIT*1.5 || st_path->evaluate(1, sample) < -EPSILON)
    {
      ROS_INFO("Test:vel = %f", st_path->evaluate(1, sample));
      ROS_INFO("111.");
      return false;
    }

    if (st_path->evaluate(2, sample) > ACC_LIMIT || st_path->evaluate(2, sample) < -DEC_LIMIT)
    {
      ROS_INFO("Test:acc = %f", st_path->evaluate(2, sample));
      ROS_INFO("222.");
      return false;
    }
  }
  return true;
}

bool TrajectoryScorer::IsLatValid(const std::shared_ptr< Curve1d > ls_path)
{

  return true;
}

//横向评分
// double TrajectoryScorer::calTrajectoryCostLat(const std::shared_ptr<Curve1d>& ls_path)
//{
//  // 计算横向轨迹的损失值
//  // a[6]是五次多项式的六个系数，s1为终止状态s值（S方向撒点值），s0为初始状态s值，l1为终止状态d值（L方向撒点值）
//  const double l1 = ls_path->evaluate(0,ls_path->paramMax());
//  return WEIGHT_LAT_KJ*calJerkIntLS(ls_path) + WEIGHT_LAT_KS*ls_path->paramLength() + WEIGHT_LAT_KD*l1*l1;
//}

// 5次LS积分
// double TrajectoryScorer::calJerkIntLS(const std::shared_ptr<Curve1d>& ls_path)
//{
//  // 计算五次曲线的jrek积分[t0,t1]
//  //a[6]是五次多项式的六个系数，t1为终止状态参数值，t0为初始状态参数值
//  //p = a[0] + a[1]*t + a[2]*t^2 + a[3]*t^3 + a[4]*t^4 + a[5]*t^5;

//  double jerkMin = ls_path->evaluate(4, ls_path->paramMin());
//  double jerkMax = ls_path->evaluate(4, ls_path->paramMax());

//  return jerkMax - jerkMin;
//}

} // namespace planning
