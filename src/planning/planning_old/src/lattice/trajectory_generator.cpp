#include "lattice/trajectory_generator.h"

namespace planning
{

TrajectoryGenerator::TrajectoryGenerator(FrenetPoint &current_location, FrenetPoint &current_goal,
                                         PerceptionInfo &envi_info, ReferenceLine &ref_line, decision deci)
    : frelocation_(current_location), fregoal_(current_goal), envi_info_(envi_info), ref_line_(ref_line), deci_(deci)
{
  // st图对象
  pt_graph_ = new PathTimeGraph(envi_info.getObstacles(), ref_line, current_location);
}

TrajectoryGenerator::~TrajectoryGenerator()
{
  if (pt_graph_ != NULL)
    delete pt_graph_;
}

std::vector< std::shared_ptr< Curve1d > > TrajectoryGenerator::getLatTrajectories()
{
  std::vector< std::shared_ptr< Curve1d > > ls_vec;

  // Equal front space of ref_line_,get vec s1 and vec l1
  // calculate smax
  double v0 = frelocation_.getdS();
  // double vc = fregoal_.getdS();
  double vc    = VEL_LIMIT;
  double s_max = 0.0;
  double s_min = 0.0;

  double t_plan_max = PLAN_TIME;
  double t_plan_min = 1.0;

  if (v0 < vc)
  {
    double ta = (vc - v0) / ACC_LIMIT;
    if (ta < t_plan_max)
    {
      s_max = vc * (t_plan_max - ta) + 0.5 * (vc * vc - v0 * v0) / ACC_LIMIT;
    }
    else
    {
      s_max = 0.5 * (vc * vc - v0 * v0) / ACC_LIMIT;
    }
  }
  else
  {
    s_max = vc * t_plan_max;
  }

  //若为停车规划，则s_max的s值不能超过目标点
  if (deci_ == Stop)
  {
    s_max = std::min(s_max, fregoal_.getS() - frelocation_.getS());
  }

  double td = v0 / DEC_LIMIT;
  if (td > t_plan_min)
  {
    s_min = v0 * t_plan_min - 0.5 * DEC_LIMIT * t_plan_min * t_plan_min;
  }
  else
  {
    s_min = 0.5 * (v0 * v0) / DEC_LIMIT;
  }

  // ROS_INFO("TrajectoryGenerator:s_max = %f,s_min = %f",s_max,s_min);

  // ref_point_max_speed_ = std::numeric_limits<double>::max();

  //按照s值4等分撒点范围
  double s_interval = (s_max - s_min) / 4.0;

  // ROS_INFO("Test get lat:get ref line point.");
  // for (double s1 = 10.0; s1 <= 80.0; s1 = s1 * 2)
  for (double s1 = s_min; s1 < s_max + EPSILON; s1 += s_interval)
  {
    //按照s值获取参考点
    // ROS_INFO("Test:get ref line s1=%f,location s=%f", s1, frelocation_.getS());

    ReferenceLinePoint ref_line_point;
    if (s1 + frelocation_.getS() > ref_line_.getReferenceLineMaxS())
    {
      s1             = ref_line_.getReferenceLineMaxS() - frelocation_.getS();
      ref_line_point = ref_line_.getReferenceLinePointByS(s1 + frelocation_.getS());
    }
    else
      ref_line_point = ref_line_.getReferenceLinePointByS(s1 + frelocation_.getS());

    // ROS_INFO("Test:After get ref line s1=%f,location s=%f", s1, frelocation_.getS());
    double ls  = -(ref_line_point.getdWidthLeft() - 0.5 * CAR_WIDTH - SAFE_DISTANCE);
    double le  = ref_line_point.getdWidthRight() - 0.5 * CAR_WIDTH - SAFE_DISTANCE;
    double mid = 0.0;

    std::vector< double > l_sample;
    l_sample.emplace_back(ls);
    l_sample.emplace_back(mid);
    l_sample.emplace_back(le);

    //获取所有撒点处最小的速度，作为规划速度限制
    //        ref_point_max_speed_ = std::min(ref_point_max_speed_,ref_line_point.getMaxSpeed());

    // for (double l1 = -ref_line_point.getdWidthLeft(); l1 <= ref_line_point.getdWidthRight(); l1 += 0.25)
    for (double l1 = -0.5; l1 <= 0.5; l1 += 0.5)
    // for (double l1 : l_sample)
    {
      // ROS_INFO("TrajectoryGenerator:s1=%f,l1=%f",s1,l1);

      std::shared_ptr< Curve1d > ls_path_ptr(new LatticeTrajectory1d(std::shared_ptr< Curve1d >(
          new QuinticPolynomial(frelocation_.getL(), frelocation_.getdL(), frelocation_.getddL(), l1, 0.0, 0.0,
                                frelocation_.getS(), frelocation_.getS() + s1))));
      ls_path_ptr->setTrajInfo("type=ls,polynomial=5,approach=cruise,s1=" + std::to_string(s1) + ",l1=" +
                               std::to_string(l1)); // for debug
      ls_vec.emplace_back(ls_path_ptr);
    }
  }

  // ROS_INFO("Test get lat: after get ref line point.");

  //停车规划时额外规划一条能够到达目标点的sl路径
  if (deci_ == Stop)
  {
    std::shared_ptr< Curve1d > ls_path_ptr(new LatticeTrajectory1d(std::shared_ptr< Curve1d >(
        new QuinticPolynomial(frelocation_.getL(), frelocation_.getdL(), frelocation_.getddL(), fregoal_.getL(),
                              fregoal_.getdL(), fregoal_.getddL(), frelocation_.getS(), fregoal_.getS()))));
    ls_path_ptr->setTrajInfo("type=ls,polynomial=5,approach=stop,s1=" +
                             std::to_string(fregoal_.getS() - frelocation_.getS()) + ",l1=" +
                             std::to_string(fregoal_.getL())); // for debug
    ls_vec.emplace_back(ls_path_ptr);
  }

  return ls_vec;
}

std::vector< std::shared_ptr< Curve1d > > TrajectoryGenerator::getLonTrajectories()
{
  std::vector< std::shared_ptr< Curve1d > > st_vec;

  // for cruising
  for (double t1 = 1.0; t1 <= PLAN_TIME; t1 = t1 + 1.0)
  // double t1 = PLAN_TIME;
  {
    //巡航理论上能够到达的最大速度与最小速度
    double vel_min = frelocation_.getdS() - DEC_LIMIT * t1;
    double vel_max = frelocation_.getdS() + ACC_LIMIT * t1;

    //不能小于0，超过车辆能到达的最大速度
    double vel_sample_min = vel_min < 0.0 ? 0.0 : vel_min;
    double vel_sample_max = vel_max > VEL_LIMIT ? VEL_LIMIT : vel_max;

    ROS_INFO("Test:vel_min=%f,vel_max=%f",vel_sample_min,vel_sample_max);
    //最大速度结合道路限制速度
    // vel_sample_max = vel_sample_max > ref_point_max_speed_ ? ref_point_max_speed_ : vel_sample_max;

    double vel_interval = (vel_sample_max - vel_sample_min)/ VEL_SAMPLE_NUM;

    //对速度采样，撒点
    for (double st_v1 = vel_sample_min; st_v1 <= vel_sample_max; st_v1 += vel_interval)
    {
      // ROS_INFO("TrajectoryGenerator:st_v1=%f,t1=%f",st_v1,t1);
      // std::cout<<"st_v1:"<<st_v1<<"t1:"<<t1<<std::endl;

      std::shared_ptr< Curve1d > st_path_ptr(new LatticeTrajectory1d(std::shared_ptr< Curve1d >(new QuarticPolynomial(
          frelocation_.getS(), frelocation_.getdS(), frelocation_.getddS(), st_v1, 0.0, 0.0, t1))));
      st_path_ptr->setTrajInfo("type=st,polynomial=4,approach=cruise,t1=" + std::to_string(t1) + ",v1=" +
                               std::to_string(st_v1)); // for debug
      st_vec.emplace_back(st_path_ptr);
    }
  }

  // ROS_INFO("Test get lon:afrer get lon cruise sample.");
  // if ( deci_==Stop )

  // st图结果撒点，针对跟车，超车
  for (SamplePoint &sample_point : pt_graph_->sampleLonEndConditionsForPathTimePoints(frelocation_))
  {
    // ROS_INFO("sample s=%f,sample ds=%f,sample dds=%f,sample
    // t=%f",sample_point.getS(),sample_point.getdS(),sample_point.getddS(),sample_point.getRelativeTime());

    std::shared_ptr< Curve1d > st_path_ptr(new LatticeTrajectory1d(std::shared_ptr< Curve1d >(
        new QuinticPolynomial(frelocation_.getS(), frelocation_.getdS(), frelocation_.getddS(), sample_point.getS(),
                              sample_point.getdS(), sample_point.getddS(), 0.0, sample_point.getRelativeTime()))));
    st_path_ptr->setTrajInfo(
        "type=st,polynomial=5,approach=st graph,t1=" + std::to_string(sample_point.getRelativeTime()) + ",v1=" +
        std::to_string(sample_point.getdS()) + ",s1=" + std::to_string(sample_point.getS())); // for debug
    st_vec.emplace_back(st_path_ptr);
    // pt_graph_->sampleLonEndConditionsForPathTimePoints(frelocation_);
  }

  // ROS_INFO("Test get lon:afrer get lon st sample.");

  // for stopping
  if (deci_ == Stop)
  {
    for (double t1 = 1.0; t1 <= PLAN_TIME; t1 += 1.0)
    {
      std::shared_ptr< Curve1d > st_path_ptr(new LatticeTrajectory1d(std::shared_ptr< Curve1d >(
          new QuinticPolynomial(frelocation_.getS(), frelocation_.getdS(), frelocation_.getddS(), fregoal_.getS(),
                                fregoal_.getdS(), fregoal_.getddS(), 0.0, t1))));
      // std::shared_ptr<Curve1d> st_path(new
      // QuinticPolynomial(frelocation_.getS(),frelocation_.getdS(),frelocation_.getddS(),fregoal_.getS(),0,0,0,t1));
      st_path_ptr->setTrajInfo("type=st,polynomial=5,approach=stop,t1=" + std::to_string(t1) + ",v1=" +
                               std::to_string(fregoal_.getdS()) + ",s1=" +
                               std::to_string(fregoal_.getS() - frelocation_.getS())); // for debug
      st_vec.emplace_back(st_path_ptr);
    }
  }
  // ROS_INFO("Test get lon:afrer get lon stop sample.");

  return st_vec;
}

bool TrajectoryGenerator::combine(const std::vector< TrajectoryPair > &pair_vec, Trajectory &best_trajectory)
{
  // combine
  ROS_INFO("TrajectoryGenerator:traj num is %d", pair_vec.size());

  //  for (TrajectoryPair traj_pair:pair_vec)
  //  {
  //    ROS_INFO("traj st path t_max = %f,s_max =
  //    %f",traj_pair.getSTPath()->paramLength(),traj_pair.getSTPath()->evaluate(0,traj_pair.getSTPath()->paramMax()));
  //    ROS_INFO("traj ls path s_max = %f,l_max =
  //    %f",traj_pair.getLSPath()->paramLength(),traj_pair.getLSPath()->evaluate(0,traj_pair.getLSPath()->paramMax()));
  //    //ROS_INFO("traj vel_lon cost = %f",traj_pair.vel_cost_);
  //    ROS_INFO("next traj ==========>>");
  //  }

  /*
   if (USE_PUBLISH_ALL_DEBUG)
   {
      for (TrajectoryPair traj_pair:pair_vec)
      {

          ROS_INFO("111.");
          Curve1d* st_path_ptr = traj_pair.getSTPath();
       Curve1d* ls_path_ptr = traj_pair.getLSPath();

       Trajectory trajectory;


          for (double t=0.0;t<=PLAN_TIME;t=t+PLAN_TIME_RESOLUTION)
          {
                          //in one case, the sampling t is greater than the maximum time range of the generated st_path,
          //for the part whose time is longer than st_path, extrapolation is made according to the speed of the end
   point of st_path.

          double s = st_path_ptr->evaluate(0,t);

          //if s>ls_path,extrapolation is not necessary.
          if (s > ls_path_ptr->paramMax())
          {
              //ROS_INFO("s=%f,ls_path->paramMax=%f",s,ls_path->paramMax());
              break;
          }

          double dot_s = std::max(st_path_ptr->evaluate(1,t),EPSILON);

          double dot_dot_s = st_path_ptr->evaluate(2,t);

          double l = ls_path_ptr->evaluate(0,s);

          double dot_l = ls_path_ptr->evaluate(1,s);

          //double vel_l = dot_s * dot_l;

          double dot_dot_l = ls_path_ptr->evaluate(2,s);

          //double acc_l = dot_dot_l * dot_s *dot_s + dot_l * dot_dot_s;

          //FrenetPoint point(s,dot_s,dot_dot_s,l,vel_l,acc_l,t);

          FrenetPoint point(s,dot_s,dot_dot_s,l,dot_l,dot_dot_l,t);

          ROS_DEBUG("TrajectoryGenerator:this traj
   point(s,ds,dds,l,dl,ddl)=(%f,%f,%f,%f,%f,%f)",point.getS(),point.getdS(),point.getddS(),point.getL(),point.getdL(),point.getddL());

          trajectory.addFrenetTrajectoryPoint(point);
          }
          ROS_INFO("222.");
          for (FrenetPoint fre_point:trajectory.getFrenetTrajectory())
          //for (FrenetPoint fre_point:traj.traj)
      {
          CartesianPoint car_point;
          if(!frenetToCartesian(fre_point,ref_line_,car_point))
          {
              ROS_ERROR("TrajectoryGenerator:Frenet point transform to Cartesian failed.");
          }
          trajectory.addCartesianTrajectoryPoint(car_point);
      }

        //all_traj_for_show.push_back(trajectory);

      }
   }
    */

  std::map< double, double > max_speed_map = ref_line_.getMaxSpeedInterval();

  for (TrajectoryPair traj_pair : pair_vec)
  {

    Curve1d *st_path_ptr = traj_pair.getSTPath();
    Curve1d *ls_path_ptr = traj_pair.getLSPath();

    Trajectory trajectory;

    // equal time to generate trajectory points in ls and st
    // for (double t=0.0;t<=PLAN_TIME;t=t+PLAN_TIME_RESOLUTION)
    for (double t = 0.0; t <= PLAN_TIME; t = t + PLAN_TIME_RESOLUTION)
    {
      // in one case, the sampling t is greater than the maximum time range of the generated st_path,
      // for the part whose time is longer than st_path, extrapolation is made according to the speed of the end point
      // of st_path.

      double s = st_path_ptr->evaluate(0, t);

      // if s>ls_path,extrapolation is not necessary.
      if (s > ls_path_ptr->paramMax())
      {
        // ROS_INFO("s=%f,ls_path->paramMax=%f",s,ls_path->paramMax());
        break;
      }

      double dot_s = std::max(st_path_ptr->evaluate(1, t), EPSILON);

      double dot_dot_s = st_path_ptr->evaluate(2, t);

      double l = ls_path_ptr->evaluate(0, s);

      double dot_l = ls_path_ptr->evaluate(1, s);

      // double vel_l = dot_s * dot_l;

      double dot_dot_l = ls_path_ptr->evaluate(2, s);

      // double acc_l = dot_dot_l * dot_s *dot_s + dot_l * dot_dot_s;

      // FrenetPoint point(s,dot_s,dot_dot_s,l,vel_l,acc_l,t);

      FrenetPoint point(s, dot_s, dot_dot_s, l, dot_l, dot_dot_l, t);

      ROS_DEBUG("TrajectoryGenerator:This traj point(s,ds,dds,l,dl,ddl)=(%f,%f,%f,%f,%f,%f)", point.getS(),
                point.getdS(), point.getddS(), point.getL(), point.getdL(), point.getddL());

      trajectory.addFrenetTrajectoryPoint(point);

      // ROS_DEBUG("TrajectoryGenerator:traj num is %d, this traj
      // (s,ds,dds,l,dl,ddl)=(%f,%f,%f,%f,%f,%f)",i,point.getS(),point.getdS(),point.getddS(),point.getL(),point.getdL(),point.getddL());
    }

    trajectory.setCost(traj_pair.getCost());

    /*
           std::map< double,double >::iterator iter;
           for (iter = max_speed_map.begin();iter!=max_speed_map.end();iter++)
           {
               double s_start = iter->first;
               double vel_max  = iter->second;

              ROS_INFO("Test:s_start=%f,vel_max=%f",s_start,vel_max);
           }
           ROS_INFO("Test:===============>");


       for (FrenetPoint fre_point:trajectory.getFrenetTrajectory())
           //for (FrenetPoint fre_point:traj.traj)
       {
           std::map< double,double >::iterator iter;
           for (iter = max_speed_map.begin();iter!=max_speed_map.end();iter++)
           {
               double s_start = iter->first;
               double vel_max  = iter->second;

               if (fre_point.getS() >= s_start)
               {
                   if (fre_point.getdS() > vel_max)
                   {
                       ROS_INFO("TrajectoryGenerator:This traj not meeting the speed limit of refrence line.");
                       goto combine_start;
                   }
               }
           }
       }
*/

    for (FrenetPoint fre_point : trajectory.getFrenetTrajectory())
    // for (FrenetPoint fre_point:traj.traj)
    {
      CartesianPoint car_point;
      if (!frenetToCartesian(fre_point, ref_line_, car_point))
      {
        ROS_ERROR("TrajectoryGenerator:Frenet point transform to Cartesian failed.");
      }
      trajectory.addCartesianTrajectoryPoint(car_point);
    }

    // use TrajectoryChecker check every trajectory
    TrajectoryChecker traj_checker(envi_info_, ref_line_);

    // check if the trajectory hits an obstacle
    if (traj_checker.isTrajInvalid(trajectory))
    {
      ROS_INFO("TrajectoryGenerator:this traj invalid.");
    }
    else
    {
      if (traj_checker.isTrajOutOfRoad(trajectory))
      {
        // ROS_INFO("TrajectoryGenerator:this traj out of road.");
        ROS_INFO("TrajectoryGenerator:this traj out of road,but now we don't care.");
      }
      // else
      // {
      // check if the trajectory meets the physical parameters of the vehicle
      if (traj_checker.isTrajCollided(trajectory))
      {
        ROS_INFO("TrajectoryGenerator:this traj would crash.");
      }
      else
      {
        best_trajectory = trajectory;
        ROS_INFO("TrajectoryGenerator:best traj cost=%f", trajectory.getCost());
        st_path_ptr->displayTrajInfo();
        ls_path_ptr->displayTrajInfo();
        return true;
      }
      //      }
    }
  }

  return false;
}

bool TrajectoryGenerator::getBestTrajectory(Trajectory &best_trajectory)
{

  //  ROS_INFO("Test:This is get best traj fuction.");
  // get all ls and st paths
  std::vector< std::shared_ptr< Curve1d > > all_ls_paths = getLatTrajectories();

  // ROS_INFO("Test:After get lat traj.");
  std::vector< std::shared_ptr< Curve1d > > all_st_paths = getLonTrajectories();

  // ROS_INFO("Test:After get lon traj.");

  // for debug
  //  for (std::shared_ptr<Curve1d> ls_path:all_ls_paths)
  //  {
  //      ROS_INFO("TrajectoryGenerator:traj ls_path end point(s,l)=(%f,%f)",
  //      ls_path->paramMax(),ls_path->evaluate(0,ls_path->paramMax()));
  //  }

  //  for (std::shared_ptr<Curve1d> st_path:all_st_paths)
  //  {
  //      ROS_INFO("TrajectoryGenerator:traj st_path end point(t,s)=(%f,%f)",
  //      st_path->paramLength(),st_path->evaluate(0,st_path->paramLength()));
  //  }

  // speed planning,get ref vel here,for score path
  // std::vector<double> ref_vel =
  // GetRefSpeed(frelocation_.getdS(),ref_point_max_speed_,deci_,frelocation_.getS(),fregoal_.getS());

  // use TrajectoryScorer to score and sort all the trajectory pairs

  //这里保留两种评分函数的对象，如果USE_NEW_SCORE_METHOD为true，则使用新版评分，若为false，

  // 1.新版评分函数对象

  //新版评分函数对象需要将停车模式作为参数
  bool stop_flag;
  if (deci_ == Stop)
    stop_flag = true; // stop planning
  else
    stop_flag = false; // cursing

  TrajectoryScorer traj_scorer = TrajectoryScorer(all_st_paths, all_ls_paths, fregoal_, stop_flag);

  // ROS_INFO("Test:After traj score.");

  std::vector< TrajectoryPair > &traj_pair_vec = traj_scorer.scorePairs();

  // ROS_INFO("Test:After score traj pairs.");

  return combine(traj_pair_vec, best_trajectory); // if it has a best trajectory,return true,otherwise,return false.
}

// std::vector<double> TrajectoryGenerator::GetRefSpeed(double v0, double vcruise, decision deci, double s0, double s1)
//{
//  //参数说明：v0 当前车速;vcruise 巡航参考速度；stopflag 是否存在停车点；s1 停车点在frenet坐标系下的位置;s0
//  当前位置在frenet坐标下的位置

//  // 注意：以下参数需要从配置文件中读取
//  // ad: 车辆舒适减速度，需要从配置文件中读取；
//  // t_plan ：轨迹预测时间，一般设置为8s;
//  // t_cycle :程序控制周期，一般设置为0.1s;
//  // SpeedRef;// 规划速度数组
//  // vcreep:爬行速度
//  // dcreep:爬行距离

//  //为方便测试代码，此处先对变量进行赋值，
//  double aa = COMFORT_ACC; //舒适加速度，单位 -- m/s^2
//  double ad = COMFORT_DEC; //舒适减速度，单位 -- m/s^2
//  //double t_plan  = 9.0; //8s
//  double t_plan  = PLAN_TIME; //8s
//  double t_cycle = PLAN_TIME_RESOLUTION; //100ms
//  double admax = DEC_LIMIT;
//  double vcreep = CREEP_VEL;
//  double dcreep = CREEP_DIS;

//  std::vector<double> SpeedRef;

//  //ROS_INFO("TrajectoryGenerator:reference velocity:");

//  // 速度生成

//  //是否有停车点
//  //若不存在停车点，车辆按照巡航速度进行行走
//  if (!deci==Stop)
//  {
//    if (v0 < vcruise)
//    {
//      double ta = (vcruise-v0)/aa;

//      double t=0.0;

//      while(t<t_plan+EPSILON)
//      {
//        if(t<ta)
//        {
//          double vf = v0+aa*t;
//          if(vf>vcruise)
//            SpeedRef.emplace_back(vcruise);
//          else
//            SpeedRef.emplace_back(vf);
//        }
//        else
//          SpeedRef.emplace_back(vcruise);

//        t+=t_cycle;
//      }
//    }
//    else
//    {
//      if (v0 > vcruise)
//      {
//        double td=(v0-vcruise)/ad;

//        double t = 0.0;

//        while(t<t_plan+EPSILON)
//        {
//          if(t<td)
//          {
//            double vf = v0-ad*t;
//            if(vf<vcruise)
//              SpeedRef.emplace_back(vcruise);
//            else
//              SpeedRef.emplace_back(vf);
//          }
//          else
//            SpeedRef.emplace_back(vcruise);
//          t+=t_cycle;
//        }
//      }
//      else
//      {
//        double t=0.0;
//        while(t<t_plan+EPSILON)
//        {
//          SpeedRef.emplace_back(vcruise);
//          t+=t_cycle;
//        }

//      }
//    }
//  }

//  else//若存在停车点
//  {
//    double sr = s1 - s0;

//    if (sr <=0) //剩余距离sr>0 N
//    {
//      SpeedRef.emplace_back(0.0);
//    }
//    else //剩余距离sr>0 Y
//    {
//      if (sr > dcreep) //剩余距离sr>爬行距离dcreep Y
//      {
//        // 计算当前速度停车所需要的停止距离
//        //double dstop  = 0.5*v0*v0/ad;
//        double dstop  = 0.5*(v0*v0-vcreep*vcreep)/ad+dcreep;

//        if (sr > dstop) //支持平稳停车 Y
//        {
//          if (vcruise >= v0) //巡航速度是否大于当前速度 Y
//          {
//            double dis_acc_stop = 0.5*(vcruise*vcruise - v0*v0)/aa + 0.5*(vcruise*vcruise-vcreep*vcreep)/ad+dcreep;

//            //判断剩余距离是否支持先加速后停车
//            if (sr >= dis_acc_stop)
//            {
//              //计算加速时间ta和支持的巡航时间tc;
//              double ta = (vcruise - v0)/aa;
//              double tc = (sr - dis_acc_stop)/vcruise;
//              double td = (vcruise - vcreep)/ad;
//              double tcreep = dcreep/vcreep;

//              double t = 0.0;

//              while(t<t_plan+EPSILON)
//              {
//                double vf;
//                if(t<=ta)
//                {
//                  vf = v0+aa*t;
//                }
//                else if(t<=ta+tc)
//                {
//                  vf = vcruise;
//                }
//                else if(t<=ta+td+tc)
//                {
//                  vf = vcruise - ad*(t-ta-tc);
//                }
//                else if(t<=ta+tc+td+tcreep)
//                {
//                  vf = vcreep;
//                }
//                else
//                  vf = 0.0;
//                SpeedRef.emplace_back(vf);

//                t+=t_cycle;
//              }
//            }
//          else// 计算能支持的最大速度
//          {
//            double vm = sqrt((2*aa*ad*(sr-dcreep) + ad*v0*v0 + aa*vcreep*vcreep) / (aa+ad));

//            double ta = (vm - v0)/aa;
//            double td = (vm-vcreep)/ad;
//            double tcreep = dcreep/vcreep;

//            double t = 0.0;

//            while(t<t_plan+EPSILON)
//            {
//              double vf;
//              if(t<=ta)
//              {
//                vf = v0+aa*t;
//              }
//              else if(t<=ta+td)
//              {
//                vf = vm-ad*(t-ta);
//              }
//              else if(t<=ta+td+tcreep)
//              {
//                vf = vcreep;
//              }
//              else
//                vf = 0.0;
//              SpeedRef.emplace_back(vf);
//              t+=t_cycle;
//            }
//          }
//        }
//        else//巡航速度是否大于当前速度 N
//        {
//          double td1 = (v0 - vcruise)/ad;
//          double tc = (sr - dstop) / vcruise;
//          double td2 = (vcruise - vcreep)/ad;
//          double tcreep = dcreep/vcreep;

//          double t = 0.0;

//          while(t<t_plan+EPSILON)
//          {
//            double vf;
//            if (t<=td1)
//            {
//              vf = v0-ad*t;
//            }
//            else if(t<=td1+tc)
//            {
//              vf=vcruise;
//            }
//            else if(t<=td1+tc+td2)
//            {
//              vf = vcruise-ad*(t-td1-tc);
//            }
//            else if(t<=td1+tc+td2+tcreep)
//            {
//              vf = vcreep;
//            }
//            else
//            {
//              vf = 0.0;
//            }
//            SpeedRef.emplace_back(vf);
//            t+=t_cycle;
//          }
//        }
//      }
//      else //支持平稳停车 N(不支持平稳停车，需要立马停车)
//      {
//        // 计算当前距离停车所支持的最小减速度
//        double adn = 0.5*v0*v0/sr;
//        double tdn = v0/adn;

//        if(adn<admax)//adn<admax Y
//        {
//          double t=0.0;
//          while(t<t_plan+EPSILON)
//          {

//            double vf = v0 - adn*t;

//            if(vf<0)
//              SpeedRef.emplace_back(0.0);
//            else
//              SpeedRef.emplace_back(vf);
//            t+=t_cycle;
//          }
//        }
//        else
//          SpeedRef.emplace_back(0.0);
//        }
//      }
//      else //剩余距离sr>爬行距离dcreep N
//      {
//        SpeedRef.emplace_back(vcreep);
//      }
//    }
//  }

//    //ROS_INFO("TrajectoryGenerator:reference velocity:loop=%d,reference velocity
//    size=%d,value=%f,t=%f",index,SpeedRef.size(),SpeedRef[index],t);
//    for(int i=0;i<SpeedRef.size();++i)
//    {
//      double t = i*PLAN_TIME_RESOLUTION+PLAN_TIME_RESOLUTION;
//      std::cout<<SpeedRef[i]<<","<<t<<std::endl;
//    }

//  return SpeedRef;
//}

} // end namespace
