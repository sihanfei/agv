
//#include <ros/ros.h>
//#include <ros/console.h>

#include "common/utils.h"

#include "math/cartesian_frenet_converter.h"
#include "pt_graph/path_time_graph.h"

namespace planning
{

PathTimeGraph::PathTimeGraph(const std::vector< Obstacle > &obstacles, const ReferenceLine &ref_line,
                             const FrenetPoint &frelocation)
{
  path_range_.first  = frelocation.getS();              //最小值
  path_range_.second = ref_line.getReferenceLineMaxS(); //最大值

  time_range_.first  = 0.0;
  time_range_.second = PLAN_TIME;

  setupObstacles(obstacles, ref_line);
}

void PathTimeGraph::setupObstacles(const std::vector< Obstacle > &obstacles, const ReferenceLine &ref_line)
{
  for (const Obstacle &obstacle : obstacles)
  {
    if (obstacle.isStatic())
    {
      setStaticObstacle(obstacle, ref_line);
    }
    else
    {
      setDynamicObstacle(obstacle, ref_line);
    }
  }

  //将设置好的静态sl障碍物按照从下往上排序?
  // std::sort(static_obs_sl_boundaries_.begin(), static_obs_sl_boundaries_.end(),
  //         [](const SLBoundary& sl0, const SLBoundary& sl1){return sl0.starts() < sl1.starts();});

  //  PathTimeObstacle path_time_obstacle = path_time_obstacle_map_[1];
  //  path_time_obstacle.bottomRightPoint()

  // for (std::pair<unsigned int,PathTimeObstacle>& path_time_obstacle : path_time_obstacle_map_){

  for (auto &path_time_obstacle : path_time_obstacle_map_)
  {
    double s_upper = std::max(path_time_obstacle.second.getBottomRightPoint().getS(),
                              path_time_obstacle.second.getUpperRightPoint().getS());

    double s_lower = std::min(path_time_obstacle.second.getBottomLeftPoint().getS(),
                              path_time_obstacle.second.getUpperLeftPoint().getS());

    path_time_obstacle.second.setPathUpper(s_upper);
    path_time_obstacle.second.setPathLower(s_lower);

    double t_upper = std::max(path_time_obstacle.second.getBottomRightPoint().getTime(),
                              path_time_obstacle.second.getUpperRightPoint().getTime());

    double t_lower = std::min(path_time_obstacle.second.getBottomLeftPoint().getTime(),
                              path_time_obstacle.second.getUpperLeftPoint().getTime());

    path_time_obstacle.second.setTimeUpper(t_upper);
    path_time_obstacle.second.setTimeLower(t_lower);

    //    std::cout<<"setpathupper="<<s_upper<<std::endl;
    //    std::cout<<"setpathlower="<<s_lower<<std::endl;
    //    std::cout<<"settimeupper="<<t_upper<<std::endl;
    //    std::cout<<"settimeupper="<<t_lower<<std::endl;

    path_time_obstacles_.push_back(path_time_obstacle.second);
  }
}

void PathTimeGraph::setStaticObstacle(const Obstacle &obstacle, const ReferenceLine &ref_line)
{
  uint32_t obstacle_id = obstacle.getId(); //记录障碍物id

  //根据障碍物的顶点xy坐标，将障碍物向sl坐标转换，得到障碍物的sl范围
  SLBoundary sl_boundary = obstacle.getSLBoundary();

  //    if (!computeObstacleBoundary(obstacle.getAllCornerPoints(), ref_line, sl_boundary))
  //    {
  //        ROS_WARN("PT_graph: Static obstacle id=%d is not in current reference line.",obstacle_id);
  //        return;
  //    }

  if (sl_boundary.getSRelatedStartL() > ref_line.getReferenceLineMaxS() ||
      sl_boundary.getSRelatedEndL() > ref_line.getReferenceLineMaxS())
  {
    ROS_WARN("PathTimeGraph:This static obstacle out of ref line max s in get obstacle distance.");
    return;
  }

  ReferenceLinePoint startl_point = ref_line.getReferenceLinePointByS(sl_boundary.getSRelatedStartL());
  ReferenceLinePoint endl_point   = ref_line.getReferenceLinePointByS(sl_boundary.getSRelatedEndL());

  double startl_s_right_width = startl_point.getdWidthRight();
  double endl_s_left_width    = endl_point.getdWidthLeft();

  //障碍物sl图在当前规划的车道范围外
  if (sl_boundary.getStartS() > path_range_.second || sl_boundary.getEndS() < path_range_.first ||
      sl_boundary.getStartL() > startl_s_right_width || sl_boundary.getEndL() < -endl_s_left_width)
  {
    // ROS_DEBUG_STREAM("Obstacle [" << obstacle_id << "] is out of range.");
    return;
  }

  const double trajectory_time_length = PLAN_TIME; //（需要修改）ST图规划的时间长度
  path_time_obstacle_map_[obstacle_id].setObstacleId(obstacle_id);

  path_time_obstacle_map_[obstacle_id].setBottomLeftPoint(
      setPathTimePoint(obstacle_id, sl_boundary.getStartS(), 0.0)); //左下
  path_time_obstacle_map_[obstacle_id].setBottomRightPoint(setPathTimePoint(obstacle_id, sl_boundary.getStartS(),
                                                                            trajectory_time_length)); //右下
  path_time_obstacle_map_[obstacle_id].setUpperLeftPoint(
      setPathTimePoint(obstacle_id, sl_boundary.getEndS(), 0.0)); //左上
  path_time_obstacle_map_[obstacle_id].setUpperRightPoint(setPathTimePoint(obstacle_id, sl_boundary.getEndS(),
                                                                           trajectory_time_length)); //右上

  path_time_obstacle_map_[obstacle_id].setdS(sl_boundary.getdS());
  // static_obs_sl_boundaries_.push_back(std::move(sl_boundary));

  //    std::pair<uint32_t, Obstacle> obstacle_test(obstacle_id, obstacle);
  //    obstacle_map_.insert(obstacle_test);

  return;
}

void PathTimeGraph::setDynamicObstacle(const Obstacle &obstacle, const ReferenceLine &ref_line)
{
  double relative_time = time_range_.first;
  while (relative_time <= time_range_.second)
  {
    uint32_t obstacle_id = obstacle.getId();

    SLBoundary sl_boundary = obstacle.getSLBoundary();

    // std::cout<<"relative_time="<<relative_time<<std::endl;
    //        if (!computeObstacleBoundary(obstacle.getAllCornerPointsByTime(relative_time), ref_line, sl_boundary))
    //        {
    //            //std::cout << "break11111111..........." << std::endl;,
    //            ROS_WARN("Pt_graph: Dynamic obstacle id=%d is not in current reference line.",obstacle_id);
    //            break;
    //        }

    if (sl_boundary.getSRelatedStartL() > ref_line.getReferenceLineMaxS() ||
        sl_boundary.getSRelatedEndL() > ref_line.getReferenceLineMaxS())
    {
      ROS_WARN("PathTimeGraph:This dynamic obstacle out of ref line max s in get obstacle distance.");
      return;
    }

    ReferenceLinePoint startl_point = ref_line.getReferenceLinePointByS(sl_boundary.getSRelatedStartL());
    ReferenceLinePoint endl_point   = ref_line.getReferenceLinePointByS(sl_boundary.getSRelatedEndL());

    double startl_s_right_width = startl_point.getdWidthRight();
    double endl_s_left_width    = endl_point.getdWidthLeft();

    //        std::cout << "path_range_.second : " << path_range_.second << std::endl;
    //        std::cout << "path_range_.first : " << path_range_.first << std::endl;
    //        std::cout << "startl_s_right_width : " << startl_s_right_width << std::endl;
    //        std::cout << "endl_s_left_width : " << endl_s_left_width << std::endl;
    //        std::cout << "sl_boundary.getStartL() : " << sl_boundary.getStartL() << std::endl;
    //        std::cout << "sl_boundary.getEndL() : " << sl_boundary.getEndL() << std::endl;

    if (sl_boundary.getStartS() > path_range_.second || sl_boundary.getEndS() < path_range_.first ||
        sl_boundary.getStartL() > startl_s_right_width || sl_boundary.getEndL() < -endl_s_left_width)
    {
      // std::cout<<"relative_time="<<relative_time<<"out of width."<<std::endl;
      if (path_time_obstacle_map_.find(obstacle.getId()) != path_time_obstacle_map_.end())
      {
        // std::cout << "break22222..........." << std::endl;
        break; //障碍物的id在规划区域内出现过
      }
      else
      {
        relative_time += 1; //累加判断周期（未定）
        // std::cout << "continue..........." << std::endl;
        continue; //障碍物id未在规划区域出现过，周期累加再判断
      }
    }

    //若障碍物id在当前规划范围内，且未在规划区域内出现过，即出现新的障碍物，记录其id及左侧点状态
    if (path_time_obstacle_map_.find(obstacle_id) == path_time_obstacle_map_.end())
    {
      path_time_obstacle_map_[obstacle_id].setObstacleId(obstacle_id);
      path_time_obstacle_map_[obstacle_id].setBottomLeftPoint(
          setPathTimePoint(obstacle_id, sl_boundary.getStartS(), relative_time));
      path_time_obstacle_map_[obstacle_id].setUpperLeftPoint(
          setPathTimePoint(obstacle_id, sl_boundary.getEndS(), relative_time));
      path_time_obstacle_map_[obstacle_id].setdS(sl_boundary.getdS());
      // std::cout << "setBottomLeftPoint : " << sl_boundary.getStartS() << relative_time << std::endl;
      // std::cout << "setUpperLeftPoint : " << sl_boundary.getEndS() << relative_time << std::endl;
    }

    path_time_obstacle_map_[obstacle_id].setBottomRightPoint(
        setPathTimePoint(obstacle_id, sl_boundary.getStartS(), relative_time));
    path_time_obstacle_map_[obstacle_id].setUpperRightPoint(
        setPathTimePoint(obstacle_id, sl_boundary.getEndS(), relative_time));

    // std::cout << "setBottomRightPoint : " << sl_boundary.getStartS() <<",t="<< relative_time << std::endl;
    // std::cout << "setUpperRightPoint : " << sl_boundary.getEndS() <<",t="<< relative_time << std::endl;

    //        std::pair<uint32_t, Obstacle> obstacle_test(obstacle_id, obstacle);
    //        obstacle_map_.insert(obstacle_test);

    // obstacle_map_[obstacle->id()] = *obstacle;

    relative_time += 1; //再累加判断周期（未定）
  }
}

// bool PathTimeGraph::computeObstacleBoundary(const std::vector<CartesianPoint>& vertices,const ReferenceLine&
// ref_line,SLBoundary&  sl_boundary) const
//{
//    //将当前类型double可取的最大最小值存入s及l
//    double start_s(std::numeric_limits<double>::max());
//    double end_s(std::numeric_limits<double>::lowest());
//    double start_l(std::numeric_limits<double>::max());
//    double end_l(std::numeric_limits<double>::lowest());

//    double s_related_start_l;
//    double s_related_end_l;
//    double l_related_start_s;
//    double l_related_end_s;

//    double ds;

//    for (const auto& point : vertices)
//    {
//        FrenetPoint frenet_point;
//        //(需要修改)，将障碍物顶点笛卡尔坐标转换为frenet坐标

//        if (cartesianToFrenet(point, ref_line, frenet_point))
//        {
//            if (start_s > frenet_point.getS())
//            {
//                start_s = frenet_point.getS();
//                l_related_start_s = frenet_point.getL();
//            }

//            if (end_s < frenet_point.getS())
//            {
//                end_s = frenet_point.getS();
//                l_related_end_s = frenet_point.getL();
//            }

//            if (start_l > frenet_point.getL())
//            {
//                start_l = frenet_point.getL();
//                s_related_start_l = frenet_point.getS();
//            }

//            if (end_l < frenet_point.getL())
//            {
//                end_l = frenet_point.getL();
//                s_related_end_l = frenet_point.getS();
//            }

//            //start_l = std::fmin(start_l, frenet_point.getL());//t最小值
//            //end_l = std::fmax(end_l, frenet_point.getL());//t最大值
//        }
//        else
//        {
//            ROS_WARN("PT_graph:Obstacles transform to frenet failed.");
//            //std::cout << "error : no referenceline..." << std::endl;
//            return false;
//        }
//        ds = frenet_point.getdS();
//    }
//    //设置障碍物的sl范围
//    sl_boundary.setStartS(start_s);
//    sl_boundary.setEndS(end_s);
//    sl_boundary.setStartL(start_l);
//    sl_boundary.setEndL(end_l);

//    sl_boundary.setSRelatedStartL(s_related_start_l);
//    sl_boundary.setSRelatedEndL(s_related_end_l);
//    sl_boundary.setLRelatedStartS(l_related_start_s);//no use
//    sl_boundary.setLRelatedEndS(l_related_end_s);//no use

//    sl_boundary.setdS(ds);

//    return true;
//}

PathTimePoint PathTimeGraph::setPathTimePoint(const uint32_t obstacle_id, const double s, const double t)
{
  return PathTimePoint(obstacle_id, s, t);
}

std::vector< PathTimePoint > PathTimeGraph::getObstacleSurroundingPoints(uint32_t obstacle_id, double s_dist,
                                                                         double t_min_density) const
{
  std::vector< PathTimePoint > pt_pairs;

  if (path_time_obstacle_map_.find(obstacle_id) == path_time_obstacle_map_.end())
  {
    return pt_pairs;
  }
  const auto &pt_obstacle = path_time_obstacle_map_.at(obstacle_id);

  double s0 = 0.0;
  double s1 = 0.0;
  double t0 = 0.0;
  double t1 = 0.0;

  if (s_dist > 0.0)
  {
    s0 = pt_obstacle.getUpperLeftPoint().getS();
    s1 = pt_obstacle.getUpperRightPoint().getS();
    t0 = pt_obstacle.getUpperLeftPoint().getTime();
    t1 = pt_obstacle.getUpperRightPoint().getTime();
  }
  else
  {
    s0 = pt_obstacle.getBottomLeftPoint().getS();
    s1 = pt_obstacle.getBottomRightPoint().getS();
    t0 = pt_obstacle.getBottomLeftPoint().getTime();
    t1 = pt_obstacle.getBottomRightPoint().getTime();

    //    std::cout<<"s0="<<s0<<std::endl;
    //    std::cout<<"s1="<<s1<<std::endl;
    //    std::cout<<"t0="<<t0<<std::endl;
    //    std::cout<<"t1="<<t1<<std::endl;
  }

  const double time_gap = std::fabs(t1 - t0);

  //  std::cout<<"time_gap:t0="<<t0<<",t1="<<t1<<std::endl;

  if (time_gap < EPSILON)
  {
    return pt_pairs;
  }

  double t_num_sections = time_gap / t_min_density + t0;
  //  const double t_interval = time_gap / num_sections;

  //  std::cout<<"num_sections:"<<num_sections<<std::endl;

  for (double t = t0; t <= t_num_sections; t += t_min_density)
  {
    // double t = t_interval * i + t0;

    double s = lerp(s0, t0, s1, t1, t) + s_dist;

    // std::cout<<"s="<<s<<",t="<<t<<std::endl;
    PathTimePoint pt_point(obstacle_id, s, t);

    pt_pairs.push_back(std::move(pt_point));
  }

  return pt_pairs;
}

//(需要修改)跟车
void PathTimeGraph::queryFollowPathTimePoints(const uint32_t obstacle_id, std::vector< SamplePoint > *sample_points)
{
  double follow_distance     = 0.0;
  const double t_min_density = TIME_DENSITY;

  const double obstacle_speed = getPathTimeObstaclesMap().at(obstacle_id).getdS();

  if (obstacle_speed > OBSTACLE_HIGH_SPEED)
  { //高速障碍物
    follow_distance = HIGH_SPEED_FOLLOW_DISTANCE;
  }
  else if (obstacle_speed > EPSILON && obstacle_speed <= OBSTACLE_HIGH_SPEED)
  { //低速障碍物
    follow_distance = LOW_SPEED_FOLLOW_DISTANCE;
  }
  else if (obstacle_speed < EPSILON)
  { //静止障碍物
    follow_distance = GENERAL_FOLLOW_DISTANCE;
  }

  std::vector< PathTimePoint > follow_path_time_points =
      getObstacleSurroundingPoints(obstacle_id, -follow_distance, t_min_density);

  for (const PathTimePoint &path_time_point : follow_path_time_points)
  {
    double ds = getPathTimeObstaclesMap().find(obstacle_id)->second.getdS();

    SamplePoint sample_point(path_time_point.getS(), ds, 0.0, path_time_point.getTime());

    //    std::cout << "s : " << sample_point.getS() << std::endl;
    //    std::cout << "ds : " << sample_point.getdS() << std::endl;
    //    std::cout << "t : " << sample_point.getRelativeTime() << std::endl;

    sample_points->push_back(sample_point);
  }
}

//(需要修改)超车
void PathTimeGraph::queryOvertakePathTimePoints(const uint32_t obstacle_id, std::vector< SamplePoint > *sample_points)
{
  double follow_distance     = GENERAL_FOLLOW_DISTANCE;
  const double t_min_density = TIME_DENSITY;

  std::vector< PathTimePoint > follow_path_time_points =
      getObstacleSurroundingPoints(obstacle_id, follow_distance, t_min_density);

  for (const PathTimePoint &path_time_point : follow_path_time_points)
  {
    double ds = getPathTimeObstaclesMap().find(obstacle_id)->second.getdS();

    SamplePoint sample_point(path_time_point.getS(), ds, 0.0, path_time_point.getTime());

    sample_points->push_back(sample_point);
  }
}

//删除不合理点
void PathTimeGraph::handleSamplingPoints(std::vector< SamplePoint > &sample_points, const FrenetPoint &fre_point)
{
  const double veh_acc_max = ACC_LIMIT; //最大加速度5m/s2
  const double veh_dec_max = -DEC_LIMIT;
  const double veh_vel_max = VEL_LIMIT; //最大速度30km/h
  if (sample_points.size() == 0)
  {
    return;
  }
  std::vector< SamplePoint >::iterator it = sample_points.begin();

  for (; it != sample_points.end();)
  {
    SamplePoint sample_point = *it;
    // std::cout<<"sample_point.getS()="<<sample_point.getS()<<",sample_point.getTime()="<<sample_point.getRelativeTime()<<std::endl;
    //撒到的点在当前位置后面 s1 - s0 < 0
    if (sample_point.getS() - fre_point.getS() < 0)
    {
      it = sample_points.erase(it);
      continue;
    }
    //检查跟随的最大速度限制
    if (sample_point.getdS() > veh_vel_max)
    {
      sample_point.setdS(veh_vel_max);
    }

    const double t_acc_min = (veh_vel_max - fre_point.getdS()) / veh_acc_max;
    const double t_dec_min = (veh_vel_max - sample_point.getdS()) / veh_dec_max;

    //时间不足以加到最大速度情况
    if (sample_point.getRelativeTime() < t_acc_min + t_dec_min)
    {
      // const double t_acc_front = (sample_point.getdS() - fre_point.getdS() +
      // veh_acc_max*sample_point.getRelativeTime())/(2*veh_acc_max);//最大加速时间 t<0????????
      const double t_acc_front =
          (sample_point.getdS() - fre_point.getdS() + veh_acc_max * sample_point.getRelativeTime()) * 0.5 / veh_acc_max;
      const double v_max      = fre_point.getdS() + veh_acc_max * t_acc_front; //加到的最大速度
      const double t_dec_back = sample_point.getRelativeTime() - t_acc_front;  //减速时间
      //能达到的最大s
      // v0*t_acc + 0.5*a_max*t_acc2 + v_max*t_dec - 0.5*a_max*t_dec2 < s1 -s0
      if (fre_point.getdS() * t_acc_front + 0.5 * veh_acc_max * t_acc_front * t_acc_front + v_max * t_dec_back -
              0.5 * veh_dec_max * t_dec_back * t_dec_back <
          sample_point.getS() - fre_point.getS())
      {
        it = sample_points.erase(it);
        // std::cout<<"11111..."<<std::endl;
        continue;
      }
    }
    else
    {
      //时间足以加到最大速度再减速
      //能否同时满足到达状态（s，t，ds）及车辆限制（v_max，a_max)
      // v0*t_acc_min + 0.5*a_max*t_acc_min2 + v_max*t_dec_min - 0.5*a_max*t_dec_min2 + v_max*(t-t_acc_min-t_dec_min) <
      // s1 - s0
      if (fre_point.getdS() * t_acc_min + 0.5 * veh_acc_max * t_acc_min * t_acc_min + veh_vel_max * t_dec_min -
              0.5 * veh_dec_max * t_dec_min * t_dec_min +
              veh_vel_max * (sample_point.getRelativeTime() - t_acc_min - t_dec_min) <
          sample_point.getS() - fre_point.getS())
      {
        it = sample_points.erase(it);
        // std::cout<<"22222..."<<std::endl;
        continue;
      }
    }
    ++it;
  }

#if (0)

  for (auto &sample_point : sample_points)
  {
    //撒到的点在当前位置后面 s1 - s0 < 0
    if (sample_point.getS() - s_location < 0)
    {
      std::cout << "111" << std::endl;
      sample_points.erase(sample_points.begin() + i);
      continue;
    }
    //检查跟随的最大速度限制
    if (sample_point.getdS() > veh_vel_max)
    {
      sample_point.setdS(veh_vel_max);
      std::cout << "222" << std::endl;
    }

    const double t_acc_min = (veh_vel_max - fre_point.getdS()) / veh_acc_max;
    const double t_dec_min = (veh_vel_max - sample_point.getdS()) / veh_acc_max;

    //时间不足以加到最大速度情况
    if (sample_point.getRelativeTime() < t_acc_min + t_dec_min)
    {
      std::cout << "sample_point.getRelativeTime() : " << sample_point.getRelativeTime() << std::endl;
      // const double t_acc_front = (sample_point.getdS() - fre_point.getdS() +
      // veh_acc_max*sample_point.getRelativeTime())/(2*veh_acc_max);//最大加速时间 t<0????????
      const double t_acc_front =
          (sample_point.getdS() - fre_point.getdS() + veh_acc_max * sample_point.getRelativeTime()) * 0.5 / veh_acc_max;
      const double v_max      = fre_point.getdS() + veh_acc_max * t_acc_front; //加到的最大速度
      const double t_dec_back = sample_point.getRelativeTime() - t_acc_front;  //减速时间
      //能达到的最大s
      // v0*t_acc + 0.5*a_max*t_acc2 + v_max*t_dec - 0.5*a_max*t_dec2 < s1 -s0
      if (fre_point.getdS() * t_acc_front + 0.5 * veh_acc_max * t_acc_front * t_acc_front + v_max * t_dec_back -
              0.5 * veh_acc_max * t_dec_back * t_dec_back <
          sample_point.getS() - s_location)
      {
        std::cout << fre_point.getdS() * t_acc_front + 0.5 * veh_acc_max * t_acc_front * t_acc_front +
                         v_max * t_dec_back - 0.5 * veh_acc_max * t_dec_back * t_dec_back
                  << std::endl;
        std::cout << "333" << std::endl;
        sample_points.erase(sample_points.begin() + i);
        continue;
      }
    }
    else
    {
      std::cout << "sample_point.getRelativeTime() : " << sample_point.getRelativeTime() << std::endl;
      //时间足以加到最大速度再减速
      //能否同时满足到达状态（s，t，ds）及车辆限制（v_max，a_max)
      // v0*t_acc_min + 0.5*a_max*t_acc_min2 + v_max*t_dec_min - 0.5*a_max*t_dec_min2 + v_max*(t-t_acc_min-t_dec_min) <
      // s1 - s0
      if (fre_point.getdS() * t_acc_min + 0.5 * veh_acc_max * t_acc_min * t_acc_min + veh_vel_max * t_dec_min -
              0.5 * veh_acc_max * t_dec_min * t_dec_min +
              veh_vel_max * (sample_point.getRelativeTime() - t_acc_min - t_dec_min) <
          sample_point.getS() - s_location)
      {
        std::cout << "444" << std::endl;
        sample_points.erase(sample_points.begin() + i);
        continue;
      }
    }

    i++;
  }

#endif
}

//撒纵向点
std::vector< SamplePoint > PathTimeGraph::sampleLonEndConditionsForPathTimePoints(const FrenetPoint &fre_point)
// std::vector<SamplePoint> sampleLonEndConditionsForPathTimePoints(const PathTimeGraph& path_time_graph, const
// FrenetPoint& fre_point)
{
  std::vector< SamplePoint > sample_points;
  if (getPathTimeObstacles().size() == 0)
  {
    return sample_points;
  }

  double s_min             = getPathTimeObstacles()[0].getBottomLeftPoint().getS();
  uint32_t obstacle_id_min = getPathTimeObstacles()[0].getObstacleId();

  for (const PathTimeObstacle &path_time_obstacle : getPathTimeObstacles())
  {
    uint32_t obstacle_id = path_time_obstacle.getObstacleId();
    double s             = path_time_obstacle.getBottomLeftPoint().getS();
    if (s < s_min)
    {
      s_min           = s;
      obstacle_id_min = obstacle_id;
    }
  }

  // std::cout << "s_min" <<s_min<< std::endl;
  queryFollowPathTimePoints(obstacle_id_min, &sample_points);
  // queryOvertakePathTimePoints(path_time_graph, obstacle_id_min, &sample_points);

  handleSamplingPoints(sample_points, fre_point);
  return sample_points;
}

std::vector< std::pair< double, double > > PathTimeGraph::getPathBlockingIntervals(const double t)
{
  std::vector< std::pair< double, double > > intervals;

  if (t < time_range_.first || t > time_range_.second)
  {
    return intervals;
  }

  for (const PathTimeObstacle &pt_obstacle : path_time_obstacles_)
  {
    if (t > pt_obstacle.getTimeUpper() || t < pt_obstacle.getTimeLower())
    {
      continue;
    }

    double s_upper = lerp(pt_obstacle.getUpperLeftPoint().getS(), pt_obstacle.getUpperLeftPoint().getTime(),
                          pt_obstacle.getUpperRightPoint().getS(), pt_obstacle.getUpperRightPoint().getTime(), t);

    double s_lower = lerp(pt_obstacle.getBottomLeftPoint().getS(), pt_obstacle.getBottomLeftPoint().getTime(),
                          pt_obstacle.getBottomRightPoint().getS(), pt_obstacle.getBottomRightPoint().getTime(), t);

    intervals.emplace_back(s_lower, s_upper);
  }
  return intervals;
}

std::vector< std::vector< std::pair< double, double > > >
PathTimeGraph::getPathBlockingIntervals(const double t_start, const double t_end, const double t_resolution)
{
  std::vector< std::vector< std::pair< double, double > > > intervals;
  for (double t = t_start; t <= t_end; t += t_resolution)
  {
    intervals.emplace_back(getPathBlockingIntervals(t));
  }
  return intervals;
}

std::vector< PathTimeObstacle > PathTimeGraph::getPathTimeObstacles() const
{
  return path_time_obstacles_;
}

// std::unordered_map<unsigned int, Obstacle> PathTimeGraph::getObstacleMap() const
//{
//  return obstacle_map_;
//}

std::unordered_map< uint32_t, PathTimeObstacle > PathTimeGraph::getPathTimeObstaclesMap() const
{
  return path_time_obstacle_map_;
}

} // namespace planning
