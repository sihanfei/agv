#include "lattice/trajectory_checker.h"

namespace planning
{

TrajectoryChecker::TrajectoryChecker(PerceptionInfo &envi_info, ReferenceLine &ref_line)
    : envi_info_(envi_info), ref_line_(ref_line)
{
  // collision_ = new Collision();
}

TrajectoryChecker::~TrajectoryChecker()
{
  // if(collision_!=NULL)
  //  delete collision_;
}

bool TrajectoryChecker::isTrajInvalid(const Trajectory &trajectory)
{
  for (CartesianPoint point : trajectory.getCartesianTrajectory())
  {
    double x = point.getX();
    if (!std::isnormal(x))
    {
      ROS_INFO("Test:x not normal.");
      return true;
    }

    double y = point.getY();
    if (!std::isnormal(y))
    {
      ROS_INFO("Test:y not normal.");
      return true;
    }

    double theta = point.getTheta();
    if (!std::isnormal(theta))
    {
      ROS_INFO("Test:theta not normal.");
      return true;
    }

    double lon_v = point.getVel();
    if (!std::isnormal(lon_v))
    {
      ROS_INFO("Test:lon v not normal.");
      return true;
    }

    if (lon_v > VEL_LIMIT*1.5 || lon_v < 0.0)
    {
      ROS_INFO("Test:lon v is error,lon_v=%f", lon_v);
      return true;
    }

    double lon_a = point.getAcc();
    if (!std::isnormal(lon_a))
    {
      ROS_INFO("Test:lon a not normal.");
      return true;
    }

    if (lon_a < -DEC_LIMIT || lon_a > ACC_LIMIT)
    {
      ROS_INFO("Test:lon a is error,lon_a=%f", lon_a);
      return true;
    }

    double kappa = point.getKappa();
    if (!std::isnormal(kappa))
    {
      ROS_INFO("Test:kappa not normal.");
      return true;
    }

//    if (kappa < -KAPPA_LIMIT || kappa > KAPPA_LIMIT)
//    {
//      ROS_INFO("Test:kappa is error,kappa=%f", kappa);
//      return true;
//    }

    double lat_a = point.getVel() * point.getVel() * point.getKappa();
    if (lat_a < -LAT_ACC_LIMIT || lat_a > LAT_ACC_LIMIT)
    {
      ROS_INFO("Test:lat a is error,lat_a=%f", lat_a);
      return true;
    }
  }

  for (uint32_t i = 1; i < trajectory.getCartesianTrajectory().size(); ++i)
  {
    CartesianPoint point1 = trajectory.getCartesianPointByIndex(i - 1);
    CartesianPoint point2 = trajectory.getCartesianPointByIndex(i);

    double dt      = point2.getRelativeTime() - point1.getRelativeTime();
    double d_lon_a = point2.getAcc() - point1.getAcc();

    double lon_jerk = d_lon_a / dt;
  }

  // double lon_j = point.

  return false;
}

// bool TrajectoryChecker::isTrajCollided(FreTraj &traj)
bool TrajectoryChecker::isTrajCollided(const Trajectory &trajectory)
{
  // check if the envelope of the vehicle at each trajctory point overlaps with the envelope of the obstacles
  if (envi_info_.getObstaclesSize() == 0)
  {
    // ROS_INFO("no obs!!!");
    return false;
  }

  // for (int i=0;i<traj.traj.size();++i)

  for (CartesianPoint point : trajectory.getCartesianTrajectory())
  {
    // calculate the footprint of every traj point

    std::vector< CartesianPoint > boundary = getBoundaryByCartesianLocation(point);

    for (Obstacle obstacle : envi_info_.getObstacles())
    // for (int j=0;j<envi_info_.obstacle_vec.size();++j)
    {
      // calculate the foot print of every obstacle
      // Obstacle obstacle = envi_info_.obstacle_vec[j];

      Box2d box1(Vect(boundary[0].getX(), boundary[0].getY()), Vect(boundary[1].getX(), boundary[1].getY()),
                 Vect(boundary[2].getX(), boundary[2].getY()), Vect(boundary[3].getX(), boundary[3].getY()));

      Box2d box2;
      if (obstacle.isStatic())
      {
        box2 = Box2d(
            Vect(obstacle.getCorner1X(), obstacle.getCorner1Y()), Vect(obstacle.getCorner2X(), obstacle.getCorner2Y()),
            Vect(obstacle.getCorner3X(), obstacle.getCorner3Y()), Vect(obstacle.getCorner4X(), obstacle.getCorner4Y()));
      }
      else
      {
        box2 = Box2d(Vect(obstacle.getCorner1XByTime(point.getRelativeTime()),
                          obstacle.getCorner1YByTime(point.getRelativeTime())),
                     Vect(obstacle.getCorner2XByTime(point.getRelativeTime()),
                          obstacle.getCorner2YByTime(point.getRelativeTime())),
                     Vect(obstacle.getCorner3XByTime(point.getRelativeTime()),
                          obstacle.getCorner3YByTime(point.getRelativeTime())),
                     Vect(obstacle.getCorner4XByTime(point.getRelativeTime()),
                          obstacle.getCorner4YByTime(point.getRelativeTime())));
      }

      if (box1.HasOverlap(box2))
      {
        return true;
      }
    }
  }

  return false;
}

//增加轨迹点外包络是否出道路边界，若有出界点则返回true，否则返回false
bool TrajectoryChecker::isTrajOutOfRoad(const Trajectory &trajectory)
{
  for (CartesianPoint point : trajectory.getCartesianTrajectory())
  {
    std::vector< CartesianPoint > boundary = getBoundaryByCartesianLocation(
        point); //该方法是common/get_boundary_by_cartesian_location.h头文件中的静态方法，直接调用，返回该CartesianPoint的四个顶点Cartesian坐标

    std::vector< FrenetPoint > fre_points = getFrenetPointsBoundary(
        boundary,
        ref_line_); //该方法是common/get_boundary_by_cartesian_location.h头文件中的静态方法，直接调用，返回四个该CartesianPoint的四个顶点的Frenet坐标

    if (fre_points.size() < 4)
    {
      ROS_WARN("TrajectoryChecker:Trajectory point foot print transform failed.");
      return true;
    }

    for (FrenetPoint fre_point : fre_points)
    {
      if (fre_point.getS() > ref_line_.getReferenceLineMaxS())
        return true;

      ReferenceLinePoint ref_line_point = ref_line_.getReferenceLinePointByS(
          fre_point.getS()); //由每个frenet顶点坐标获取其对应的参考点，并判断其是否出道路边界
      if (fre_point.getL() > ref_line_point.getdWidthRight() || fre_point.getL() < -ref_line_point.getdWidthLeft())
        return true;
    }
  }

  return false;
}

} // end namespace
