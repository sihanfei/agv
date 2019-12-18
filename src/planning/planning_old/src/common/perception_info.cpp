
#include "common/perception_info.h"

namespace planning
{

// class Obstacle
Obstacle::Obstacle(uint32_t id, Eigen::Vector2d corner1, Eigen::Vector2d corner2, Eigen::Vector2d corner3,
                   Eigen::Vector2d corner4, double velocity, double heading)
    : id_(id), corner1_(corner1), corner2_(corner2), corner3_(corner3), corner4_(corner4), velocity_(velocity)
{
    if (velocity_ < EPSILON)
    {
      ROS_INFO("Obstacle:Obstacle id = %d is static.", id_);
      is_static_ = true;
    }
    else
    {
      ROS_INFO("Obstacle:Obstacle id = %d is dynamic.", id_);
      velocity_vect_(velocity * cos(heading), velocity * sin(heading));
      heading_   = heading;
      is_static_ = false;
    }
}

bool Obstacle::init(ReferenceLine& ref_line)
{
  corner_vec_.push_back(getCorner1());
  corner_vec_.push_back(getCorner2());
  corner_vec_.push_back(getCorner3());
  corner_vec_.push_back(getCorner4());

  double x_min(std::numeric_limits< double >::max());
  double x_max(std::numeric_limits< double >::lowest());
  double y_min(std::numeric_limits< double >::max());
  double y_max(std::numeric_limits< double >::lowest());

  for (Eigen::Vector2d corner : getAllCorners())
  {
    x_min = std::min(x_min,corner(0));
    x_max = std::max(x_max,corner(0));
    y_min = std::min(y_min,corner(1));
    y_max = std::max(y_max,corner(1));
  }
  double center_x = (x_min+x_max)*0.5;
  double center_y = (y_min+y_max)*0.5;

  ReferenceLinePoint ref_point = ref_line.getNearestRefLinePoint(center_x,center_y);

  uint8_t num = 0;

  for (Eigen::Vector2d corner : getAllCorners())
  {
    CartesianPoint car_point;
    car_point.setX(corner(0));
    car_point.setY(corner(1));

    car_point.setVel(velocity_);

    car_point.setTheta(ref_point.getTheta());

    car_point.setKappa(0.0);
    car_point.setAcc(0.0);
    car_point_vec_.push_back(car_point);

    FrenetPoint fre_point;
    if (cartesianToFrenet(car_point, ref_line, fre_point))
    {
      num++;

      double s_related_start_l;
      double s_related_end_l;
      double l_related_start_s;
      double l_related_end_s;

      double start_s(std::numeric_limits< double >::max());
      double end_s(std::numeric_limits< double >::lowest());
      double start_l(std::numeric_limits< double >::max());
      double end_l(std::numeric_limits< double >::lowest());

      if (start_s > fre_point.getS())
      {
        start_s           = fre_point.getS();
        l_related_start_s = fre_point.getL();
      }

      if (end_s < fre_point.getS())
      {
        end_s           = fre_point.getS();
        l_related_end_s = fre_point.getL();
      }

      if (start_l > fre_point.getL())
      {
        start_l           = fre_point.getL();
        s_related_start_l = fre_point.getS();
      }

      if (end_l < fre_point.getL())
      {
        end_l           = fre_point.getL();
        s_related_end_l = fre_point.getS();
      }

      sl_boundary_.setStartS(start_s);
      sl_boundary_.setEndS(end_s);
      sl_boundary_.setStartL(start_l);
      sl_boundary_.setEndL(end_l);

      sl_boundary_.setSRelatedStartL(s_related_start_l);
      sl_boundary_.setSRelatedEndL(s_related_end_l);
      sl_boundary_.setLRelatedStartS(l_related_start_s); // no use
      sl_boundary_.setLRelatedEndS(l_related_end_s);     // no use

      sl_boundary_.setdS(fre_point.getdS());
    }
    else
    {
      ROS_WARN("Perception_info:One cornor transform to frenet point failed,which Obstacle id=%d.", id_);
    }
  }

  if (num == 0) //没有一个顶点在参考线范围内
    return false;

  return true;
}

uint32_t Obstacle::getId() const
{
  return id_;
}
void Obstacle::setId(uint32_t id)
{
  id_ = id;
}

double Obstacle::getHeading() const
{
  return heading_;
}
void Obstacle::setHeading(double heading)
{
  heading_ = heading;
}

double Obstacle::getVelocity() const
{
  return velocity_;
}
void Obstacle::setVelocity(double v)
{
  velocity_ = v;
}

Eigen::Vector2d Obstacle::getCorner1() const
{
  return corner1_;
}
double Obstacle::getCorner1X() const
{
  return corner1_(0);
}

double Obstacle::getCorner1Y() const
{
  return corner1_(1);
}

Eigen::Vector2d Obstacle::getCorner2() const
{
  return corner2_;
}
double Obstacle::getCorner2X() const
{
  return corner2_(0);
}
double Obstacle::getCorner2Y() const
{
  return corner2_(1);
}

Eigen::Vector2d Obstacle::getCorner3() const
{
  return corner3_;
}
double Obstacle::getCorner3X() const
{
  return corner3_(0);
}
double Obstacle::getCorner3Y() const
{
  return corner3_(1);
}

Eigen::Vector2d Obstacle::getCorner4() const
{
  return corner4_;
}
double Obstacle::getCorner4X() const
{
  return corner4_(0);
}
double Obstacle::getCorner4Y() const
{
  return corner4_(1);
}

Eigen::Vector2d Obstacle::getVelocityVect() const
{
  return velocity_vect_;
}
double Obstacle::getVelocityVectX() const
{
  return velocity_vect_(0);
}
double Obstacle::getVelocityVectY() const
{
  return velocity_vect_(1);
}

double Obstacle::getCorner1XByTime(double t) const
{
  return corner1_(0) + velocity_vect_(0) * t;
}

double Obstacle::getCorner1YByTime(double t) const
{
  return corner1_(1) + velocity_vect_(1) * t;
}

double Obstacle::getCorner2XByTime(double t) const
{
  return corner2_(0) + velocity_vect_(0) * t;
}

double Obstacle::getCorner2YByTime(double t) const
{
  return corner2_(1) + velocity_vect_(1) * t;
}

double Obstacle::getCorner3XByTime(double t) const
{
  return corner3_(0) + velocity_vect_(0) * t;
}

double Obstacle::getCorner3YByTime(double t) const
{
  return corner3_(1) + velocity_vect_(1) * t;
}

double Obstacle::getCorner4XByTime(double t) const
{
  return corner4_(0) + velocity_vect_(0) * t;
}

double Obstacle::getCorner4YByTime(double t) const
{
  return corner4_(1) + velocity_vect_(1) * t;
}

bool Obstacle::isStatic() const
{
  // return velocity_vect_(0) == 0 && velocity_vect_(1) == 0;
  return is_static_;
}

//  Box2d Obstacle::getBox2d() const
//  {
//      Box2d box( Vect(getCorner1X(),getCorner1Y()),
//                  Vect(getCorner2X(),getCorner2Y()),
//                  Vect(getCorner3X(),getCorner3Y()),
//                  Vect(getCorner4X(),getCorner4Y()) );
//      return box;
//  }

//  Box2d Obstacle::getBox2dByTime(double t) const
//  {
//      Box2d box( Vect(getCorner1XByTime(t),getCorner1YByTime(t)),
//                  Vect(getCorner2XByTime(t),getCorner2YByTime(t)),
//                  Vect(getCorner3XByTime(t),getCorner3YByTime(t)),
//                  Vect(getCorner4XByTime(t),getCorner4YByTime(t)) );
//      return box;
//  }

std::vector< Eigen::Vector2d > Obstacle::getAllCorners() const
{
  return corner_vec_;
}

std::vector< CartesianPoint > Obstacle::getAllCornerPoints() const
{
  return car_point_vec_;
}

std::vector< Eigen::Vector2d > Obstacle::getAllCornersByTime(double t) const
{
  std::vector< Eigen::Vector2d > corner_vec;
  for (Eigen::Vector2d corner : getAllCorners())
  {
    Eigen::Vector2d corner_t(corner(0) + velocity_vect_(0) * t, corner(1) + velocity_vect_(1) * t);

    corner_vec.push_back(corner_t);
  }

  return corner_vec;
}

std::vector< CartesianPoint > Obstacle::getAllCornerPointsByTime(double t) const
{
  std::vector< CartesianPoint > car_point_vec;

  for (Eigen::Vector2d corner : getAllCornersByTime(t))
  {
    CartesianPoint car_point;
    car_point.setX(corner(0));
    car_point.setY(corner(1));

    car_point.setVel(velocity_);
    car_point.setTheta(heading_);

    car_point.setKappa(0.0);
    car_point.setAcc(0.0);
    car_point_vec.push_back(car_point);
  }
  return car_point_vec;
}

SLBoundary Obstacle::getSLBoundary() const
{
  return sl_boundary_;
}

// class PerceptionInfo

std::vector< Obstacle > PerceptionInfo::getObstacles() const
{
  return obstacles_;
}

void PerceptionInfo::setObstacles(const std::vector< Obstacle > obstacles)
{
  obstacles_ = obstacles;
}

void PerceptionInfo::addObstacle(Obstacle obstacle)
{
  obstacles_.push_back(obstacle);
}

void PerceptionInfo::clearObstacles()
{
  obstacles_.clear();
}

uint8_t PerceptionInfo::getObstaclesSize() const
{
  return obstacles_.size();
}

Obstacle PerceptionInfo::getObstacleByIndex(uint32_t index) const
{
  // ReferenceLinePoint ref_line_point = ref_line_points_[index];
  return obstacles_[index];
}

} // end namespace
