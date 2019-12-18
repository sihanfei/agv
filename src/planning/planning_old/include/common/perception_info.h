#ifndef PERCEPTION_INFO_H
#define PERCEPTION_INFO_H

#include "math/box2d.h"
#include "math/cartesian_frenet_converter.h"

#include "common/cartesian_point.h"
#include "common/sl_boundary.h"

namespace planning
{

class Obstacle
{
public:
  Obstacle()  = default;
  ~Obstacle() = default;

  Obstacle(uint32_t id, Eigen::Vector2d corner1, Eigen::Vector2d corner2, Eigen::Vector2d corner3,
           Eigen::Vector2d corner4, double velocity, double heading);

  // bool init(ReferenceLine &ref_line);
  bool init(ReferenceLine& ref_line);

  uint32_t getId() const;
  void setId(uint32_t id);

  double getHeading() const;
  void setHeading(double heading);

  double getVelocity() const;
  void setVelocity(double v);

  Eigen::Vector2d getCorner1() const;
  double getCorner1X() const;
  double getCorner1Y() const;

  Eigen::Vector2d getCorner2() const;
  double getCorner2X() const;
  double getCorner2Y() const;

  Eigen::Vector2d getCorner3() const;
  double getCorner3X() const;
  double getCorner3Y() const;

  Eigen::Vector2d getCorner4() const;
  double getCorner4X() const;
  double getCorner4Y() const;

  Eigen::Vector2d getVelocityVect() const;
  double getVelocityVectX() const;
  double getVelocityVectY() const;

  double getCorner1XByTime(double t) const;
  double getCorner1YByTime(double t) const;

  double getCorner2XByTime(double t) const;
  double getCorner2YByTime(double t) const;

  double getCorner3XByTime(double t) const;
  double getCorner3YByTime(double t) const;

  double getCorner4XByTime(double t) const;
  double getCorner4YByTime(double t) const;

  bool isStatic() const;

  //  Box2d getBox2d() const;

  //  Box2d getBox2dByTime(double t) const;

  std::vector< Eigen::Vector2d > getAllCorners() const;
  std::vector< CartesianPoint > getAllCornerPoints() const;

  std::vector< Eigen::Vector2d > getAllCornersByTime(double t) const;
  std::vector< CartesianPoint > getAllCornerPointsByTime(double t) const;

  SLBoundary getSLBoundary() const;

private:
  uint32_t id_;

  Eigen::Vector2d corner1_;
  Eigen::Vector2d corner2_;
  Eigen::Vector2d corner3_;
  Eigen::Vector2d corner4_;

  double heading_;
  double velocity_;
  Eigen::Vector2d velocity_vect_;

  std::vector< Eigen::Vector2d > corner_vec_;
  std::vector< CartesianPoint > car_point_vec_;

  bool is_static_;

  SLBoundary sl_boundary_;
};

class PerceptionInfo
{
public:
  PerceptionInfo()  = default;
  ~PerceptionInfo() = default;

  std::vector< Obstacle > getObstacles() const;
  void setObstacles(const std::vector< Obstacle > obstacles);
  void clearObstacles();
  void addObstacle(Obstacle obstacle);
  uint8_t getObstaclesSize() const;
  Obstacle getObstacleByIndex(uint32_t index) const;

private:
  std::vector< Obstacle > obstacles_;
};

} // end namespace

#endif // PERCEPTION_INFO_H
