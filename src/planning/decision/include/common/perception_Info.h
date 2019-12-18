#ifndef PERCEPTION_INFO_H
#define PERCEPTION_INFO_H

#include "ref_line.h"
#include "box2d.h"

namespace pnc
{
//障碍物类
class Obstacle
{

public:

  Obstacle()  = default;
  ~Obstacle() = default;

  Obstacle(uint32_t id, Eigen::Vector2d corner1, Eigen::Vector2d corner2, Eigen::Vector2d corner3,Eigen::Vector2d corner4, double velocity, double heading);

  uint32_t getId() const;			// 获取障碍物的ID
  double getHeading() const;	// 获取障碍物的朝向信息
  double getVelocity() const;	// 获取障碍物的速度信息
  bool isStatic() const;			// 获取障碍物的静止状态
  std::vector< Eigen::Vector2d > getAllCorners() const;	// 获取障碍物四点顶点信息

	// 障碍物信息初始化
	bool init(ReferenceLine& ref_line);

	// 设置障碍物在frenet坐标下的边框信息
	void setSmax(double s);
	void setSmin(double s);
	void setLmax(double l);
	void setLmin(double l);
	// 获取障碍物在frenet坐标下的边框信息
	double getSmax();
	double getSmin();
	double getLmax();
	double getLmin();

	// 获取障碍物的包络
	Box2d getBox2d() const;

private:

  uint32_t id_;			//障碍物的ID
  double heading_;	//障碍物的朝向信息
  double velocity_;	//障碍物的速度信息
  bool is_static_;	//障碍物的静止状态
	
	// 障碍物四点顶点信息
  Eigen::Vector2d corner1_;
  Eigen::Vector2d corner2_;
  Eigen::Vector2d corner3_;
  Eigen::Vector2d corner4_;
  std::vector< Eigen::Vector2d > corner_vec_;

	// 障碍物四个顶点的包络
	Box2d box_obs_;


	// 障碍物的边界信息
	double s_max_;
	double s_min_;
	double l_max_;
	double l_min_;


};

// 感知类
class PerceptionInfo
{

public:

  PerceptionInfo()  = default;
  ~PerceptionInfo() = default;

  void clearObstacleVec();																			// 清除障碍物信息
  void setObstacleVec(const std::vector< Obstacle > obstacles);	// 加入障碍物信息
  std::vector< Obstacle >& getObstacleVec();										// 获取障碍物信息


private:

  std::vector< Obstacle > obstacle_vec_;	// 环境中的障碍物信息

};

} 

#endif

