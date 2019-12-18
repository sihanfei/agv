#include "utils.h"

namespace pnc
{

// 单个障碍物处理

Obstacle::Obstacle(uint32_t id, Eigen::Vector2d corner1, Eigen::Vector2d corner2, Eigen::Vector2d corner3,
                   Eigen::Vector2d corner4, double velocity, double heading)
    : id_(id), corner1_(corner1), corner2_(corner2), corner3_(corner3), corner4_(corner4),velocity_(velocity),heading_(heading)
{
   
}



bool Obstacle::init(ReferenceLine& ref_line)	// 障碍物信息初始化
{

	is_static_ = true;		// 目前车辆感知能力有限，所有的障碍物都当做静态障碍物来处理

	ROS_WARN("Obstacle: all Obstacle are seen as static , because of the ability of perception is limited. ");

/*
	// 判断障碍物是静止还是动态的
	if (fabs(velocity_) < EPSILON)
  {
      is_static_ = true;
  }
  else
  {
      is_static_ = false;
  }
*/
	// 障碍物四个顶点信息
	corner_vec_.push_back(corner1_);
	corner_vec_.push_back(corner2_);
	corner_vec_.push_back(corner3_);
	corner_vec_.push_back(corner4_);


	// 计算障碍物的边界信息
	double s_temp_max = -MAX_NUM;
	double s_temp_min = MAX_NUM;
	double l_temp_max = -MAX_NUM;
	double l_temp_min = MAX_NUM;

	
	double x_obs[4],y_obs[4];
	uint8_t i = 0;
	for(Eigen::Vector2d corner : getAllCorners())	// 依次遍历障碍物的四个角
	{
		
		CartesianPoint temp_point_cartesian;
		temp_point_cartesian.setX(corner(0));
		temp_point_cartesian.setY(corner(1));
		temp_point_cartesian.setTheta(ThetaTransform(angle2Radian(heading_)));	// 注意将角度转换为弧度
		temp_point_cartesian.setVel(0.0);					// 转换时，假定障碍物速度为0，这是因为，计算障碍物的边界信息，不牵涉到速度的计算			
		temp_point_cartesian.setAcc(0.0);
		temp_point_cartesian.setKappa(0.0);

		FrenetPoint temp_point_frenet;
		if (!cartesianToFrenet(temp_point_cartesian, ref_line, temp_point_frenet))	// 将障碍物转换到frenet坐标下
		{
			ROS_WARN("Obstacle--cartesian transform to Frenet failed.");
			return false;
		}

		if(s_temp_max < temp_point_frenet.getS())
		{
			s_temp_max = temp_point_frenet.getS();
		}
		if(s_temp_min > temp_point_frenet.getS())
		{
			s_temp_min = temp_point_frenet.getS();
		}

		if(l_temp_max < temp_point_frenet.getL())
		{
			l_temp_max = temp_point_frenet.getL();
		}
		if(l_temp_min > temp_point_frenet.getL())
		{
			l_temp_min = temp_point_frenet.getL();
		}

		x_obs[i] = corner(0);
		y_obs[i] = corner(1);
		i = i + 1;
	}

	// 设置障碍物的边界信息，将这些边界信息写入到障碍物类中
	setSmax(s_temp_max);
	setSmin(s_temp_min);
	setLmax(l_temp_max);
	setLmin(l_temp_min);

	ROS_INFO("Obstacle: Obs_smax = %f, Obs_smin = %f, Obs_lmax = %f, Obs_lmin = %f.",s_temp_max,s_temp_min,l_temp_max,l_temp_min);

	// 构建障碍物四顶点包络
	box_obs_ = Box2d(Vect(x_obs[0], y_obs[0]),Vect(x_obs[1], y_obs[1]),Vect(x_obs[2], y_obs[2]),Vect(x_obs[3], y_obs[3]));
	

	return true;

}



uint32_t Obstacle::getId() const
{
  return id_;
}
double Obstacle::getHeading() const
{
  return heading_;
}
double Obstacle::getVelocity() const
{
  return velocity_;
}

bool Obstacle::isStatic() const
{
  return is_static_;
}

std::vector< Eigen::Vector2d > Obstacle::getAllCorners() const
{
  return corner_vec_;
}

// 设置障碍物在frenet坐标下的边框信息
void Obstacle::setSmax(double s)
{
	s_max_ = s;
}
void Obstacle::setSmin(double s)
{
	s_min_ = s;
}
void Obstacle::setLmax(double l)
{
	l_max_ = l;
}
void Obstacle::setLmin(double l)
{
	l_min_ = l;
}

// 获取障碍物在frenet坐标下的边框信息
double Obstacle::getSmax()
{
	return s_max_;
}
double Obstacle::getSmin()
{
	return s_min_;
}
double Obstacle::getLmax()
{
	return l_max_;
}
double Obstacle::getLmin()
{
	return l_min_;
}


// 获取障碍物的四顶点包络
Box2d Obstacle::getBox2d() const 
{
	return box_obs_;
}


// 感知环境处理

void PerceptionInfo::clearObstacleVec()
{
  obstacle_vec_.clear();
}

void PerceptionInfo::setObstacleVec(const std::vector< Obstacle > obstacles)
{
	obstacle_vec_ = obstacles;
}

std::vector< Obstacle >& PerceptionInfo::getObstacleVec()
{
  return obstacle_vec_;
}




}
