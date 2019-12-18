#include "utils.h"

namespace pnc
{

// 获取frennet坐标下的轨迹信息
std::vector<FrenetPoint> Trajectory::getFrenetTrajectory() const
{
    return frenet_trajectory_;
}

// 获取cartesian坐标下的轨迹信息
std::vector<CartesianPoint> Trajectory::getCartesianTrajectory() const
{
    return cartesian_trajectory_;
}

// 获取轨迹点个数：若转换前后frenet点和cartesian点个数相同，则返回点个数，若不同，则认为轨迹无效
uint32_t Trajectory::getTrajectorySize() const
{

	if (frenet_trajectory_.size() == cartesian_trajectory_.size())
	{
		return frenet_trajectory_.size();
	}
	else
	{
		ROS_WARN("Trajectory:frenet traj size != cartesian traj size.");
		return 0;
	}
}

// 获取轨迹上最大的S值
double Trajectory::getMaxS()
{
    return frenet_trajectory_.back().getS();
}

// 在轨迹中增加一个frenet轨迹点
void Trajectory::addFrenetTrajectoryPoint(FrenetPoint frenet_point)
{
    frenet_trajectory_.emplace_back(frenet_point);
}

// 在轨迹中增加一个cartesian轨迹点
void Trajectory::addCartesianTrajectoryPoint(CartesianPoint cartesian_point)
{
    cartesian_trajectory_.emplace_back(cartesian_point);
}

// 获取轨迹的评价值
double Trajectory::getCost() const
{
    return cost_;
}

// 设置轨迹的评价值
void Trajectory::setCost(double cost)
{
    cost_ = cost;
}

// 根据index查找frenet轨迹上相对应的点
FrenetPoint Trajectory::getFrenetPointByIndex(uint32_t index) const
{
    return frenet_trajectory_[index];
}

// 根据index查找cartesian轨迹上相对应的点
CartesianPoint Trajectory::getCartesianPointByIndex(uint32_t index) const
{
    return cartesian_trajectory_[index];
}

// 根据cartesian坐标在cartesian轨迹上找最近的点的index
uint32_t Trajectory::getNearestCartesianPointIndex(CartesianPoint cartesian_point)
{
    uint32_t dis_min_index = 0;
    double dis_min = std::numeric_limits< double >::max();

    for (uint32_t i = 0; i < cartesian_trajectory_.size();++i)
    {
        CartesianPoint car_point = cartesian_trajectory_[i];
        double dis = sqrt(pow(cartesian_point.getX() - car_point.getX(),2) + pow(cartesian_point.getY() - car_point.getY(),2));
        if (dis < dis_min)
        {
            dis_min = dis;
            dis_min_index = i;
        }
    }

    return dis_min_index;
}

// 清除轨迹上所有点 20190918
void Trajectory::clearTrajectoryPoint()
{
	frenet_trajectory_.clear();
	cartesian_trajectory_.clear();
}

// 计算上一条轨迹和当前轨迹的相似性
double Trajectory::CalculateCartesianTrajectoryCoincidence(Trajectory& last_trajectory)
{
    uint32_t dis_min_index_last = 0;
    double dis_min_last = std::numeric_limits< double >::max();

    CartesianPoint traj_start_point = cartesian_trajectory_.front();//获取当前轨迹的起始点

    //找寻轨迹比较的起始位置
    //找到上一条轨迹中与当前轨迹起始点的最临近点下标
    for (uint32_t i = 0; i < last_trajectory.getTrajectorySize();++i)
    {
        CartesianPoint car_point = last_trajectory.getCartesianPointByIndex(i);
        double dis = sqrt(pow(traj_start_point.getX() - car_point.getX(),2) + pow(traj_start_point.getY() - car_point.getY(),2));
        if (dis < dis_min_last)
        {
            dis_min_last = dis;
            dis_min_index_last = i;
        }
    }

	 FrenetPoint corres_point = last_trajectory.getFrenetPointByIndex(dis_min_index_last);

	 double s1 = corres_point.getS() - last_trajectory.getFrenetTrajectory().front().getS();

    uint32_t dis_min_index = 0;
    double dis_min = std::numeric_limits< double >::max();

    //找寻轨迹比较的终点位置
    //若新轨迹长，则用老轨迹在新轨迹上的映射，求得新轨迹多出的部分
    //若老轨迹长，则用新轨迹在老轨迹上的映射，求得老轨迹多出的部分

    //不重叠部分也作为一项评价指标
    double coin = 0.0;
    double dis_dif = 0.0;

    //老轨迹长
    if (last_trajectory.getMaxS() > frenet_trajectory_.back().getS())
    {
         CartesianPoint traj_end_point = cartesian_trajectory_.back();
        //找到当前轨迹末点与上一条轨迹中的最临近点下标
        for (uint32_t i = 0; i < last_trajectory.getTrajectorySize();++i)
        {
            CartesianPoint car_point = last_trajectory.getCartesianPointByIndex(i);
            double dis = sqrt(pow(traj_end_point.getX() - car_point.getX(),2) + pow(traj_end_point.getY() - car_point.getY(),2));
            if (dis < dis_min)
            {
                dis_min = dis;
                dis_min_index = i;
            }
        }
			
	 	  FrenetPoint corres_point_last1 = last_trajectory.getFrenetPointByIndex(dis_min_index);
		  
        dis_dif = s1 + (last_trajectory.getFrenetTrajectory().back().getS() - corres_point_last1.getS());  

        //对应点的值相加求总的coin
        for (uint32_t i = dis_min_index_last; i < dis_min_index; ++i)
        {
            CartesianPoint last_car_point = last_trajectory.getCartesianPointByIndex(i);
            CartesianPoint car_point = cartesian_trajectory_[i-dis_min_index_last];
            coin = coin + fabs(car_point.getX() - last_car_point.getX()) + fabs(car_point.getY() - last_car_point.getY()) 
                        + fabs(car_point.getTheta() - last_car_point.getTheta()) + fabs(car_point.getVel() - last_car_point.getVel());

				ROS_INFO("Test:old traj long,dis_x=%f,dis_y=%f,dis_theta=%f,dis_vel=%f",
		fabs(car_point.getX() - last_car_point.getX()),fabs(car_point.getY() - last_car_point.getY()),
		fabs(car_point.getTheta() - last_car_point.getTheta()),fabs(car_point.getVel() - last_car_point.getVel()));

        }
    }
    //新轨迹长
    else
    {
         CartesianPoint last_traj_end_point = last_trajectory.getCartesianTrajectory().back();
        //找到当前轨迹末点与上一条轨迹中的最临近点下标
        for (uint32_t i = 0; i < cartesian_trajectory_.size();++i)
        {
            CartesianPoint car_point = cartesian_trajectory_[i];
            double dis = sqrt(pow(last_traj_end_point.getX() - car_point.getX(),2) + pow(last_traj_end_point.getY() - car_point.getY(),2));
            if (dis < dis_min)
            {
                dis_min = dis;
                dis_min_index = i;
            }
        }

	 	  FrenetPoint corres_point_last2 = frenet_trajectory_[dis_min_index];
		  
        dis_dif = s1 + (frenet_trajectory_.back().getS() - corres_point_last2.getS()); 

        //对应点的值相加求总的coin
        for (uint32_t i = dis_min_index_last; i < last_trajectory.getTrajectorySize(); ++i)
        {
            CartesianPoint last_car_point = last_trajectory.getCartesianPointByIndex(i);
            CartesianPoint car_point = cartesian_trajectory_[i-dis_min_index_last];
            coin = coin + fabs(car_point.getX() - last_car_point.getX()) + fabs(car_point.getY() - last_car_point.getY()) 
                        + fabs(car_point.getTheta() - last_car_point.getTheta()) + fabs(car_point.getVel() - last_car_point.getVel());


				ROS_INFO("Test:new traj long,dis_x=%f,dis_y=%f,dis_theta=%f,dis_vel=%f",
		fabs(car_point.getX() - last_car_point.getX()),fabs(car_point.getY() - last_car_point.getY()),
		fabs(car_point.getTheta() - last_car_point.getTheta()),fabs(car_point.getVel() - last_car_point.getVel())
);


        }
         
    }

	ROS_INFO("Test:coin=%f,dis_dif=%f,s_dif=%f",coin,dis_dif,(frenet_trajectory_.back().getS() - frenet_trajectory_.front().getS()) 
									+ (last_trajectory.getFrenetTrajectory().back().getS() - last_trajectory.getFrenetTrajectory().front().getS()));
    coin = (coin + dis_dif)/((frenet_trajectory_.back().getS() - frenet_trajectory_.front().getS()) 
									+ (last_trajectory.getFrenetTrajectory().back().getS() - last_trajectory.getFrenetTrajectory().front().getS()));
     
    return coin;
}



}

