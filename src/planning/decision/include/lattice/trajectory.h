#ifndef FRENET_TRAJECTORY_H
#define FRENET_TRAJECTORY_H


#include "common/cartesian_point.h"
#include "common/frenet_point.h"
#include "common/cartesian_frenet_converter.h"
#include <inttypes.h>

namespace pnc
{

class Trajectory
{
public:

    Trajectory()=default;
    ~Trajectory()=default;

    std::vector<FrenetPoint> getFrenetTrajectory() const;					// 获取frennet坐标下的轨迹信息
    std::vector<CartesianPoint> getCartesianTrajectory() const;		// 获取cartesian坐标下的轨迹信息

    uint32_t getTrajectorySize() const;	// 获取轨迹点个数
    double getMaxS();										// 获取轨迹上最大的S值
    double getCost() const;							// 获取轨迹的评价值
    void setCost(double cost);					// 设置轨迹的评价值

    void addFrenetTrajectoryPoint(FrenetPoint frenet_point);									// 在轨迹中增加一个frenet轨迹点
    void addCartesianTrajectoryPoint(CartesianPoint cartesian_point);					// 在轨迹中增加一个cartesian轨迹点

    FrenetPoint getFrenetPointByIndex(uint32_t index) const;									// 根据index查找frenet轨迹上相对应的点
    CartesianPoint getCartesianPointByIndex(uint32_t index) const;						// 根据index查找cartesian轨迹上相对应的点

    uint32_t getNearestCartesianPointIndex(CartesianPoint cartesian_point); 	// 根据cartesian坐标在cartesian轨迹上找最近的点的index

    double CalculateCartesianTrajectoryCoincidence(Trajectory& last_trajectory);	// 计算上一条轨迹和当前轨迹的相似性


		void clearTrajectoryPoint();	// 清除历史轨迹信息



    std::vector<double> s_vec_;			// 轨迹上的s值
    std::vector<double> l_vec_;			// 轨迹上的l值
	 	std::vector<double> ds_vec_;		// 轨迹上的v值

private:

    std::vector<FrenetPoint> frenet_trajectory_;					// frenet轨迹点坐标
    std::vector<CartesianPoint> cartesian_trajectory_;		// cartesian轨迹点坐标
    double cost_;				// 轨迹的cost值
};

}//end namespace



#endif // FRENET_TRAJECTORY_H

