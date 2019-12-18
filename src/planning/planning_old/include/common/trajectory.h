#ifndef FRENET_TRAJECTORY_H
#define FRENET_TRAJECTORY_H

#include "common/utils.h"

#include "common/frenet_point.h"
#include "common/cartesian_point.h"

namespace planning {

class Trajectory
{
public:
    Trajectory()=default;
    ~Trajectory()=default;

    std::vector<FrenetPoint> getFrenetTrajectory() const;
    std::vector<CartesianPoint> getCartesianTrajectory() const;

    uint32_t getTrajectorySize() const;

    void addFrenetTrajectoryPoint(FrenetPoint frenet_point);
    void addCartesianTrajectoryPoint(CartesianPoint cartesian_point);

    double getCost() const;
    void setCost(double cost);

    FrenetPoint getFrenetPointByIndex(uint32_t index) const;
    CartesianPoint getCartesianPointByIndex(uint32_t index) const;

    uint32_t getNearestCartesianPointIndex(CartesianPoint cartesian_point); 

    double CalculateCartesianTrajectoryCoincidence(Trajectory& last_trajectory);

    double getMaxS();

private:
    std::vector<FrenetPoint> frenet_trajectory_;
    std::vector<CartesianPoint> cartesian_trajectory_;

    double cost_;
};

}//end namespace



#endif // FRENET_TRAJECTORY_H
