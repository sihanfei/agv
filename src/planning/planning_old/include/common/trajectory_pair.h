#ifndef TRAJECTORY_PAIR_H
#define TRAJECTORY_PAIR_H

#include "math/curve1d.h"

namespace planning {

class TrajectoryPair
{
public:
    TrajectoryPair()=default;
    ~TrajectoryPair()=default;
    TrajectoryPair(Curve1d* ls_path,Curve1d* st_path);

    void setLSPath(Curve1d* ls_path);
    Curve1d* getLSPath() const;

    void setSTPath(Curve1d* st_path);
    Curve1d* getSTPath() const;

    void setCost(double cost);
    double getCost() const;

    //double vel_cost_;//for test

private:
    Curve1d* ls_path_;
    Curve1d* st_path_;
    double cost_;
};
}

#endif // TRAJECTORY_PAIR_H
