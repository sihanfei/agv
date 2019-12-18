
#include "common/trajectory_pair.h"

namespace planning {

TrajectoryPair::TrajectoryPair(Curve1d* ls_path,Curve1d* st_path)
    :ls_path_(ls_path),st_path_(st_path)
{}

void TrajectoryPair::setLSPath(Curve1d* ls_path)
{
    ls_path_ = ls_path;
}

Curve1d* TrajectoryPair::getLSPath() const
{
    return ls_path_;
}

void TrajectoryPair::setSTPath(Curve1d* st_path)
{
    st_path_ = st_path;
}

Curve1d* TrajectoryPair::getSTPath() const
{
    return st_path_;
}

void TrajectoryPair::setCost(double cost)
{
    cost_ = cost;
}

double TrajectoryPair::getCost() const
{
    return cost_;
}
}
