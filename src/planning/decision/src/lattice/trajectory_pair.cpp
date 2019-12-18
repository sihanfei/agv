#include "utils.h"


namespace pnc
{

TrajectoryPair::TrajectoryPair(Curve* ls_path,Curve* st_path)
    :ls_path_(ls_path),st_path_(st_path)
{}

void TrajectoryPair::setLSPath(Curve* ls_path)
{
    ls_path_ = ls_path;
}

void TrajectoryPair::setSTPath(Curve* st_path)
{
    st_path_ = st_path;
}

void TrajectoryPair::setCost(double cost)
{
    cost_ = cost;
}

Curve* TrajectoryPair::getSTPath() const
{
    return st_path_;
}

Curve* TrajectoryPair::getLSPath() const
{
    return ls_path_;
}

double TrajectoryPair::getCost() const
{
    return cost_;
}




// 新增设计轨迹评价参数函数 20190920

void TrajectoryPair::setCost_lat_jerk(double cost)
{
    cost_lat_jerk = cost;
}
void TrajectoryPair::setCost_lat_s(double cost)
{
    cost_lat_s = cost;
}
void TrajectoryPair::setCost_lat_l(double cost)
{
    cost_lat_l = cost;
}
void TrajectoryPair::setCost_lon_jerk(double cost)
{
    cost_lon_jerk = cost;
}
void TrajectoryPair::setCost_lon_t(double cost)
{
    cost_lon_t = cost;
}
void TrajectoryPair::setCost_lon_s(double cost)
{
    cost_lon_s = cost;
}
void TrajectoryPair::setCost_lon_v(double cost)
{
    cost_lon_v = cost;
}



double TrajectoryPair::getCost_lat_jerk() const
{
    return cost_lat_jerk;
}
double TrajectoryPair::getCost_lat_s() const
{
    return cost_lat_s;
}
double TrajectoryPair::getCost_lat_l() const
{
    return cost_lat_l;
}
double TrajectoryPair::getCost_lon_jerk() const
{
    return cost_lon_jerk;
}
double TrajectoryPair::getCost_lon_t() const
{
    return cost_lon_t;
}
double TrajectoryPair::getCost_lon_s() const
{
    return cost_lon_s;
}
double TrajectoryPair::getCost_lon_v() const
{
    return cost_lon_v;
}








}

