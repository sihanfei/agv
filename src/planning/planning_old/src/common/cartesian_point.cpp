#include "common/cartesian_point.h"

namespace planning
{
CartesianPoint::CartesianPoint(double x,double y,double theta,double kappa,double vel,double acc,double relative_time)
    :x_(x),y_(y),theta_(theta),kappa_(kappa),vel_(vel),acc_(acc),relative_time_(relative_time)
{

}

void CartesianPoint::setX(double x)
{
    x_ = x;
}

void CartesianPoint::setY(double y)
{
    y_ = y;
}

void CartesianPoint::setTheta(double theta)
{
    theta_ = theta;
}

void CartesianPoint::setKappa(double kappa)
{
    kappa_ = kappa;
}
void CartesianPoint::setVel(double vel)
{
    vel_ = vel;
}
void CartesianPoint::setAcc(double acc)
{
    acc_ = acc;
}
void CartesianPoint::setRelativeTime(double relative_time)
{
    relative_time_ = relative_time;
}

double CartesianPoint::getX() const
{
    return x_;
}

double CartesianPoint::getY() const
{
    return y_;
}

double CartesianPoint::getTheta() const
{
    return theta_;
}

double CartesianPoint::getKappa() const
{
    return kappa_;
}
double CartesianPoint::getVel() const
{
    return vel_;
}
double CartesianPoint::getAcc() const
{
    return acc_;
}
double CartesianPoint::getRelativeTime() const
{
    return relative_time_;
}
}
