#include "utils.h"

namespace pnc
{

CartesianPoint::CartesianPoint(double x,double y,double theta,double kappa,double vel,double acc)
    :x_(x),y_(y),theta_(theta),kappa_(kappa),vel_(vel),acc_(acc)
{

}

// 给私有变量赋值
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

// 获取私有变量的值
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

}
