
#include "lattice/lattice_trajectory1d.h"

namespace planning {

LatticeTrajectory1d::LatticeTrajectory1d(std::shared_ptr<Curve1d> ptr_trajectory1d)
{
    ptr_trajectory1d_ = ptr_trajectory1d;
}

double LatticeTrajectory1d::evaluate(uint8_t mode, double param) const
{
    double param_max = ptr_trajectory1d_->paramMax();

    if (param < param_max + EPSILON)
    {
        return ptr_trajectory1d_->evaluate(mode,param);
    }

    double p = ptr_trajectory1d_->getx1();
    double v = ptr_trajectory1d_->getdx1();
    double a = ptr_trajectory1d_->getddx1();

    double t = param - param_max;

    switch (mode)
    {
    case 0:
        return p + v*t + 0.5*a*t*t;

    case 1:
        return v + a*t;

    case 2:
        return a;

    default:
        return 0.0;
    }
}

double LatticeTrajectory1d::evaluateNormalization(uint8_t mode, double param) const
{
    double param_max = ptr_trajectory1d_->paramMax();

    if (param < param_max + EPSILON)
    {
        return ptr_trajectory1d_->evaluateNormalization(mode,param);
    }

    //There are mostly useless right now
    //该函数用作评分计算，外推逻辑用不到
    double p = ptr_trajectory1d_->getx1();
    double v = ptr_trajectory1d_->getdx1();
    double a = ptr_trajectory1d_->getddx1();

    double t = param - param_max;

    switch (mode)
    {
    case 0:
        return p + v*t + 0.5*a*t*t;

    case 1:
        return v + a*t;

    case 2:
        return a;

    default:
        return 0.0;
    }
    //------
}

double LatticeTrajectory1d::paramLength() const
{
    return ptr_trajectory1d_->paramLength();
}

double LatticeTrajectory1d::paramMax() const
{
    return ptr_trajectory1d_->paramMax();
}

double LatticeTrajectory1d::paramMin() const
{
    return ptr_trajectory1d_->paramMin();
}

void LatticeTrajectory1d::setCost(double cost)
{
    ptr_trajectory1d_->setCost(cost);
}

double LatticeTrajectory1d::getCost() const
{
    return ptr_trajectory1d_->getCost();
}

double LatticeTrajectory1d::getx1() const
{
    return ptr_trajectory1d_->getx1();
}

double LatticeTrajectory1d::getdx1() const
{
    return ptr_trajectory1d_->getdx1();
}
double LatticeTrajectory1d::getddx1() const
{
    return ptr_trajectory1d_->getddx1();
}

double LatticeTrajectory1d::getx0() const
{
    return ptr_trajectory1d_->getx0();
}
double LatticeTrajectory1d::getdx0() const
{
    return ptr_trajectory1d_->getdx0();
}
double LatticeTrajectory1d::getddx0() const
{
    return ptr_trajectory1d_->getddx0();
}

void LatticeTrajectory1d::setTrajInfo(std::string info)
{
    ptr_trajectory1d_->setTrajInfo(info);
}//for debug

void LatticeTrajectory1d::displayTrajInfo()
{
    ptr_trajectory1d_->displayTrajInfo();
}//for debug


// void LatticeTrajectory1d::setRestriction(Restriction& restriction)
// {
// 	ptr_trajectory1d_->setRestriction(restriction);
// }

// Restriction LatticeTrajectory1d::getRestriction() const
// {
// 	return ptr_trajectory1d_->getRestriction();

// }

}



