#include "common/frenet_point.h"

namespace planning
{
FrenetPoint::FrenetPoint(double s,double ds,double dds,double l,double dl,double ddl,double relative_time)
    :s_(s),ds_(ds),dds_(dds),l_(l),dl_(dl),ddl_(ddl),relative_time_(relative_time)
{

}

void FrenetPoint::setS(double s)
{
    s_ = s;
}

void FrenetPoint::setdS(double ds)
{
    ds_ = ds;
}

void FrenetPoint::setddS(double dds)
{
    dds_ = dds;
}

void FrenetPoint::setL(double l)
{
    l_ = l;
}
void FrenetPoint::setdL(double dl)
{
    dl_ = dl;
}
void FrenetPoint::setddL(double ddl)
{
    ddl_ = ddl;
}
void FrenetPoint::setRelativeTime(double relative_time)
{
    relative_time_ = relative_time;
}

double FrenetPoint::getS() const
{
    return s_;
}

double FrenetPoint::getdS() const
{
    return ds_;
}

double FrenetPoint::getddS() const
{
    return dds_;
}

double FrenetPoint::getL() const
{
    return l_;
}
double FrenetPoint::getdL() const
{
    return dl_;
}
double FrenetPoint::getddL() const
{
    return ddl_;
}
double FrenetPoint::getRelativeTime() const
{
    return relative_time_;
}
}
