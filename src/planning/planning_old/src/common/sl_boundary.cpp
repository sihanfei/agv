
#include "common/sl_boundary.h"

namespace planning {

SLBoundary::SLBoundary(double start_s,double end_s,double start_l,double end_l)
    :start_s_(start_s),end_s_(end_s),start_l_(start_l),end_l_(end_l)
{

}

void SLBoundary::setStartS(const double start_s)
{
    start_s_ = start_s;
}
void SLBoundary::setEndS(const double end_s)
{
    end_s_ = end_s;
}
void SLBoundary::setStartL(const double start_l)
{
    start_l_ = start_l;
}
void SLBoundary::setEndL(const double end_l)
{
    end_l_ = end_l;
}
double SLBoundary::getStartS() const
{
    return start_s_;
}
double SLBoundary::getEndS() const
{
    return end_s_;
}
double SLBoundary::getStartL() const
{
    return start_l_;
}
double SLBoundary::getEndL() const
{
    return end_l_;
}

void SLBoundary::setdS(double ds)
{
    ds_=ds;
}
double SLBoundary::getdS() const
{
    return ds_;
}

void SLBoundary::setSRelatedStartL(double s_related_start_l)
{
    s_related_start_l_ = s_related_start_l;
}
void SLBoundary::setSRelatedEndL(double s_related_end_l)
{
    s_related_end_l_ = s_related_end_l;
}
void SLBoundary::setLRelatedStartS(double l_related_start_s)
{
    l_related_start_s_ = l_related_start_s;
}
void SLBoundary::setLRelatedEndS(double l_related_end_s)
{
    l_related_end_s_ = l_related_end_s;
}

double SLBoundary::getSRelatedStartL() const
{
    return s_related_start_l_;
}
double SLBoundary::getSRelatedEndL() const
{
    return s_related_end_l_;
}
double SLBoundary::getLRelatedStartS() const
{
    return l_related_start_s_;
}
double SLBoundary::getLRelatedEndS() const
{
    return l_related_end_s_;
}

}


