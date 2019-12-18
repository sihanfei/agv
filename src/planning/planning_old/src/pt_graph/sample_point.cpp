
#include "pt_graph/sample_point.h"

namespace planning {

SamplePoint::SamplePoint(double s,double ds,double dds,double t)
    :s_(s),ds_(ds),dds_(dds),t_(t)
{

}

double SamplePoint::getS() const
{
    return s_;
}
double SamplePoint::getdS() const
{
    return ds_;
}
double SamplePoint::getddS() const
{
    return dds_;
}
double SamplePoint::getRelativeTime() const
{
    return t_;
}

void SamplePoint::setS(double s)
{
    s_=s;
}
void SamplePoint::setdS(double ds)
{
    ds_=ds;
}
void SamplePoint::setddS(double dds)
{
    dds_=dds;
}
void SamplePoint::setRelativeTime(double relative_time)
{
    t_=relative_time;
}

}
