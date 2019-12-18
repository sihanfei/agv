#ifndef SAMPLE_POINT_H
#define SAMPLE_POINT_H

#include "common/utils.h"

namespace planning{

class SamplePoint
{
public:

    SamplePoint()=default;
    ~SamplePoint()=default;

    SamplePoint(double s,double ds,double dds,double t);

    double getS() const;
    double getdS() const;
    double getddS() const;
    double getRelativeTime() const;

    void setS(double s);
    void setdS(double ds);
    void setddS(double dds);
    void setRelativeTime(double relative_time);

private:
    double s_;
    double ds_;
    double dds_;
    double t_;
};
}

#endif // SAMPLE_POINT_H
