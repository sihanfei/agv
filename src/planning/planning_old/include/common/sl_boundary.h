#ifndef SL_BOUNDARY_H
#define SL_BOUNDARY_H

#include "common/utils.h"

namespace planning {

class SLBoundary
{
public:
    SLBoundary()=default;
    ~SLBoundary()=default;

    SLBoundary(double start_s,double end_s,double start_l,double end_l);

    void setStartS(const double start_s);
    void setEndS(const double end_s);
    void setStartL(const double start_l);
    void setEndL(const double end_l);

    double getStartS() const;
    double getEndS() const;
    double getStartL() const;
    double getEndL() const;

    void setdS(double ds);
    double getdS() const;

    void setSRelatedStartL(double s_related_start_l);
    void setSRelatedEndL(double s_related_end_l);
    void setLRelatedStartS(double l_related_start_s);
    void setLRelatedEndS(double l_related_end_s);

    double getSRelatedStartL() const;
    double getSRelatedEndL() const;
    double getLRelatedStartS() const;
    double getLRelatedEndS() const;

private:
    double start_s_;
    double end_s_;
    double start_l_;
    double end_l_;
    double ds_;

    double s_related_start_l_;
    double s_related_end_l_;
    double l_related_start_s_;
    double l_related_end_s_;
};

}


#endif // SL_BOUNDARY_H
