
#include "pt_graph/path_time_point.h"

namespace planning{

PathTimePoint::PathTimePoint(uint32_t id,double s,double t)
    :obstacle_id_(id),s_(s),t_(t)
{

}

void PathTimePoint::setObstacleId(uint32_t id)
{
    obstacle_id_ = id;
}
uint32_t PathTimePoint::getObstacleId() const
{
    return obstacle_id_;
}

void PathTimePoint::setTime(double t)
{
    t_ = t;
}
double PathTimePoint::getTime() const
{
    return t_;
}

void PathTimePoint::setS(double s)
{
    s_ = s;
}
double PathTimePoint::getS() const
{
    return s_;
}
}
