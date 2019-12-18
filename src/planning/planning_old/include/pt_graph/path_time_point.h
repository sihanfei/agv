#ifndef PATH_TIME_POINT_H
#define PATH_TIME_POINT_H

#include "common/utils.h"

namespace planning{

class PathTimePoint
{
public:
    PathTimePoint()=default;
    ~PathTimePoint()=default;
    PathTimePoint(uint32_t id,double s,double t);

    void setObstacleId(uint32_t id);
    uint32_t getObstacleId() const;

    void setTime(double t);
    double getTime() const;

    void setS(double s);
    double getS() const;

private:
    uint32_t obstacle_id_;
    double s_;
    double t_;
};

}

#endif // PATH_TIME_POINT_H
