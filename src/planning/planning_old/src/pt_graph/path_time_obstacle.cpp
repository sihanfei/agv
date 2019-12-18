#include "pt_graph/path_time_obstacle.h"

namespace planning {
void PathTimeObstacle::setObstacleId(uint32_t obstacle_id)
{
    obstacle_id_ = obstacle_id;
}
uint32_t PathTimeObstacle::getObstacleId() const
{
    return obstacle_id_;
}

PathTimePoint PathTimeObstacle::getUpperLeftPoint() const
{
    return upper_left_point_;
}
PathTimePoint PathTimeObstacle::getUpperRightPoint() const
{
    return upper_right_point_;
}
PathTimePoint PathTimeObstacle::getBottomLeftPoint() const
{
    return bottom_left_point_;
}
PathTimePoint PathTimeObstacle::getBottomRightPoint() const
{
    return bottom_right_point_;
}

void PathTimeObstacle::setUpperLeftPoint(const PathTimePoint upper_left_point)
{
    upper_left_point_ = upper_left_point;
}
void PathTimeObstacle::setUpperRightPoint(const PathTimePoint upper_right_point)
{
    upper_right_point_ = upper_right_point;
}
void PathTimeObstacle::setBottomLeftPoint(const PathTimePoint bottom_left_point)
{
    bottom_left_point_ = bottom_left_point;
}
void PathTimeObstacle::setBottomRightPoint(const PathTimePoint bottom_right_point)
{
    bottom_right_point_ = bottom_right_point;
}

void PathTimeObstacle::setPathUpper(const double path_upper)
{
    path_upper_ = path_upper;
}
void PathTimeObstacle::setPathLower(const double path_lower)
{
    path_lower_ = path_lower;
}
void PathTimeObstacle::setTimeUpper(const double time_upper)
{
    time_upper_ = time_upper;
}
void PathTimeObstacle::setTimeLower(const double time_lower)
{
    time_lower_ = time_lower;
}

double PathTimeObstacle::getPathUpper() const
{
    return path_upper_;
}
double PathTimeObstacle::getPathLower() const
{
    return path_lower_;
}
double PathTimeObstacle::getTimeUpper() const
{
    return time_upper_;
}
double PathTimeObstacle::getTimeLower() const
{
    return time_lower_;
}

void PathTimeObstacle::setdS(double ds)
{
    ds_=ds;
}
double PathTimeObstacle::getdS() const
{
    return ds_;
}

}
