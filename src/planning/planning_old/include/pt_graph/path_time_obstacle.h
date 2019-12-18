#ifndef PATH_TIME_OBSTACLE_H
#define PATH_TIME_OBSTACLE_H

#include "common/utils.h"
#include "pt_graph/path_time_point.h"

namespace planning
{
class PathTimeObstacle
{
public:
    PathTimeObstacle() = default;
    ~PathTimeObstacle() = default;

    void setObstacleId(uint32_t obstacle_id);
    uint32_t getObstacleId() const;
    //std::vector<PathTimePoint> path_time_points;
    PathTimePoint getUpperLeftPoint() const;
    PathTimePoint getUpperRightPoint() const;
    PathTimePoint getBottomLeftPoint() const;
    PathTimePoint getBottomRightPoint() const;

    void setUpperLeftPoint(const PathTimePoint upper_left_point);
    void setUpperRightPoint(const PathTimePoint upper_right_point);
    void setBottomLeftPoint(const PathTimePoint bottom_left_point);
    void setBottomRightPoint(const PathTimePoint bottom_right_point);

    double getPathUpper() const;
    double getPathLower() const;
    double getTimeUpper() const;
    double getTimeLower() const;

    void setPathUpper(const double path_upper);
    void setPathLower(const double path_lower);
    void setTimeUpper(const double time_upper);
    void setTimeLower(const double time_lower);

    void setdS(double ds);
    double getdS() const;

private:
    uint32_t obstacle_id_;
    PathTimePoint upper_left_point_;
    PathTimePoint upper_right_point_;
    PathTimePoint bottom_left_point_;
    PathTimePoint bottom_right_point_;
    double path_upper_;
    double path_lower_;
    double time_upper_;
    double time_lower_;

    double ds_;
};
}



#endif // PATH_TIME_OBSTACLE_H
