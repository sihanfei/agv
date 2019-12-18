#ifndef PATH_TIME_GRAPH_H_
#define PATH_TIME_GRAPH_H_

//#include <ros/ros.h>

#include "common/utils.h"
#include "common/sl_boundary.h"
#include "common/perception_info.h"

#include "pt_graph/path_time_obstacle.h"
#include "pt_graph/sample_point.h"


namespace planning{

class PathTimeGraph
{
public:
    PathTimeGraph()=default;
    ~PathTimeGraph()=default;

    //设置ST图中s和t的范围，设置图上障碍物
    PathTimeGraph(const std::vector<Obstacle>& obstacles,
                  const ReferenceLine& ref_line,
                  const FrenetPoint& frelocation);

    //传入障碍物id、超车/跟车的距离、最小撒点间隔时间，计算得到一组可选择的ST点
    std::vector<PathTimePoint> getObstacleSurroundingPoints(uint32_t obstacle_id,
                                                            double s_dist,
                                                            double t_min_density) const;

    std::vector<PathTimeObstacle> getPathTimeObstacles() const;
    //std::unordered_map<unsigned int, Obstacle> getObstacleMap() const;

    std::unordered_map<uint32_t, PathTimeObstacle> getPathTimeObstaclesMap() const;

    //撒纵向点
    std::vector<SamplePoint> sampleLonEndConditionsForPathTimePoints(const FrenetPoint& fre_point);
    //std::vector<SamplePoint> sampleLonEndConditionsForPathTimePoints(const PathTimeGraph& path_time_graph, const FrenetPoint& fre_point);

    std::vector< std::vector< std::pair<double, double> > > getPathBlockingIntervals(const double t_start, const double t_end, const double t_resolution);

private:
    //设置图上障碍物
    void setupObstacles(const std::vector<Obstacle>& obstacles,const ReferenceLine &ref_line);

    //将静态障碍物转换到ST图中，得到其4个顶点的st值，并保留障碍物sl范围（？）
    void setStaticObstacle(const Obstacle& obstacle,const ReferenceLine& ref_line);

    void setDynamicObstacle(const Obstacle& obstacles, const ReferenceLine& ref_line);

    //bool computeObstacleBoundary(const std::vector<CartesianPoint>& vertices,const ReferenceLine& ref_line,SLBoundary& sl_boundary) const;

    PathTimePoint setPathTimePoint(const uint32_t obstacle_id, const double s, const double t);

    //(需要修改)跟车
    void queryFollowPathTimePoints(const uint32_t obstacle_id, std::vector<SamplePoint>* sample_points);

    //(需要修改)超车
    void queryOvertakePathTimePoints(const uint32_t obstacle_id, std::vector<SamplePoint>* sample_points);

    //删除不合理点
    void handleSamplingPoints(std::vector<SamplePoint>& sample_points, const FrenetPoint& fre_point);

    std::vector< std::pair<double, double> > getPathBlockingIntervals(const double t);


    std::pair<double, double> time_range_;
    std::pair<double, double> path_range_;
    //const ReferenceLineInfo* ptr_reference_line_info_;
    //std::array<double, 3> init_d_;
    std::unordered_map<uint32_t, PathTimeObstacle> path_time_obstacle_map_;

    //std::unordered_map<uint32_t, Obstacle> obstacle_map_;
    std::vector<PathTimeObstacle> path_time_obstacles_;
    //静态障碍物sl范围
    //std::vector<SLBoundary> static_obs_sl_boundaries_;

    double NormalizeAngle(const double angle)
    {
        double a = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (a < 0.0) {
            a += (2.0 * M_PI);
        }
        return a - M_PI;
    }

    double slerp(const double a0, const double t0, const double a1, const double t1,const double t)
    {
        if (std::abs(t1 - t0) <= 1.0e-6) {
            //AERROR << "input time difference is too small";
            return NormalizeAngle(a0);
        }
        const double a0_n = NormalizeAngle(a0);
        const double a1_n = NormalizeAngle(a1);
        double d = a1_n - a0_n;
        if (d > M_PI) {
            d = d - 2 * M_PI;
        } else if (d < -M_PI) {
            d = d + 2 * M_PI;
        }

        const double r = (t - t0) / (t1 - t0);
        const double a = a0_n + d * r;
        return NormalizeAngle(a);
    }
};

} // namespace planning

#endif // PATH_TIME_GRAPH_H_
