#ifndef GET_BOUNDARY_BY_CARTESIAN_LOCATION_H
#define GET_BOUNDARY_BY_CARTESIAN_LOCATION_H

//2019-5-28 新增 by rain.wei，用来用定位计算车体包络的四个顶点

#include "common/utils.h"
#include "math/cartesian_frenet_converter.h"

namespace planning
{
static std::vector<CartesianPoint> getBoundaryByCartesianLocation(CartesianPoint& point)
{
    double car_left = point.getX() - CAR_WIDTH/2;
    double car_upper = point.getY() + CAR_LENGTH/2;

    double car_right = point.getX() + CAR_WIDTH/2;
    double car_lower = point.getY() - CAR_LENGTH/2;

    //rectangle rotation formula according to center point
    //x2 = (x1 - x0) * cosa - (y1 - y0) * sina + x0
    //y2 = (y1 - y0) * cosa + (x1 - x0) * sina + y0

    double upper_left_corner_x = (car_left - point.getX())*cos(point.getTheta())
            -(car_upper - point.getY())*sin(point.getTheta())+point.getX();

    double upper_left_corner_y =(car_upper - point.getY())*cos(point.getTheta())
            +(car_left - point.getX())*sin(point.getTheta())+point.getY();

    double lower_left_corner_x =(car_left - point.getX())*cos(point.getTheta())
            -(car_lower - point.getY())*sin(point.getTheta())+point.getX();

    double lower_left_corner_y =(car_lower - point.getY())*cos(point.getTheta())
            +(car_left - point.getX())*sin(point.getTheta())+point.getY();

    double lower_right_corner_x =(car_right - point.getX())*cos(point.getTheta())
            -(car_lower - point.getY())*sin(point.getTheta())+point.getX();

    double lower_right_corner_y =(car_lower - point.getY())*cos(point.getTheta())
            +(car_right - point.getX())*sin(point.getTheta())+point.getY();

    double upper_right_corner_x =(car_right - point.getX())*cos(point.getTheta())
            -(car_upper - point.getY())*sin(point.getTheta())+point.getX();

    double upper_right_corner_y =(car_upper - point.getY())*cos(point.getTheta())
            +(car_right - point.getX())*sin(point.getTheta())+point.getY();

    std::vector<CartesianPoint> boundary;
    boundary.push_back(CartesianPoint(upper_left_corner_x,upper_left_corner_y,point.getTheta(),point.getKappa(),point.getVel(),point.getAcc(),point.getRelativeTime()));
    boundary.push_back(CartesianPoint(lower_left_corner_x,lower_left_corner_y,point.getTheta(),point.getKappa(),point.getVel(),point.getAcc(),point.getRelativeTime()));
    boundary.push_back(CartesianPoint(lower_right_corner_x,lower_right_corner_y,point.getTheta(),point.getKappa(),point.getVel(),point.getAcc(),point.getRelativeTime()));
    boundary.push_back(CartesianPoint(upper_right_corner_x,upper_right_corner_y,point.getTheta(),point.getKappa(),point.getVel(),point.getAcc(),point.getRelativeTime()));

    //CartesianPoint(double x,double y,double theta,double kappa,double vel,double acc,double relative_time);

    return boundary;
}

static std::vector<FrenetPoint> getFrenetPointsBoundary(std::vector<CartesianPoint>& boundary,ReferenceLine& ref_line)
{
    std::vector<FrenetPoint> frenet_points;
    for(CartesianPoint car_point : boundary)
    {
        FrenetPoint fre_point;
        if (!cartesianToFrenet(car_point,ref_line,fre_point))//change in 2019-6-14 by rain.wei
        {
            ROS_WARN("GET_BOUNDARY:Cartesian location transform to frenet point failed.");
            //return std::vector<FrenetPoint>;
        }
        else
            frenet_points.push_back(fre_point);

    }
    return frenet_points;
}
}

#endif // GET_BOUNDARY_BY_CARTESIAN_LOCATION_H
