#ifndef REFERENCELINE_H
#define REFERENCELINE_H

#include <vector>
#include <inttypes.h>
#include <map>

namespace planning {

struct LaneRange
{
    double left_boundary_;
    double right_boundary_;
};

class ReferenceLinePoint
{
public:
    ReferenceLinePoint()=default;
    ~ReferenceLinePoint()=default;

    ReferenceLinePoint(double s,double x,double y,double theta,double kappa,double dkappa);
    ReferenceLinePoint(double s,double x,double y,double theta,double kappa,double dkappa,double max_speed,std::vector<LaneRange> lane_ranges);

    double getS() const;
    double getX() const;
    double getY() const;
    double getTheta() const;
    double getKappa() const;
    double getdKappa() const;
    double getMaxSpeed() const;
    double getdWidthLeft() const;
    double getdWidthRight() const;
    std::vector<LaneRange> getLaneRanges() const;

    void setS(double s);
    void setX(double x);
    void setY(double y);
    void setTheta(double theta);
    void setKappa(double kappa);
    void setdKappa(double dkappa);
    void setMaxSpeed(double max_speed);
    void setdWidthLeft(double width_left);
    void setdWidthRight(double width_right);
    void setLaneRanges(std::vector<LaneRange> lane_ranges);

private:
    double s_;
    double x_;
    double y_;
    double theta_;
    double kappa_;
    double dkappa_;
    double max_speed_;
    double width_left_;
    double width_right_;

    std::vector<LaneRange> lane_ranges_;
};

class ReferenceLine
{
public:
    ReferenceLine()=default;
    ~ReferenceLine()=default;

    double getReferenceLineMaxS() const;

    std::vector<ReferenceLinePoint> getReferenceLinePoints() const;
    void setReferenceLinePoints(const std::vector<ReferenceLinePoint> ref_line_points);

    void addReferencePoint(ReferenceLinePoint ref_point);
    void clearReferencePoints();

    uint32_t getReferenceLinePointsSize() const;

    ReferenceLinePoint getReferenceLinePointByIndex(uint32_t index) const;

    ReferenceLinePoint getReferenceLinePointByS(double s) const;

    void CalculateMaxSpeedInterval();
    std::map <double,double> getMaxSpeedInterval() const;
    void clearMaxSpeedIntervalMap();

    ReferenceLinePoint getNearestRefLinePoint(double x,double y);

private:

    double last_max_speed_;

    std::vector<ReferenceLinePoint> ref_line_points_;

    std::map <double,double> max_speed_interval_;
};
}//end namespace

#endif // REFERENCELINE_H
