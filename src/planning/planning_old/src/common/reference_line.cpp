#include "common/reference_line.h"
#include "common/utils.h"

// class ReferenceLinePoint
namespace planning
{

ReferenceLinePoint::ReferenceLinePoint(double s, double x, double y, double theta, double kappa, double dkappa)
    : s_(s), x_(x), y_(y), theta_(theta), kappa_(kappa), dkappa_(dkappa)
{
}

ReferenceLinePoint::ReferenceLinePoint(double s, double x, double y, double theta, double kappa, double dkappa,
                                       double max_speed, std::vector< LaneRange > lane_ranges)
    : s_(s), x_(x), y_(y), theta_(theta), kappa_(kappa), dkappa_(dkappa), max_speed_(max_speed),
      lane_ranges_(lane_ranges)
{
  width_left_  = -lane_ranges_.front().left_boundary_;
  width_right_ = lane_ranges_.back().right_boundary_;
  // width_left_  = 50.0;
  // width_right_ = 50.0;
}

void ReferenceLinePoint::setS(double s)
{
  s_ = s;
}

void ReferenceLinePoint::setX(double x)
{
  x_ = x;
}

void ReferenceLinePoint::setY(double y)
{
  y_ = y;
}

void ReferenceLinePoint::setTheta(double theta)
{
  theta_ = theta;
}

void ReferenceLinePoint::setKappa(double kappa)
{
  kappa_ = kappa;
}

void ReferenceLinePoint::setdKappa(double dkappa)
{
  dkappa_ = dkappa;
}

void ReferenceLinePoint::setMaxSpeed(double max_speed)
{
  max_speed_ = max_speed;
}

void ReferenceLinePoint::setdWidthLeft(double width_left)
{
  width_left_ = width_left;
}
void ReferenceLinePoint::setdWidthRight(double width_right)
{
  width_right_ = width_right;
}

void ReferenceLinePoint::setLaneRanges(std::vector< LaneRange > lane_ranges)
{
  lane_ranges_ = lane_ranges;
}

double ReferenceLinePoint::getS() const
{
  return s_;
}

double ReferenceLinePoint::getX() const
{
  return x_;
}

double ReferenceLinePoint::getY() const
{
  return y_;
}

double ReferenceLinePoint::getTheta() const
{
  return theta_;
}
double ReferenceLinePoint::getKappa() const
{
  return kappa_;
}
double ReferenceLinePoint::getdKappa() const
{
  return dkappa_;
}

double ReferenceLinePoint::getMaxSpeed() const
{
  return max_speed_;
}

double ReferenceLinePoint::getdWidthLeft() const
{
  return width_left_;
}

double ReferenceLinePoint::getdWidthRight() const
{
  return width_right_;
}

std::vector< LaneRange > ReferenceLinePoint::getLaneRanges() const
{
  return lane_ranges_;
}

// class ReferenceLine
std::vector< ReferenceLinePoint > ReferenceLine::getReferenceLinePoints() const
{
  return ref_line_points_;
}

void ReferenceLine::setReferenceLinePoints(const std::vector< ReferenceLinePoint > ref_line_points)
{
  ref_line_points_ = ref_line_points;
}

void ReferenceLine::addReferencePoint(ReferenceLinePoint ref_point)
{
  ref_line_points_.push_back(ref_point);
}

void ReferenceLine::clearReferencePoints()
{
  ref_line_points_.clear();
}

double ReferenceLine::getReferenceLineMaxS() const
{
  return ref_line_points_.back().getS();
}

uint32_t ReferenceLine::getReferenceLinePointsSize() const
{
  return ref_line_points_.size();
}

ReferenceLinePoint ReferenceLine::getReferenceLinePointByIndex(uint32_t index) const
{
  // ReferenceLinePoint ref_line_point = ref_line_points_[index];
  return ref_line_points_[index];
}

ReferenceLinePoint ReferenceLine::getReferenceLinePointByS(double s) const
{
  if (s < EPSILON)
    return ref_line_points_[0];

  // binary search
  uint32_t left  = 0;
  uint32_t right = ref_line_points_.size() - 2;

  while (left <= right)
  {
    uint32_t mid = (left + right) * 0.5;

    if (ref_line_points_[mid].getS() < s && ref_line_points_[mid + 1].getS() >= s)
    {
      return ref_line_points_[mid + 1];
    }
    if (ref_line_points_[mid].getS() >= s)
    {
      right = mid - 1;
    }
    if (ref_line_points_[mid + 1].getS() < s)
    {
      left = mid + 1;
    }
  }

  //  for(int i=0;i<ref_line_points_.size()-1;++i)
  //  {
  //    if(ref_line_points_[i].getS() < s && ref_line_points_[i+1].getS() >= s)
  //    {
  //        return ref_line_points_[i+1];
  //    }
  //  }
}

void ReferenceLine::CalculateMaxSpeedInterval()
{
  // double last_
  last_max_speed_ = std::numeric_limits< double >::max();

  for (ReferenceLinePoint ref_point : ref_line_points_)
  {
    if (fabs(last_max_speed_ - ref_point.getMaxSpeed()) > EPSILON)
    {
      last_max_speed_                       = ref_point.getMaxSpeed();
      max_speed_interval_[ref_point.getS()] = ref_point.getMaxSpeed();
    }
  }
}

std::map< double, double > ReferenceLine::getMaxSpeedInterval() const
{
  return max_speed_interval_;
}

void ReferenceLine::clearMaxSpeedIntervalMap()
{
  max_speed_interval_.clear();
}

ReferenceLinePoint ReferenceLine::getNearestRefLinePoint(double x, double y)
{
  uint32_t index_min = 0;
  double dis_min     = std::numeric_limits< double >::max();

  for (uint32_t i = 0; i < ref_line_points_.size(); ++i)
  {
    ReferenceLinePoint ref_line_point = ref_line_points_[i];
    double dis                        = sqrt(pow(ref_line_point.getX() - x, 2) + pow(ref_line_point.getY() - y, 2));

    if (dis < dis_min)
    {
      dis_min   = dis;
      index_min = i;
    }
  }
  return ref_line_points_[index_min];
}

} // end namespace
