#include "utils.h"


namespace pnc
{

// 参考点
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



// 获取参考线上的点
std::vector< ReferenceLinePoint > ReferenceLine::getReferenceLinePoints() const
{
  return ref_line_points_;
}

// 设置参考线上的参考点
void ReferenceLine::setReferenceLinePoints(const std::vector< ReferenceLinePoint > ref_line_points)
{
  ref_line_points_ = ref_line_points;
}

// 在参考线上增加一个参考点
void ReferenceLine::addReferencePoint(ReferenceLinePoint ref_point)
{
  ref_line_points_.push_back(ref_point);
}

// 清除参考线上所有的参考点信息
void ReferenceLine::clearReferencePoints()
{
  ref_line_points_.clear();
}

// 获取参考线上最后一个点的s值
double ReferenceLine::getReferenceLineMaxS() const
{
  return ref_line_points_.back().getS();
}

// 获取参考线总点数
uint32_t ReferenceLine::getReferenceLinePointsSize() const
{
  return ref_line_points_.size();
}

// 通过索引号获取参考线上对应的参考点信息
ReferenceLinePoint ReferenceLine::getReferenceLinePointByIndex(uint32_t index) const
{
  return ref_line_points_[index];
}

// 通过s值获取参考线上相应的参考点信息
ReferenceLinePoint ReferenceLine::getReferenceLinePointByS(double s) const
{
  if (s < EPSILON)
    return ref_line_points_[0];

  // binary search
  uint32_t left  = 0;
  uint32_t right = ref_line_points_.size() - 2;

  uint32_t num  = ref_line_points_.size();

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
}

// 根据XY坐标获取参考上相应的参考点信息
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

