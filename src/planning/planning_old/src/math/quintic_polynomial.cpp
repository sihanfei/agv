#include "math/quintic_polynomial.h"

namespace planning
{

QuinticPolynomial::QuinticPolynomial(const double x0, const double dx0, const double ddx0, const double x1,
                                     const double dx1, const double ddx1, const double param0, const double param1)
    : x0_(x0), dx0_(dx0), ddx0_(ddx0), x1_(x1), dx1_(dx1), ddx1_(ddx1), param0_(param0), param1_(param1)
{
  computeCoefficients(x0_, dx0_, ddx0_, x1_, dx1_, ddx1_, param0_, param1_);
  // ROS_INFO("QuinticPolynomial:a1=%f,a2=%f,a3=%f,a4=%f,a5=%f,a6=%f",coef_[0],coef_[1],coef_[2],coef_[3],coef_[4],coef_[5]);
  sum_coef_ = fabs(coef_[0]) + fabs(coef_[1]) + fabs(coef_[2]) + fabs(coef_[3]) + fabs(coef_[4]) + fabs(coef_[5]);
}

double QuinticPolynomial::evaluate(uint8_t order, double param) const
{
  switch (order)
  {
  case 0: // s
    return coef_[0] + coef_[1] * param + coef_[2] * pow(param, 2) + coef_[3] * pow(param, 3) +
           coef_[4] * pow(param, 4) + coef_[5] * pow(param, 5);

  case 1: // ds
    return coef_[1] + 2 * coef_[2] * param + 3 * coef_[3] * pow(param, 2) + 4 * coef_[4] * pow(param, 3) +
           5 * coef_[5] * pow(param, 4);

  case 2: // dds
    return 2 * coef_[2] + 6 * coef_[3] * param + 12 * coef_[4] * pow(param, 2) + 20 * coef_[5] * pow(param, 3);

  case 3: // ddds
    return 6 * coef_[3] + 24 * coef_[4] * param + 60 * coef_[5] * pow(param, 2);

  case 4: // ddds^2的不定积分
    return 36 * (coef_[3] * coef_[3] * param + 4 * coef_[3] * coef_[4] * pow(param, 2) +
                 (20 * coef_[3] * coef_[5] + 16 * coef_[4] * coef_[4]) * pow(param, 3) / 3 +
                 20 * coef_[4] * coef_[5] * pow(param, 4) + 20 * coef_[5] * coef_[5] * pow(param, 5));

  default:
    return 0.0;
  }
}

double QuinticPolynomial::evaluateNormalization(uint8_t order, double param) const
{
  switch (order)
  {
  case 0: // s
    return (coef_[0] + coef_[1] * param + coef_[2] * pow(param, 2) + coef_[3] * pow(param, 3) +
            coef_[4] * pow(param, 4) + coef_[5] * pow(param, 5)) /
           (sum_coef_ + EPSILON);

  case 1: // ds
    return (coef_[1] + 2 * coef_[2] * param + 3 * coef_[3] * pow(param, 2) + 4 * coef_[4] * pow(param, 3) +
            5 * coef_[5] * pow(param, 4)) /
           (sum_coef_ + EPSILON);

  case 2: // dds
    return (2 * coef_[2] + 6 * coef_[3] * param + 12 * coef_[4] * pow(param, 2) + 20 * coef_[5] * pow(param, 3)) /
           (sum_coef_ + EPSILON);

  case 3: // ddds
    return (6 * coef_[3] + 24 * coef_[4] * param + 60 * coef_[5] * pow(param, 2)) / (sum_coef_ + EPSILON);

  case 4: // ddds^2的不定积分
    return (36 * (coef_[3] * coef_[3] * param + 4 * coef_[3] * coef_[4] * pow(param, 2) +
                  (20 * coef_[3] * coef_[5] + 16 * coef_[4] * coef_[4]) * pow(param, 3) / 3 +
                  20 * coef_[4] * coef_[5] * pow(param, 4) + 20 * coef_[5] * coef_[5] * pow(param, 5))) /
           (sum_coef_ + EPSILON);

  default:
    return 0.0;
  }
}

double QuinticPolynomial::paramLength() const
{
  return param1_ - param0_;
}

double QuinticPolynomial::paramMax() const
{
  return param1_;
}

double QuinticPolynomial::paramMin() const
{
  return param0_;
}

double QuinticPolynomial::getx1() const
{
  return x1_;
}

double QuinticPolynomial::getdx1() const
{
  return dx1_;
}
double QuinticPolynomial::getddx1() const
{
  return ddx1_;
}

double QuinticPolynomial::getx0() const
{
  return x0_;
}
double QuinticPolynomial::getdx0() const
{
  return dx0_;
}
double QuinticPolynomial::getddx0() const
{
  return ddx0_;
}

void QuinticPolynomial::computeCoefficients(double x0, double dx0, double ddx0, double x1, double dx1, double ddx1,
                                            double s0, double s1)
{
  Eigen::Matrix< double, 6, 6 > A;

  A << 1, s0, pow(s0, 2), pow(s0, 3), pow(s0, 4), pow(s0, 5), 0, 1, 2 * s0, 3 * pow(s0, 2), 4 * pow(s0, 3),
      5 * pow(s0, 4), 0, 0, 2, 6 * s0, 12 * pow(s0, 2), 20 * pow(s0, 3), 1, s1, pow(s1, 2), pow(s1, 3), pow(s1, 4),
      pow(s1, 5), 0, 1, 2 * s1, 3 * pow(s1, 2), 4 * pow(s1, 3), 5 * pow(s1, 4), 0, 0, 2, 6 * s1, 12 * pow(s1, 2),
      20 * pow(s1, 3);

  Eigen::Matrix< double, 6, 1 > b;
  b << x0, dx0, ddx0, x1, dx1, ddx1;

  Eigen::Matrix< double, 6, 1 > x = A.lu().solve(b);

  std::vector< double > vec(x.data(), x.data() + x.rows() * x.cols());
  coef_ = vec;
}

void QuinticPolynomial::setCost(double cost)
{
  cost_ = cost;
}

double QuinticPolynomial::getCost() const
{
  return cost_;
}

void QuinticPolynomial::setTrajInfo(std::string info)
{
  info_ = info;
}

void QuinticPolynomial::displayTrajInfo()
{
  ROS_INFO(info_.c_str());
}

// void QuinticPolynomial::setRestriction(Restriction& restriction)
// {
//     restriction_ = restriction;
// }

// Restriction QuinticPolynomial::getRestriction() const
// {
//     return restriction_;
// }
}
