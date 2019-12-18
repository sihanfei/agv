#include "math/quartic_polynomial.h"

namespace planning
{

QuarticPolynomial::QuarticPolynomial(const double x0, const double dx0, const double ddx0, const double dx1,
                                     const double ddx1, const double param0, const double param1)
    : x0_(x0), dx0_(dx0), ddx0_(ddx0), dx1_(dx1), ddx1_(ddx1), param0_(param0), param1_(param1)
{
  computeCoefficients(x0_, dx0_, ddx0_, dx1_, ddx1_, param0_, param1_);
  sum_coef_ = fabs(coef_[0]) + fabs(coef_[1]) + fabs(coef_[2]) + fabs(coef_[3]) + fabs(coef_[4]);
  x1_       = coef_[0] + coef_[1] * param1_ + coef_[2] * pow(param1_, 2) + coef_[3] * pow(param1_, 3) +
        coef_[4] * pow(param1_, 4);
}

double QuarticPolynomial::evaluate(uint8_t mode, double param) const
{
  switch (mode)
  {
  case 0: // s
    return coef_[0] + coef_[1] * param + coef_[2] * pow(param, 2) + coef_[3] * pow(param, 3) + coef_[4] * pow(param, 4);

  case 1: // ds
    return coef_[1] + 2 * coef_[2] * param + 3 * coef_[3] * pow(param, 2) + 4 * coef_[4] * pow(param, 3);

  case 2: // dds
    return 2 * coef_[2] + 6 * coef_[3] * param + 12 * coef_[4] * pow(param, 2);

  case 3: // ddds
    return 6 * coef_[3] + 24 * coef_[4] * param;

  case 4: // ddds^2的不定积分
    return 36 * (coef_[3] * coef_[3] * param + 4 * coef_[3] * coef_[4] * pow(param, 2) +
                 16 * coef_[4] * coef_[4] * pow(param, 3) / 3);

  default:
    return 0.0;
  }
}

double QuarticPolynomial::evaluateNormalization(uint8_t mode, double param) const
{
  switch (mode)
  {
  case 0: // s
    return (coef_[0] + coef_[1] * param + coef_[2] * pow(param, 2) + coef_[3] * pow(param, 3) +
            coef_[4] * pow(param, 4)) /
           (sum_coef_ + EPSILON);

  case 1: // ds
    return (coef_[1] + 2 * coef_[2] * param + 3 * coef_[3] * pow(param, 2) + 4 * coef_[4] * pow(param, 3)) /
           (sum_coef_ + EPSILON);

  case 2: // dds
    return (2 * coef_[2] + 6 * coef_[3] * param + 12 * coef_[4] * pow(param, 2)) / (sum_coef_ + EPSILON);

  case 3: // ddds
    return (6 * coef_[3] + 24 * coef_[4] * param) / (sum_coef_ + EPSILON);

  case 4: // ddds^2的不定积分
    return (36 * (coef_[3] * coef_[3] * param + 4 * coef_[3] * coef_[4] * pow(param, 2) +
                  16 * coef_[4] * coef_[4] * pow(param, 3) / 3)) /
           (sum_coef_ + EPSILON);

  default:
    return 0.0;
  }
}

double QuarticPolynomial::paramLength() const
{
  return param1_ - param0_;
}

double QuarticPolynomial::paramMax() const
{
  return param1_;
}

double QuarticPolynomial::paramMin() const
{
  return param0_;
}

double QuarticPolynomial::getx1() const
{
  return x1_;
}

double QuarticPolynomial::getdx1() const
{
  return dx1_;
}

double QuarticPolynomial::getddx1() const
{
  return ddx1_;
}

double QuarticPolynomial::getx0() const
{
  return x0_;
}

double QuarticPolynomial::getdx0() const
{
  return dx0_;
}

double QuarticPolynomial::getddx0() const
{
  return ddx0_;
}

void QuarticPolynomial::computeCoefficients(double x0, double dx0, double ddx0, double dx1, double ddx1, double t0,
                                            double t1)
{
  Eigen::Matrix< double, 5, 5 > A;

  A << 1, t0, pow(t0, 2), pow(t0, 3), pow(t0, 4), 0, 1, 2 * t0, 3 * pow(t0, 2), 4 * pow(t0, 3), 0, 0, 2, 6 * t0,
      12 * pow(t0, 2), 0, 1, 2 * t1, 3 * pow(t1, 2), 4 * pow(t1, 3), 0, 0, 2, 6 * t1, 12 * pow(t1, 2);

  Eigen::Matrix< double, 5, 1 > b;

  b << x0, dx0, ddx0, dx1, ddx1;

  // Eigen::VectorXd B = A.lu().solve(b);
  Eigen::Matrix< double, 5, 1 > x = A.lu().solve(b);
  std::vector< double > vec(x.data(), x.data() + x.rows() * x.cols());
  coef_ = vec;
}

void QuarticPolynomial::setCost(double cost)
{
  cost_ = cost;
}

double QuarticPolynomial::getCost() const
{
  return cost_;
}

void QuarticPolynomial::setTrajInfo(std::string info)
{
  info_ = info;
}

void QuarticPolynomial::displayTrajInfo()
{
  ROS_INFO(info_.c_str());
}

// void QuarticPolynomial::setRestriction(Restriction& restriction)
// {
//     restriction_ = restriction;
// }

// Restriction QuarticPolynomial::getRestriction() const
// {
//     return restriction_;
// }

} // end namespace
