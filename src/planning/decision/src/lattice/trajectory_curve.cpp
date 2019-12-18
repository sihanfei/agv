
#include "utils.h"

namespace pnc 
{

TrajectoryCurve::TrajectoryCurve(std::shared_ptr<Curve> ptr_trajectory_curve)
{
	ptr_trajectory_curve_ = ptr_trajectory_curve;
}

double TrajectoryCurve::evaluate(uint8_t mode, double param) const
{

	double param_max = ptr_trajectory_curve_->paramMax();
	if (param < param_max + EPSILON)
	{
		return ptr_trajectory_curve_->evaluate(mode,param);
	}
	double p = ptr_trajectory_curve_->getx1();
	double v = ptr_trajectory_curve_->getdx1();
	double a = ptr_trajectory_curve_->getddx1();
	double t = param - param_max;
	switch (mode)
	{
		case 0:
			return p + v*t + 0.5*a*t*t;
		case 1:
			return v + a*t;
  	case 2:
			return a;
		default:
			return 0.0;
  }

/*
	// 通过多项式系数来得到曲线的插值点 20190917
	uint8_t order = ptr_trajectory_curve_->getOrder();						// 获取曲线的阶数
	std::vector<double> coef_ = ptr_trajectory_curve_->getCoef();	// 获取曲线的系数

	if(order == 5)
	{
		switch (mode)
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

			default:
				return 0.0;
		}
	}
	else if(order == 4)
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

			default:
				return 0.0;
		}
	}
	else
	{
		ROS_WARN("TrajectoryCurve : parameter error, order is invalid.");
	}
*/

}

double TrajectoryCurve::evaluateNormalization(uint8_t mode, double param) const
{
	double param_max = ptr_trajectory_curve_->paramMax();
  if (param < param_max + EPSILON)
  {
      return ptr_trajectory_curve_->evaluateNormalization(mode,param);
  }
  //There are mostly useless right now
  //该函数用作评分计算，外推逻辑用不到
  double p = ptr_trajectory_curve_->getx1();
  double v = ptr_trajectory_curve_->getdx1();
  double a = ptr_trajectory_curve_->getddx1();
  double t = param - param_max;
  switch (mode)
  {
		case 0:
			return p + v*t + 0.5*a*t*t;
		case 1:
			return v + a*t;
		case 2:
			return a;
		default:
			return 0.0;
  }

/*
	// 通过多项式技术来得到曲线的插值点 20190917
	uint8_t order = ptr_trajectory_curve_->getOrder();						// 获取曲线的阶数
	std::vector<double> coef_ = ptr_trajectory_curve_->getCoef();	// 获取曲线的系数

	if(order == 5)
	{
		double sum_coef_ = fabs(coef_[0]) + fabs(coef_[1]) + fabs(coef_[2]) + fabs(coef_[3]) + fabs(coef_[4]) + fabs(coef_[5]);

		switch (mode)
		{
			case 0: // s
				return (coef_[0] + coef_[1] * param + coef_[2] * pow(param, 2) + coef_[3] * pow(param, 3) +
				        coef_[4] * pow(param, 4) + coef_[5] * pow(param, 5)) / (sum_coef_ + EPSILON);

			case 1: // ds
				return (coef_[1] + 2 * coef_[2] * param + 3 * coef_[3] * pow(param, 2) + 4 * coef_[4] * pow(param, 3) +
				        5 * coef_[5] * pow(param, 4)) /(sum_coef_ + EPSILON);

			case 2: // dds
				return (2 * coef_[2] + 6 * coef_[3] * param + 12 * coef_[4] * pow(param, 2) + 20 * coef_[5] * pow(param, 3)) / (sum_coef_ + EPSILON);

			case 3: // ddds
				return (6 * coef_[3] + 24 * coef_[4] * param + 60 * coef_[5] * pow(param, 2)) / (sum_coef_ + EPSILON);

			default:
				return 0.0;
		}
	}
	else if(order == 4)
	{
		double sum_coef_ = fabs(coef_[0]) + fabs(coef_[1]) + fabs(coef_[2]) + fabs(coef_[3]) + fabs(coef_[4]);	

		switch (mode)
		{
			case 0: // s
				return (coef_[0] + coef_[1] * param + coef_[2] * pow(param, 2) + coef_[3] * pow(param, 3) +
				        coef_[4] * pow(param, 4)) / (sum_coef_ + EPSILON);

			case 1: // ds
				return (coef_[1] + 2 * coef_[2] * param + 3 * coef_[3] * pow(param, 2) + 4 * coef_[4] * pow(param, 3)) / (sum_coef_ + EPSILON);

			case 2: // dds
				return (2 * coef_[2] + 6 * coef_[3] * param + 12 * coef_[4] * pow(param, 2)) / (sum_coef_ + EPSILON);

			case 3: // ddds
				return (6 * coef_[3] + 24 * coef_[4] * param) / (sum_coef_ + EPSILON);

			default:
				return 0.0;
		}
	}
	else
	{
		ROS_WARN("TrajectoryCurve : parameter error, order is invalid.");
	}
*/

}

double TrajectoryCurve::paramLength() const
{
    return ptr_trajectory_curve_->paramLength();
}

double TrajectoryCurve::paramMax() const
{
    return ptr_trajectory_curve_->paramMax();
}

double TrajectoryCurve::paramMin() const
{
    return ptr_trajectory_curve_->paramMin();
}

void TrajectoryCurve::setCost(double cost)
{
    ptr_trajectory_curve_->setCost(cost);
}

double TrajectoryCurve::getCost() const
{
    return ptr_trajectory_curve_->getCost();
}

double TrajectoryCurve::getx1() const
{
    return ptr_trajectory_curve_->getx1();
}

double TrajectoryCurve::getdx1() const
{
    return ptr_trajectory_curve_->getdx1();
}
double TrajectoryCurve::getddx1() const
{
    return ptr_trajectory_curve_->getddx1();
}

double TrajectoryCurve::getx0() const
{
    return ptr_trajectory_curve_->getx0();
}
double TrajectoryCurve::getdx0() const
{
    return ptr_trajectory_curve_->getdx0();
}
double TrajectoryCurve::getddx0() const
{
    return ptr_trajectory_curve_->getddx0();
}


bool TrajectoryCurve::isValid()
{
	ptr_trajectory_curve_->isValid();
}



// 返回曲线轨迹的阶数  20190917
uint8_t TrajectoryCurve::getOrder() const
{
	return ptr_trajectory_curve_->getOrder();
}
// 获取曲线轨迹的多项式系数  20190917
std::vector<double> TrajectoryCurve::getCoef() const
{
	return ptr_trajectory_curve_->getCoef();
}

}




