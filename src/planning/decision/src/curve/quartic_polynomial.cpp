#include "utils.h"

namespace pnc
{

QuarticPolynomial::QuarticPolynomial(const double x0, const double dx0, const double ddx0, const double dx1,
                                     const double ddx1, const double param0, const double param1)
    : x0_(x0), dx0_(dx0), ddx0_(ddx0), dx1_(dx1), ddx1_(ddx1), param0_(param0), param1_(param1)
{
	// 计算四次多项式多项式系数
  computeCoefficients(x0_, dx0_, ddx0_, dx1_, ddx1_, param0_, param1_);	

/*
	// 方程法求四次多项式多项式系数
	ComputeCoefficients_Equation(x0_, dx0_, ddx0_, dx1_, ddx1_, param0_, param1_);	
	ROS_INFO("QuarticPolynomial:Coefficients Difference da0 = %f, da1 = %f, da2 = %f, da3 = %f, da4 = %f",coef_[0] - coef_test[0],coef_[1] - coef_test[1],coef_[2] - coef_test[2],coef_[3] - coef_test[3],coef_[4] - coef_test[4]);
*/

  sum_coef_ = fabs(coef_[0]) + fabs(coef_[1]) + fabs(coef_[2]) + fabs(coef_[3]) + fabs(coef_[4]);	// 多项式系数之和，便于归一化

  x1_       = coef_[0] + coef_[1] * param1_ + coef_[2] * pow(param1_, 2) + coef_[3] * pow(param1_, 3) + coef_[4] * pow(param1_, 4);

	is_valid_ = true;	  

	// 检查多项式系数，若有系数为无穷大或者NAN，则该多项式曲线无效
	for (double c : coef_)
	{
		if(std::isnan(c)||std::isinf(c))
	 	{
			is_valid_ = false;

			ROS_INFO("QuarticPolynomial:Trajectory Coefficients is nan or inf .");

			break;
		}
	}

	order_ = 4;

}


/*
void QuarticPolynomial::ComputeCoefficients_Equation(double x0, double dx0, double ddx0, double dx1, double ddx1, double s0, double s1)
{
	// 利用方程法求解四次多项式系数
	double delta_dp = dx1 -dx0;
	double delta_ddp = ddx1 -ddx0;

	double T1 = s1 - s0;
	double T2 = pow(s1, 2) - pow(s0, 2);
	double T3 = pow(s1, 3) - pow(s0, 3);


	double N = delta_dp - T1*ddx0;
	double M1 = 3*T2 - 6*T1*s0;
	double M2 = 4*T3 - 12*T1*pow(s0, 2);


	double a4 = (M2 - 2*T2/T1*M1)/(N - delta_ddp/(6*T1)*M1);
	double a3 = (delta_ddp - 12*T2*a4)/(6*T1);
	double a2 = (ddx0 - 6*s0*a3 - 12*pow(s0, 2)*a4)/2;
	double a1 = dx0 - 2*s0*a2 - 3*pow(s0, 2)*a3 - 4*pow(s0, 3)*a4;
	double a0 = x0 - s0*a1 - pow(s0, 2)*a2 - pow(s0, 3)*a3 - pow(s0, 4)*a4;

	coef_.push_back(a0);
	coef_.push_back(a1);
	coef_.push_back(a2);
	coef_.push_back(a3);
	coef_.push_back(a4);

}

*/
void QuarticPolynomial::computeCoefficients(double x0, double dx0, double ddx0, double dx1, double ddx1, double t0, double t1)
{
	// 计算四次多项式多项式系数
/*
  Eigen::Matrix< double, 5, 5 > A;

  A << 1, t0, pow(t0, 2), pow(t0, 3), pow(t0, 4), 0, 1, 2 * t0, 3 * pow(t0, 2), 4 * pow(t0, 3), 0, 0, 2, 6 * t0,
      12 * pow(t0, 2), 0, 1, 2 * t1, 3 * pow(t1, 2), 4 * pow(t1, 3), 0, 0, 2, 6 * t1, 12 * pow(t1, 2);

  Eigen::Matrix< double, 5, 1 > b;

  b << x0, dx0, ddx0, dx1, ddx1;

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
	svd.singularValues();
	svd.matrixU();
	svd.matrixV();
	Eigen::Matrix< double, 5, 1 > x = svd.solve(b);


  // Eigen::VectorXd B = A.lu().solve(b);
  //Eigen::Matrix< double, 5, 1 > x = A.lu().solve(b);
	//Eigen::Matrix< double, 5, 1 > x = A.svd().solve(b);
  std::vector< double > vec(x.data(), x.data() + x.rows() * x.cols());


  coef_ = vec;

*/

	double p = t1 - t0;
	std::vector<double> vec(5); 
	if (p <= 0.0)
	{ 
		vec[0]=0.0;
		vec[1]=0.0; 
		vec[2]=0.0; 
		vec[3]=0.0; 
		vec[4]=0.0; 
		coef_ = vec; 
		return; 
	} 
	vec[0] = x0; 
	vec[1] = dx0; 
	vec[2] = 0.5 * ddx0; 
	const double b0 = dx1 - ddx0 * p - dx0; 
	const double b1 = ddx1 - ddx0; 
	const double p2 = p * p; 
	const double p3 = p2 * p; 
	vec[3] = (3.0 * b0 - b1 * p) / (3.0 * p2); 
	vec[4] = (-2.0 * b0 + b1 * p) / (4.0 * p3);

	coef_ = vec;

}


double QuarticPolynomial::evaluate(uint8_t mode, double param) const
{
	// 求四次曲线在param处的mode阶导
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
		  return 36 * (coef_[3] * coef_[3] * param + 4 * coef_[3] * coef_[4] * pow(param, 2) + 16 * coef_[4] * coef_[4] * pow(param, 3) / 3);

		default:
		  return 0.0;
  }
}


double QuarticPolynomial::evaluateNormalization(uint8_t mode, double param) const
{
	// 求四次曲线在param处的mode阶导（归一化）
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


void QuarticPolynomial::setCost(double cost)
{
  cost_ = cost;
}

double QuarticPolynomial::getCost() const
{
  return cost_;
}

bool QuarticPolynomial::isValid()
{
	return is_valid_;
}

// 返回曲线轨迹的阶数  20190917
uint8_t QuarticPolynomial::getOrder() const
{
	return order_;
}
// 获取曲线轨迹的多项式系数  20190917
std::vector<double> QuarticPolynomial::getCoef() const
{
	return coef_;
}
		


} // end namespace

