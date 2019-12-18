#include "utils.h"

namespace pnc
{

QuinticPolynomial::QuinticPolynomial(const double x0, const double dx0, const double ddx0,
								 										 const double x1,const double dx1, const double ddx1, 
																		 const double param0, const double param1)
    : x0_(x0), dx0_(dx0), ddx0_(ddx0), x1_(x1), dx1_(dx1), ddx1_(ddx1), param0_(param0), param1_(param1)
{
	// 计算五次多项式多项式系数


  computeCoefficients(x0_, dx0_, ddx0_, x1_, dx1_, ddx1_, param0_, param1_);

/*
	// 方程法计算五次多项式系数
  ComputeCoefficients_Equation(x0_, dx0_, ddx0_, x1_, dx1_, ddx1_, param0_, param1_);

	ROS_INFO("QuinticPolynomial:Coefficients Difference da0 = %f, da1 = %f, da2 = %f, da3 = %f, da4 = %f, da5 = %f",coef_[0] - coef_test[0],coef_[1] - coef_test[1],coef_[2] - coef_test[2],coef_[3] - coef_test[3],coef_[4] - coef_test[4],coef_[5] - coef_test[5]);
*/

  sum_coef_ = fabs(coef_[0]) + fabs(coef_[1]) + fabs(coef_[2]) + fabs(coef_[3]) + fabs(coef_[4]) + fabs(coef_[5]);	// 多项式系数之和，便于归一化

  is_valid_ = true;	  

	// 检查多项式系数，若有系数为无穷大或者NAN，则该多项式曲线无效
  for (double c : coef_)
  {
		if(std::isnan(c)||std::isinf(c))
	 	{
			ROS_INFO("QuinticPolynomial:Trajectory Coefficients is nan or inf .");
			is_valid_ = false;
			break;
		}
  }

		order_ = 5;
	
}


/*

void QuinticPolynomial::ComputeCoefficients_Equation(double x0, double dx0, double ddx0, double x1, double dx1, double ddx1, double s0, double s1)
{
	// 利用方程法求解五次多项式系数
	double delta_p = x1 -x0;
	double delta_dp = dx1 -dx0;
	double delta_ddp = ddx1 -ddx0;

	double T1 = s1 - s0;
	double T2 = pow(s1, 2) - pow(s0, 2);
	double T3 = pow(s1, 3) - pow(s0, 3);
	double T4 = pow(s1, 4) - pow(s0, 4);
	double T5 = pow(s1, 5) - pow(s0, 5);

	double N1 = (delta_p - T1*dx0) - delta_dp*(T2 - 2*T1*s0)/(2*T1);
	double N2 = (delta_p - T1*dx1) - delta_dp*(T2 - 2*T1*s1)/(2*T1);

	double M11 = (T3 - 3*T1*pow(s0, 2)) - 3*T2/(2*T1)*(T2 - 2*T1*s0);
	double M12 = (T4 - 4*T1*pow(s0, 3)) - 4*T3/(2*T1)*(T2 - 2*T1*s0);
	double M13 = (T5 - 5*T1*pow(s0, 4)) - 5*T4/(2*T1)*(T2 - 2*T1*s0);

	double M21 = (T3 - 3*T1*pow(s1, 2)) - 3*T2/(2*T1)*(T2 - 2*T1*s1);
	double M22 = (T4 - 4*T1*pow(s1, 3)) - 4*T3/(2*T1)*(T2 - 2*T1*s1);
	double M23 = (T5 - 5*T1*pow(s1, 4)) - 5*T4/(2*T1)*(T2 - 2*T1*s1);

	double J1 = N1 - delta_ddp/(6*T1)*M11;
	double J2 = N2 - delta_ddp/(6*T1)*M21;

	double K11 = M12 - 2*T2/T1*M11;
	double K12 = M13 - 10*T3/(3*T1)*M11;
	double K21 = M22 - 2*T2/T1*M21;
	double K22 = M23 - 10*T3/(3*T1)*M21;


	double a5 = (J1/K11 - J2/K21)/(K12/K11 - K22/K21);
	double a4 = (J1 - K12*a5)/K11;
	double a3 = (delta_ddp - 12*T2*a4 - 20*T3*a5)/(6*T1);
	double a2 = (delta_dp - 3*T2*a3 - 4*T3*a4 - 5*T4*a5)/(2*T1);
	double a1 = dx0 - 2*s0*a2 - 3*pow(s0, 2)*a3 - 4*pow(s0, 3)*a4 - 5*pow(s0, 4)*a5;
	double a0 = x0 - s0*a1 - pow(s0, 2)*a2 - pow(s0, 3)*a3 - pow(s0, 4)*a4 - pow(s0, 5)*a5;
	coef_.push_back(a0);
	coef_.push_back(a1);
	coef_.push_back(a2);
	coef_.push_back(a3);
	coef_.push_back(a4);
	coef_.push_back(a5);

}

*/

void QuinticPolynomial::computeCoefficients(double x0, double dx0, double ddx0, double x1, double dx1, double ddx1, double s0, double s1)
{

/*
	// 矩阵法计算五次多项式多项式系数

  Eigen::Matrix< double, 6, 6 > A;

  A << 1, s0, pow(s0, 2), pow(s0, 3), pow(s0, 4), pow(s0, 5), 0, 1, 2 * s0, 3 * pow(s0, 2), 4 * pow(s0, 3),
      5 * pow(s0, 4), 0, 0, 2, 6 * s0, 12 * pow(s0, 2), 20 * pow(s0, 3), 1, s1, pow(s1, 2), pow(s1, 3), pow(s1, 4),
      pow(s1, 5), 0, 1, 2 * s1, 3 * pow(s1, 2), 4 * pow(s1, 3), 5 * pow(s1, 4), 0, 0, 2, 6 * s1, 12 * pow(s1, 2),
      20 * pow(s1, 3);

  Eigen::Matrix< double, 6, 1 > b;
  b << x0, dx0, ddx0, x1, dx1, ddx1;

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
	svd.singularValues();
	svd.matrixU();
	svd.matrixV();

	Eigen::Matrix< double, 6, 1 > x = svd.solve(b);

  //Eigen::Matrix< double, 6, 1 > x = A.lu().solve(b);
	//Eigen::Matrix< double, 6, 1 > x = A.svd().solve(b);

  std::vector< double > vec(x.data(), x.data() + x.rows() * x.cols());

  coef_ = vec;

*/

	double p = s1 - s0; 
	std::vector<double> vec(6); 
	if (p <= 0.0) 
	{ 
		vec[0] = 0.0; 
		vec[1] = 0.0; 
		vec[2] = 0.0; 
		vec[3] = 0.0; 
		vec[4] = 0.0; 
		vec[5] = 0.0; 
		coef_ = vec; 
		return; 
	}
	vec[0] = x0; 
	vec[1] = dx0; 
	vec[2] = ddx0 * 0.5; 
	const double p2 = p * p; 
	const double p3 = p * p2; 
	const double c0 = (x1 - 0.5 * p2 * ddx0 - dx0 * p - x0) / p3; 
	const double c1 = (dx1 - ddx0 * p - dx0) / p2; 
	const double c2 = (ddx1 - ddx0) / p; 
	vec[3] = 0.5 * (20.0 * c0 - 8.0 * c1 + c2); 
	vec[4] = (-15.0 * c0 + 7.0 * c1 - c2) / p; 
	vec[5] = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / p2; 

	coef_ = vec; 

}




double QuinticPolynomial::evaluate(uint8_t mode, double param) const
{
	// 求五次曲线在param处的mode阶导
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
	// 求五次曲线在param处的mode阶导（归一化）
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



void QuinticPolynomial::setCost(double cost)
{
  cost_ = cost;
}

double QuinticPolynomial::getCost() const
{
  return cost_;
}


bool QuinticPolynomial::isValid()
{
	return is_valid_;
}

// 返回曲线轨迹的阶数  20190917
uint8_t QuinticPolynomial::getOrder() const
{
	return order_;
}
// 获取曲线轨迹的多项式系数  20190917
std::vector<double> QuinticPolynomial::getCoef() const
{
	return coef_;
}
		

}

