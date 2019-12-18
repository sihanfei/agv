#ifndef QUARTIC_POLYNOMIAL_H
#define QUARTIC_POLYNOMIAL_H

#include "curve.h"

namespace pnc
{

class QuarticPolynomial : public Curve
{
public:
    QuarticPolynomial()=default;
    QuarticPolynomial(const double x0,const double dx0,const double ddx0,
                      const double dx1,const double ddx1,
											const double param0,const double param1);

    virtual ~QuarticPolynomial()=default;

    double evaluate(uint8_t order, double param) const;							// 求四次曲线在param处的mode阶导

    double evaluateNormalization(uint8_t mode, double param) const;	// 求四次曲线在param处的mode阶导（归一化）

    double paramLength() const;
    double paramMax() const;
    double paramMin() const;

    void setCost(double cost);
    double getCost() const;

    double getx1() const;
    double getdx1() const;
    double getddx1() const;
    double getx0() const;
    double getdx0() const;
    double getddx0() const;

		bool isValid();


		uint8_t getOrder() const;					// 获取曲线轨迹的阶数 20190917
		std::vector<double> getCoef() const;		// 获取曲线轨迹的多项式系数  20190917

private:

    void computeCoefficients(double x0,double dx0,double ddx0,double dx1,double ddx1,double t0,double t1);	// 计算四次多项式多项式系数

		void ComputeCoefficients_Equation(double x0,double dx0,double ddx0,double dx1,double ddx1,double t0,double t1);	// 方程法求四次多项式多项式系数

    std::vector<double> coef_;	// 求到的多项式系数


    double sum_coef_;

    double x0_;
    double dx0_;
    double ddx0_;

    double x1_;
    double dx1_;
    double ddx1_;

    double param0_;
    double param1_;

};

}//end namespace
#endif // QUARTIC_POLYNOMIAL_H

