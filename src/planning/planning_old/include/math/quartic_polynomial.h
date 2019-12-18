#ifndef QUARTIC_POLYNOMIAL_H
#define QUARTIC_POLYNOMIAL_H
//si ci duo xiang shi

#include "curve1d.h"

namespace planning
{

class QuarticPolynomial : public Curve1d
{
public:
    QuarticPolynomial()=default;
    QuarticPolynomial(const double x0,const double dx0,const double ddx0,
                      const double dx1,const double ddx1,const double param0,const double param1);

    virtual ~QuarticPolynomial()=default;
    double evaluate(uint8_t order, double param) const;
    double evaluateNormalization(uint8_t mode, double param) const;
    double paramLength() const;
    double paramMax() const;
    double paramMin() const;

    //int getCoefSize() const;

    void setCost(double cost);
    double getCost() const;
    //double cost_;

    double getx1() const;
    double getdx1() const;
    double getddx1() const;
    double getx0() const;
    double getdx0() const;
    double getddx0() const;

    void setTrajInfo(std::string info);//for debug
    void displayTrajInfo();//for debug
    // void setRestriction(Restriction& restriction);
    // Restriction getRestriction() const;

private:
    void computeCoefficients(double x0,double dx0,double ddx0,double dx1,double ddx1,double t0,double t1);
    std::vector<double> coef_;
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
