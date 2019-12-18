#ifndef LATTICE_TRAJECTORY1D_H
#define LATTICE_TRAJECTORY1D_H

#include "curve/curve.h"

namespace pnc 
{

class TrajectoryCurve:public Curve
{

public:

    explicit TrajectoryCurve(std::shared_ptr<Curve> ptr_trajectory_curve);

    virtual ~TrajectoryCurve()=default;

    double evaluate(uint8_t mode, double param) const;

    double evaluateNormalization(uint8_t mode, double param) const;

    double paramLength() const;
    double paramMax() const;

    double paramMin() const;

    double getx1() const;
    double getdx1() const;
    double getddx1() const;
    double getx0() const;
    double getdx0() const;
    double getddx0() const;

    void setCost(double cost);
    double getCost() const;


    bool isValid();


		uint8_t getOrder() const;					// 获取曲线轨迹的阶数 20190917
		std::vector<double> getCoef() const;		// 获取曲线轨迹的多项式系数  20190917

private:

    std::shared_ptr<Curve> ptr_trajectory_curve_;

};

}


#endif // LATTICE_TRAJECTORY1D_H

