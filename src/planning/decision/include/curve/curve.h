#ifndef CURVE_H
#define CURVE_H

#include <float.h>
#include <iostream>
#include <queue>
#include <string>
#include <vector>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

namespace pnc
{

// 平面曲线类
class Curve
{
public:
    Curve()=default;
    virtual ~Curve()=default;

    virtual double evaluate(uint8_t mode,double param) const = 0;									// 求曲线在param处的mode阶导

    virtual double evaluateNormalization(uint8_t mode, double param) const = 0;		// 求曲线在param处的mode阶导（归一化）

    virtual double paramLength() const = 0;
    virtual double paramMax() const = 0;

    virtual double paramMin() const = 0;


    virtual double getx1() const = 0;
    virtual double getdx1() const = 0;
    virtual double getddx1() const = 0;

    virtual double getx0() const = 0;
    virtual double getdx0() const = 0;
    virtual double getddx0() const = 0;

    virtual void setCost(double cost) = 0;
    virtual double getCost() const = 0;


	 	virtual bool isValid() = 0;

		virtual uint8_t getOrder() const = 0; 							// 获取曲线轨迹的阶数 20190917
		virtual std::vector<double> getCoef() const = 0;		// 获取曲线轨迹的多项式系数  20190917



protected:

    double cost_;					// 轨迹的cost值
	 	bool is_valid_;				// 轨迹是否有效
		uint8_t order_;				// 曲线轨迹的阶数 20190917

};
}//end namespace
#endif // CURVE_H

