#ifndef CURVE_H
#define CURVE_H

#include "common/utils.h"

namespace planning
{

class Curve1d
{
public:
    Curve1d()=default;
    virtual ~Curve1d()=default;

    virtual double evaluate(uint8_t mode,double param) const = 0;
    virtual double evaluateNormalization(uint8_t mode, double param) const = 0;

    virtual double paramLength() const = 0;
    virtual double paramMax() const = 0;

    virtual double paramMin() const = 0;
    //virtual double getx1() const = 0;
    //virtual double getxLength() const = 0;
    //virtual double getdxLength() const = 0;
    //virtual int getCoefSize() const = 0;

    virtual double getx1() const = 0;
    virtual double getdx1() const = 0;
    virtual double getddx1() const = 0;

    virtual double getx0() const = 0;
    virtual double getdx0() const = 0;
    virtual double getddx0() const = 0;

    virtual void setCost(double cost) = 0;
    virtual double getCost() const = 0;

    virtual void setTrajInfo(std::string info) = 0;//for debug
    virtual void displayTrajInfo() = 0;//for debug

    // virtual void setRestriction(Restriction& restriction) = 0;
    // virtual Restriction getRestriction() const = 0;

protected:

    double cost_;

    std::string info_;
    //Restriction restriction_;
    
    //double x1_;
    //double dx1_;

};
}//end namespace
#endif // CURVE_H
