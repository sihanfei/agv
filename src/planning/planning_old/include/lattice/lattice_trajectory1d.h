#ifndef LATTICE_TRAJECTORY1D_H
#define LATTICE_TRAJECTORY1D_H

#include "math/curve1d.h"

namespace planning {

class LatticeTrajectory1d:public Curve1d
{
public:
    explicit LatticeTrajectory1d(std::shared_ptr<Curve1d> ptr_trajectory1d);
    virtual ~LatticeTrajectory1d()=default;

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

    void setTrajInfo(std::string info);//for debug
    void displayTrajInfo();//for debug

    // void setRestriction(Restriction& restriction);

    // Restriction getRestriction() const;

private:
    std::shared_ptr<Curve1d> ptr_trajectory1d_;

};

}


#endif // LATTICE_TRAJECTORY1D_H
