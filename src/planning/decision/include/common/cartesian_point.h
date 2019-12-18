#ifndef CARTESIANPOINT_H
#define CARTESIANPOINT_H

namespace pnc {

class CartesianPoint
{
public:
    CartesianPoint()=default;
    ~CartesianPoint()=default;
    CartesianPoint(double x,double y,double theta,double kappa,double vel,double acc);

    void setX(double x);
    void setY(double y);
    void setTheta(double theta);
    void setKappa(double kappa);
    void setVel(double vel);
    void setAcc(double acc);


    double getX() const;
    double getY() const;
    double getTheta() const;
    double getKappa() const;
    double getVel() const;
    double getAcc() const;



private:
    double x_;
    double y_;
    double theta_;
    double kappa_;
    double vel_;
    double acc_;


};

}

#endif // CARTESIANPOINT_H
