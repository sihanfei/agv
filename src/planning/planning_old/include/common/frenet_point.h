#ifndef FRENET_POINT_H
#define FRENET_POINT_H

namespace planning {
class FrenetPoint
{
public:
    FrenetPoint()=default;
    ~FrenetPoint()=default;
    FrenetPoint(double s,double ds,double dds,double l,double dl,double ddl,double relative_time);

    double getS() const;
    double getdS() const;
    double getddS() const;
    double getL() const;
    double getdL() const;
    double getddL() const;
    double getRelativeTime() const;

    void setS(double s);
    void setdS(double ds);
    void setddS(double dds);
    void setL(double l);
    void setdL(double dl);
    void setddL(double ddl);
    void setRelativeTime(double relative_time);

private:
    double s_;
    double ds_;
    double dds_;
    double l_;
    double dl_;
    double ddl_;
    double relative_time_;

};
}

#endif // FRENET_POINT_H
