#ifndef TRAJECTORY_PAIR_H
#define TRAJECTORY_PAIR_H

#include "curve/curve.h"

namespace pnc
{


class TrajectoryPair
{

public:

    TrajectoryPair() = default;
    ~TrajectoryPair()= default;
    TrajectoryPair(Curve* ls_path,Curve* st_path);

    void setLSPath(Curve* ls_path);
    void setSTPath(Curve* st_path);
    void setCost(double cost);


    Curve* getLSPath() const;
    Curve* getSTPath() const;
    double getCost() const;

		// 新增设计轨迹评价参数函数 20190920
		void setCost_lat_jerk(double cost);
		void setCost_lat_s(double cost);
		void setCost_lat_l(double cost);
		void setCost_lon_jerk(double cost);
		void setCost_lon_t(double cost);
		void setCost_lon_s(double cost);
		void setCost_lon_v(double cost);

		double getCost_lat_jerk() const;
		double getCost_lat_s() const;
		double getCost_lat_l() const;
		double getCost_lon_jerk() const;
		double getCost_lon_t() const;
		double getCost_lon_s() const;
		double getCost_lon_v() const;


private:

    Curve* ls_path_;
    Curve* st_path_;

    double cost_;

		// 新增每个轨迹对评价值 20190920
		double cost_lat_jerk;
		double cost_lat_s;
		double cost_lat_l;
		double cost_lon_jerk;
		double cost_lon_t;
		double cost_lon_s;
		double cost_lon_v;



};

}

#endif // TRAJECTORY_PAIR_H

