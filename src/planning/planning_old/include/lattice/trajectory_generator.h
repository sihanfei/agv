
#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include "lattice/trajectory_scorer.h"
#include "lattice/trajectory_checker.h"
#include "lattice/lattice_trajectory1d.h"

#include "math/cartesian_frenet_converter.h"
#include "pt_graph/path_time_graph.h"
#include "decision.h"

namespace planning
{

class TrajectoryGenerator
{
public:
    TrajectoryGenerator()=default;
    ~TrajectoryGenerator();

    TrajectoryGenerator(FrenetPoint& current_location, FrenetPoint& current_goal, PerceptionInfo& envi_info, ReferenceLine& ref_line, decision deci);

    //bool GetBestTrajectory(const geometry_msgs::Pose &goal, ReferenceLine &ref_line, EnviInfo &envi_info);
    bool getBestTrajectory(Trajectory& best_trajectory);

private:

    std::vector< std::shared_ptr<Curve1d> > getLatTrajectories();//hengxiang
    std::vector< std::shared_ptr<Curve1d> > getLonTrajectories();//hengxiang
    //std::vector<double> GetRefSpeed(double v0,double vcruise,decision deci,double s0,double s1);

    bool combine(const std::vector<TrajectoryPair>& pair_vec, Trajectory& best_trajectory);
    //bool combinebByRefVel(const std::vector< std::shared_ptr<Curve1d> >& ls_paths, Trajectory& best_trajectory, const std::vector<double>& ref_vel);

    FrenetPoint frelocation_;
    FrenetPoint fregoal_;
    PerceptionInfo envi_info_;
    //bool stop_flag_;
    decision deci_;

    ReferenceLine ref_line_;

    PathTimeGraph* pt_graph_;
    //Traj best_traj_;

    //double ref_point_max_speed_;

    //std::vector< Traj > traies_;
};

//for debug
//extern std::vector< planning::Trajectory > all_traj_for_show;

}//end namespace

#endif //TRAJECTORY_GENERATOR_H
