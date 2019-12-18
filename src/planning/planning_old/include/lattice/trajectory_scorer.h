
#ifndef TRAJECTORY_SCORE_H
#define TRAJECTORY_SCORE_H

#include "math/quartic_polynomial.h"
#include "math/quintic_polynomial.h"

#include "common/trajectory_pair.h"
#include "pt_graph/path_time_graph.h"

#include "common/frenet_point.h"

//#include <memory>

namespace planning{

bool pairSortCost(TrajectoryPair& p1, TrajectoryPair& p2);
bool pathSortCost(std::shared_ptr<Curve1d>& c1, std::shared_ptr<Curve1d>& c2);

class TrajectoryScorer
{
public:
    TrajectoryScorer()=default;
    ~TrajectoryScorer()=default;

    TrajectoryScorer(std::vector< std::shared_ptr<Curve1d> >& all_st_paths, std::vector< std::shared_ptr<Curve1d> >& all_ls_paths, FrenetPoint& goal, bool stop_flag);
    std::vector<TrajectoryPair>& scorePairs();
    //std::vector<std::shared_ptr<Curve1d>>& scoreLSPaths();

private:
    double calJerkIntST(const TrajectoryPair& traj_pair);
    double calJerkIntLS(const TrajectoryPair& traj_pair);
    double calJerkIntLS(const std::shared_ptr<Curve1d>& ls_path);
    double calJerkInt(const TrajectoryPair& traj_pair);

    double calTrajectoryCostLat(const TrajectoryPair& traj_pair);
    double calTrajectoryCostLat(const std::shared_ptr<Curve1d>& ls_path);
    
    double calTrajectoryCostLonStopingMerging(const TrajectoryPair& traj_pair);
    double calTrajectoryCostLonCruise(const TrajectoryPair& traj_pair);
    double calTrajectoryCostPair(const TrajectoryPair& traj_pair);

    bool IsLonValid(const std::shared_ptr<Curve1d> st_path);
    bool IsLatValid(const std::shared_ptr<Curve1d> ls_path);

    //std::vector< std::vector< std::pair<double, double> > > path_time_intervals_;
    std::vector< std::shared_ptr<Curve1d> > ls_path_;
    std::vector<TrajectoryPair> traj_pair_vec_;
    FrenetPoint goal_;
    bool stop_flag_;
    
};


} // namespace planning

#endif
