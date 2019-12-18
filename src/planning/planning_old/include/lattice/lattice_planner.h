
#ifndef LATTICE_PLANNER_H
#define LATTICE_PLANNER_H

#include "trajectory_generator.h"

namespace planning {

class LatticePlanner
{
public:
    //LatticePlanner()=default;
    LatticePlanner();
    ~LatticePlanner()=default;

    void setGoal(const FrenetPoint fregoal);
    bool isGoalReached(const FrenetPoint& frelocation);

    bool planning(FrenetPoint& fre_start_plan_point,FrenetPoint& fre_stop_plan_point,ReferenceLine& ref_line,PerceptionInfo perception_info,decision deci,Trajectory& best_trajectory);
    //bool fastStopPlanning(ReferenceLine& ref_line,Trajectory& best_trajectory);
    //bool slowStopPlanning(ReferenceLine& ref_line,Trajectory& best_trajectory);

    bool isStartReplan(const ControlInfo control_info, const Trajectory& last_best_trajectory);

private:

    FrenetPoint goal_;
    FrenetPoint current_goal_;
    FrenetPoint current_location_;
    PerceptionInfo envi_info_;

    bool is_end_point_;
    //  EnviInfo envi_info_;
    uint32_t offset_count_;
    uint32_t replan_count_;
};

}//end namespace

#endif // LATTICE_PLANNER_H
