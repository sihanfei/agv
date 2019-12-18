#ifndef TRAJECTORY_CHECKER_H
#define TRAJECTORY_CHECKER_H

//#include "lattice/collision.h"
#include "math/box2d.h"
#include "common/utils.h"
#include "common/get_boundary_by_cartesian_location.h"

#include "common/perception_info.h"
#include "common/reference_line.h"

#include "common/trajectory.h"

namespace planning
{
class TrajectoryChecker
{
public:
    TrajectoryChecker()=default;
    //~TrajectoryChecker()=default;
    ~TrajectoryChecker();

    explicit TrajectoryChecker(PerceptionInfo& envi_info, ReferenceLine& ref_line);

    bool isTrajInvalid(const Trajectory& trajectory);
    bool isTrajCollided(const Trajectory& trajectory);
    bool isTrajOutOfRoad(const Trajectory& trajectory);

private:

    PerceptionInfo envi_info_;
    ReferenceLine ref_line_;
    //Collision* collision_;
};
}

#endif // TRAJECTORY_CHECKER_H
