//
// Created by nermin on 20.03.25.
//

#ifndef RPMPL_MOTIONVALIDITY_H
#define RPMPL_MOTIONVALIDITY_H

#include "StateSpace.h"
#include "Splines.h"
#include "PlanningTypes.h"

namespace planning::trajectory
{
    class MotionValidity
    {
    public:
        MotionValidity(const std::shared_ptr<base::StateSpace> &ss_, planning::TrajectoryInterpolation traj_interpolation_, 
            float resolution_coll_check_, const std::shared_ptr<planning::trajectory::Splines> &splines_, const std::shared_ptr<base::State> &q_previous_,
            const std::shared_ptr<base::State> &q_current_, const std::shared_ptr<base::State> &q_goal_, 
            std::vector<std::shared_ptr<base::State>> &path_, float max_iter_time_);

        bool check();

    private:
        bool check_v1();
        bool check_v2();

        std::shared_ptr<base::StateSpace> ss;
        planning::TrajectoryInterpolation traj_interpolation;
        float resolution_coll_check;
        std::shared_ptr<planning::trajectory::Splines> splines;
        std::shared_ptr<base::State> q_previous;
        std::shared_ptr<base::State> q_current;
        std::shared_ptr<base::State> q_goal;
        std::vector<std::shared_ptr<base::State>> path;
        float max_iter_time;
        size_t num_checks;          // Maximal number of validity checks when robot moves from previous to current configuration, 
                                    // while the obstacles are moving simultaneously.
    };
}

#endif //RPMPL_MOTIONVALIDITY_H