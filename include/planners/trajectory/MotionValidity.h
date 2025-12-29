//
// Created by nermin on 20.03.25.
//

#ifndef RPMPL_MOTIONVALIDITY_H
#define RPMPL_MOTIONVALIDITY_H

#include "StateSpace.h"
#include "PlanningTypes.h"
#include "TrajectoryConfig.h"

namespace planning::trajectory
{
    class MotionValidity
    {
    public:
        MotionValidity(const std::shared_ptr<base::StateSpace> &ss_, float resolution_coll_check_, float max_iter_time_,
                       std::vector<std::shared_ptr<base::State>>* path_);
        ~MotionValidity() {}

        bool check(const std::shared_ptr<base::State> &q_previous, const std::shared_ptr<base::State> &q_current);
        bool check(const std::vector<Eigen::VectorXf> &traj_points_current_iter);
        
    private:
        std::shared_ptr<base::StateSpace> ss;
        float resolution_coll_check;
        float max_iter_time;
        std::vector<std::shared_ptr<base::State>>* path;

    };
}

#endif //RPMPL_MOTIONVALIDITY_H