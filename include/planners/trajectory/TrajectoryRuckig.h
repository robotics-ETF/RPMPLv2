//
// Created by nermin on 03.10.25.
//

#ifndef RPMPL_TRAJECTORYRUCKIG_H
#define RPMPL_TRAJECTORYRUCKIG_H

#include "AbstractTrajectory.h"
#include "RRTConnectConfig.h"
#include <ruckig/ruckig.hpp>

namespace planning::trajectory
{
    class TrajectoryRuckig : public AbstractTrajectory
    {
    public:
        TrajectoryRuckig(const std::shared_ptr<base::StateSpace> &ss_, size_t num_waypoints);
        TrajectoryRuckig(const std::shared_ptr<base::StateSpace> &ss_, planning::trajectory::State current, float max_iter_time_);
        ~TrajectoryRuckig();
        
        bool computeRegular(planning::trajectory::State current, planning::trajectory::State target, 
                            float t_iter_remain, float t_max, bool non_zero_final_vel) override;
        bool computeSafe(planning::trajectory::State current, planning::trajectory::State target, 
                         float t_iter_remain, float t_max, const std::shared_ptr<base::State> q_current) override;
        
        Eigen::VectorXf getPosition(float t) override;
        Eigen::VectorXf getVelocity(float t) override;
        Eigen::VectorXf getAcceleration(float t) override;

        bool convertPathToTraj(const std::vector<std::shared_ptr<base::State>> &path) override;
        
    private:
        void setParams();
        void setCurrentState(const planning::trajectory::State &current);
        void setTargetState(const planning::trajectory::State &target);
        void setTraj(const ruckig::Trajectory<ruckig::DynamicDOFs> &traj_);
        Eigen::VectorXf getPos(const ruckig::Trajectory<ruckig::DynamicDOFs> &traj_, float t);
        Eigen::VectorXf getVel(const ruckig::Trajectory<ruckig::DynamicDOFs> &traj_, float t);
        Eigen::VectorXf getAcc(const ruckig::Trajectory<ruckig::DynamicDOFs> &traj_, float t);

        ruckig::InputParameter<ruckig::DynamicDOFs> input;
        ruckig::OutputParameter<ruckig::DynamicDOFs> output;
        ruckig::Trajectory<ruckig::DynamicDOFs> traj;
        ruckig::Trajectory<ruckig::DynamicDOFs> traj_emg;
        float time_join;                                        // In case emergency trajectory is not computed, it is set to -1.
        
    };
}

#endif //RPMPL_TRAJECTORYRUCKIG_H