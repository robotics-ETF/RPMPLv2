//
// Created by nermin on 19.03.25.
//

#ifndef RPMPL_UPDATINGSTATE_H
#define RPMPL_UPDATINGSTATE_H

#include "StateSpace.h"
#include "RealVectorSpaceConfig.h"
#include "PlanningTypes.h"
#include "AbstractTrajectory.h"

namespace planning::drbt
{
    class DRGBT;
}

namespace planning::trajectory
{
    class UpdatingState
    {
    public:
        UpdatingState(const std::shared_ptr<base::StateSpace> &ss_, planning::TrajectoryInterpolation traj_interpolation_, float max_iter_time_);
        ~UpdatingState() {}

        bool update(std::shared_ptr<base::State> &q_previous, std::shared_ptr<base::State> &q_current, 
                    const std::shared_ptr<base::State> q_next_, base::State::Status &status);
        bool update(std::shared_ptr<base::State> &q_previous, std::shared_ptr<base::State> &q_current, 
                    const std::shared_ptr<base::State> q_next_, const std::shared_ptr<base::State> q_next_reached_, base::State::Status &status);
        inline void setTrajectory(const std::shared_ptr<planning::trajectory::AbstractTrajectory> &traj_) { traj = traj_; }
        inline void setGuaranteedSafeMotion(bool guaranteed_safe_motion_) { guaranteed_safe_motion = guaranteed_safe_motion_; }
        inline void setNonZeroFinalVel(bool non_zero_final_vel_) { non_zero_final_vel = non_zero_final_vel_; }
        inline void setMaxRemainingIterTime(float max_remaining_iter_time_) { max_remaining_iter_time = max_remaining_iter_time_; }
        inline void setTimeIterStart(std::chrono::steady_clock::time_point &time_iter_start_) { time_iter_start = time_iter_start_; }
        inline void setNextState(const std::shared_ptr<base::State> &q_next_) { q_next = q_next_; }
        inline void setNextStateReached(const std::shared_ptr<base::State> &q_next_reached_) { q_next_reached = q_next_reached_; }
        inline void setDRGBTinstance(planning::drbt::DRGBT* drgbt_instance_) { drgbt_instance = drgbt_instance_; };
        inline float getWaitingTime() { return waiting_time; }

    private:
        bool update_v1(std::shared_ptr<base::State> &q_previous, std::shared_ptr<base::State> &q_current, 
                       const std::shared_ptr<base::State> q_next_, const std::shared_ptr<base::State> q_next_reached_, base::State::Status &status);
        bool update_v2(std::shared_ptr<base::State> &q_previous, std::shared_ptr<base::State> &q_current, 
                       const std::shared_ptr<base::State> q_next_, const std::shared_ptr<base::State> q_next_reached_, base::State::Status &status);
        bool invokeChangeNextState();
        float getElapsedTime();

        std::shared_ptr<base::StateSpace> ss;
        planning::TrajectoryInterpolation traj_interpolation;
        float max_iter_time;                                                    // Maximal iteration time in [s]
        
        // Additional info (not mandatory to be set):
        std::shared_ptr<planning::trajectory::AbstractTrajectory> traj;         // Trajectory which is generated from 'q_current' towards 'q_next'
        bool all_robot_vel_same;                                                // Whether all joint velocities are the same
        bool guaranteed_safe_motion;                                            // Whether robot motion is surely safe for environment
        bool non_zero_final_vel;                                                // Whether final spline velocity can be non-zero (available only when 'guaranteed_safe_motion' is false)
        float max_remaining_iter_time;                                          // Maximal remaining iteration time in [s] till the end of the current iteration
        std::chrono::steady_clock::time_point time_iter_start;                  // Time point when the current iteration started
        float waiting_time;                                                     // Return value of 'update_v2' function. Waiting time in [s] after which the new trajectory
                                                                                // will become active (if 'planning::TrajectoryInterpolation::None' is not used).
        std::shared_ptr<base::State> q_next;
        std::shared_ptr<base::State> q_next_reached;
        planning::drbt::DRGBT* drgbt_instance;
    };
}

#endif //RPMPL_UPDATINGSTATE_H