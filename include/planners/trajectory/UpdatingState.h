//
// Created by nermin on 19.03.25.
//

#ifndef RPMPL_UPDATINGSTATE_H
#define RPMPL_UPDATINGSTATE_H

#include "StateSpace.h"
#include "Splines.h"
#include "RealVectorSpaceConfig.h"
#include "PlanningTypes.h"

namespace planning::drbt
{
    class DRGBT;
}

namespace planning::trajectory
{
    class UpdatingState
    {
    public:
        UpdatingState(const std::shared_ptr<base::StateSpace> &ss_, const std::shared_ptr<base::State> &q_previous_,
            const std::shared_ptr<base::State> &q_current_, const std::shared_ptr<base::State> &q_next_, 
            base::State::Status &status_, float max_iter_time_, const std::chrono::steady_clock::time_point &time_iter_start_);
        ~UpdatingState() {}

        void update();
        inline void setTrajectoryInterpolation(planning::TrajectoryInterpolation traj_interpolation_) { traj_interpolation = traj_interpolation_; }
        inline void setSplines(const std::shared_ptr<planning::trajectory::Splines> &splines_) { splines = splines_; }
        inline void setNextStateReached(const std::shared_ptr<base::State> &q_next_reached_) { q_next_reached = q_next_reached_; }
        inline void setMeasureTime(bool measure_time_) { measure_time = measure_time_; }
        inline void setMaxRemainingIterTime(float max_remaining_iter_time_) { max_remaining_iter_time = max_remaining_iter_time_; }
        inline void setGuaranteedSafeMotion(bool guaranteed_safe_motion_) { guaranteed_safe_motion = guaranteed_safe_motion_; }
        inline void setNonZeroFinalVel(bool non_zero_final_vel_) { non_zero_final_vel = non_zero_final_vel_; }
        inline void setDRGBTinstance(planning::drbt::DRGBT* drgbt_instance_) { drgbt_instance = drgbt_instance_; };
        inline float getRemainingTime() { return remaining_time; }

    private:
        void update_v1();
        void update_v2();
        float getElapsedTime();

        std::shared_ptr<base::StateSpace> ss;
        std::shared_ptr<base::State> q_previous;
        std::shared_ptr<base::State> q_current;
        std::shared_ptr<base::State> q_next;
        base::State::Status status;
        float max_iter_time;                                        // Maximal iteration time in [s]
        std::chrono::steady_clock::time_point time_iter_start;      // Time point when the current iteration started
        
        // Additional info (not mandatory to be set):
        std::shared_ptr<base::State> q_next_reached;
        planning::TrajectoryInterpolation traj_interpolation;
        std::shared_ptr<planning::trajectory::Splines> splines;
        bool all_robot_vel_same;                                    // Whether all joint velocities are the same
        bool guaranteed_safe_motion;                                // Whether robot motion is surely safe for environment
        bool non_zero_final_vel;                                    // Whether final spline velocity can be non-zero (available only when 'guaranteed_safe_motion' is false)
        float max_remaining_iter_time;                              // Maximal remaining iteration time in [s] till the end of the current iteration
        bool measure_time;                                          // If true, elapsed time when computing a spline will be exactly measured. 
                                                                    // If false, elapsed time will be computed (default: false).
                                                                    // It should always be false when simulation pacing is used, since then a time measuring will not be correct! 
        
        float remaining_time;                                       // Return value of 'update_v2' function. Remaining time in [s] after which the new spline 'splines->spline_next' 
                                                                    // will become active (if 'planning::TrajectoryInterpolation::Spline' is used).

        planning::drbt::DRGBT* drgbt_instance;
        bool invokeChangeNextState();
    };
}

#endif //RPMPL_UPDATINGSTATE_H