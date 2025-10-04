//
// Created by nermin on 03.10.25.
//

#ifndef RPMPL_TrajectoryRuckig_H
#define RPMPL_TrajectoryRuckig_H

#include "StateSpace.h"
#include "TrajectoryConfig.h"
#include "RealVectorSpaceConfig.h"

#include <ruckig/ruckig.hpp>

namespace planning::trajectory
{
    class TrajectoryRuckig
    {
    public:
        TrajectoryRuckig(const std::shared_ptr<base::StateSpace> &ss_, const Eigen::VectorXf &q_current_, float max_iter_time_);
        ~TrajectoryRuckig() {}
        
        bool computeRegular(const Eigen::VectorXf &current_pos, const Eigen::VectorXf &current_vel, 
                            const Eigen::VectorXf &current_acc, float t_iter_remain, float t_max, bool non_zero_final_vel);
        
        Eigen::VectorXf getPosition(float t);
        Eigen::VectorXf getVelocity(float t);
        Eigen::VectorXf getAcceleration(float t);
        void addTrajPointCurrentIter(const Eigen::VectorXf &pos);
        void clearTrajPointCurrentIter();
        bool isFinalConf(const Eigen::VectorXf &q);

        float getTimeCurrent(bool measure_time = false);
        inline float getTimeBegin() const { return time_begin; }
        inline float getTimeEnd() const { return time_end; }
        inline float getTimeFinal() const { return time_final; }
        inline bool getIsZeroFinalVel() const { return is_zero_final_vel; }
        inline const std::vector<Eigen::VectorXf> &getTrajPointCurrentIter() const { return traj_points_current_iter; }

        void setTimeStart(float time_start_offset_);
        inline void setTimeBegin(float time_begin_) { time_begin = time_begin_; }
        inline void setTimeEnd(float time_end_) { time_end = time_end_; }
        inline void setCurrentState(const Eigen::VectorXf &q_current_) { q_current = q_current_; }
        inline void setTargetState(const Eigen::VectorXf &q_target_) { q_target = q_target_; }
        inline void setTimeCurrent(float time_current_) { time_current = time_current_; }
        
    private:
        ruckig::InputParameter<ruckig::DynamicDOFs, ruckig::EigenVector> input;
        ruckig::Trajectory<ruckig::DynamicDOFs, ruckig::EigenVector> traj;
        ruckig::Trajectory<ruckig::DynamicDOFs, ruckig::EigenVector> traj_temp;
        ruckig::Ruckig<ruckig::DynamicDOFs, ruckig::EigenVector> otg;

        std::shared_ptr<base::StateSpace> ss;
        Eigen::VectorXf q_current;                                  // Current robot configuration
        Eigen::VectorXf q_target;                                   // Target robot configuration to which the robot is currently heading to, as well as the configuration where the trajectory is ending
        bool all_robot_vel_same;                                    // Whether all joint velocities are the same
        size_t max_num_iter_trajectory;                             // Maximal number of iterations when computing trajectory
        float max_iter_time;                                        // Maximal iteration time
        float remaining_iter_time;                                  // Remaining iteration time till the end of the current iteration
        float time_current;                                         // Current time for a trajectory
        float time_final;                                           // Final time for a trajectory
        float time_final_temp;                                      // Final time (temp) for a trajectory
        float time_begin;                                           // Time instance in [s] when a trajectory begins in the current iteration
        float time_end;                                             // Time instance in [s] when a trajectory ends in the current iteration
        std::chrono::steady_clock::time_point time_start;           // Start time point when a trajectory is created
        float time_start_offset;                                    // Time offset in [s] which determines how much earlier 'time_start' is created
        bool is_zero_final_vel;                                     // Whether final velocity is zero. If not, robot will move at constant velocity after 'time_final'.
        std::vector<Eigen::VectorXf> traj_points_current_iter;      // Trajectory points from the current iteration to be validated within 'MotionValidity'

    };
}

#endif //RPMPL_TrajectoryRuckig_H