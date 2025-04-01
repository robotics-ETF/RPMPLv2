//
// Created by nermin on 20.07.24.
//

#ifndef RPMPL_SPLINES_H
#define RPMPL_SPLINES_H

#include "StateSpace.h"
#include "Spline4.h"
#include "Spline5.h"
#include "CompositeSpline.h"
#include "RRTConnectConfig.h"

namespace planning::trajectory
{
    class Splines
    {
    public:
        Splines(const std::shared_ptr<base::StateSpace> &ss_, const std::shared_ptr<base::State> &q_current_, float max_iter_time_);

        bool computeRegular(Eigen::VectorXf &current_pos, Eigen::VectorXf &current_vel, Eigen::VectorXf &current_acc, 
                            float t_iter_remain, float t_max, bool non_zero_final_vel);
        bool computeSafe(Eigen::VectorXf &current_pos, Eigen::VectorXf &current_vel, Eigen::VectorXf &current_acc, 
                         float t_iter_remain, float t_max);
        
        inline void setCurrentState(const std::shared_ptr<base::State> q_current_) { q_current = q_current_; }
        inline void setTargetState(const std::shared_ptr<base::State> q_target_) { q_target = q_target_; }
        inline void setMaxRemainingIterTime(float max_remaining_iter_time_) { max_remaining_iter_time = max_remaining_iter_time_; }
        void recordTrajectory(bool spline_computed);

        std::shared_ptr<planning::trajectory::Spline> spline_current;           // Current spline that 'q_current' is following in the current iteration
        std::shared_ptr<planning::trajectory::Spline> spline_next;              // Next spline generated from 'q_current' to 'q_target'

    private:
        bool checkCollision(std::shared_ptr<base::State> q_init, float t_iter);
        float computeDistanceUnderestimation(const std::shared_ptr<base::State> q, 
                                             const std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points, float delta_t);
        
        std::shared_ptr<base::StateSpace> ss;
        std::shared_ptr<base::State> q_current;                                 // Current robot configuration
        std::shared_ptr<base::State> q_target;                                  // Target robot configuration to which the robot is currently heading to, as well as the configuration where the spline is ending
        bool all_robot_vel_same;                                                // Whether all joint velocities are the same
        float max_obs_vel;                                                      // Maximal velocity of dynamic obstacles used to generate dynamic bubbles
        size_t max_num_iter_spline_regular;                                     // Maximal number of iterations when computing regular spline
        float max_iter_time;                                                    // Maximal iteration time
        float max_remaining_iter_time;                                          // Maximal remaining iteration time till the end of the current iteration
    };
}

#endif //RPMPL_SPLINES_H