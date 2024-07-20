//
// Created by nermin on 13.04.22.
//
#ifndef RPMPL_DRGBT_H
#define RPMPL_DRGBT_H

#include "RRTConnect.h"
#include "RBTConnect.h"
#include "RGBTConnect.h"
#include "RGBMTStar.h"
#include "DRGBTConfig.h"
#include "HorizonState.h"
#include "Splines.h"

// #include <glog/log_severity.h>
// #include <glog/logging.h>
// WARNING: You need to be very careful with using LOG(INFO) for console output, due to a possible "stack smashing detected" error.
// If you get this error, just use std::cout for console output.

namespace planning
{
    namespace drbt
    {
        class DRGBT : public planning::rbt::RGBTConnect
        {
        public:
            DRGBT(const std::shared_ptr<base::StateSpace> ss_);
			DRGBT(const std::shared_ptr<base::StateSpace> ss_, 
                  const std::shared_ptr<base::State> q_start_, const std::shared_ptr<base::State> q_goal_);
            ~DRGBT();                         
            
            bool solve() override;
            bool checkTerminatingCondition(base::State::Status status) override;
            void outputPlannerData(const std::string &filename, bool output_states_and_paths = true, bool append_output = false) const override;
            
		protected:
            void generateHorizon();
            void updateHorizon();
            void generateGBur();
            void shortenHorizon(size_t num);
            void addRandomStates(size_t num);
            void addLateralStates();
            bool modifyState(std::shared_ptr<planning::drbt::HorizonState> &q, size_t max_num_attempts);
            void computeReachedState(const std::shared_ptr<planning::drbt::HorizonState> q);
            void computeNextState();
            int getIndexInHorizon(const std::shared_ptr<planning::drbt::HorizonState> q);
            float updateCurrentState(bool measure_time);
            void updateCurrentState();
            bool changeNextState(std::vector<std::shared_ptr<planning::drbt::HorizonState>> &visited_states);
            bool computeSplineNext(Eigen::VectorXf &current_pos, Eigen::VectorXf &current_vel, Eigen::VectorXf &current_acc, float t_iter_remain);
            bool computeSplineSafe(Eigen::VectorXf &current_pos, Eigen::VectorXf &current_vel, Eigen::VectorXf &current_acc, float t_iter_remain);
            bool checkSplineCollision(std::shared_ptr<planning::trajectory::Spline> spline, float t_offset, float t_max, std::shared_ptr<base::State> &q_init);
            bool computeTargetState(float time = DRGBTConfig::MAX_ITER_TIME);
            bool whetherToReplan();
            std::unique_ptr<planning::AbstractPlanner> initStaticPlanner(float max_planning_time);
            virtual void replan(float max_planning_time);
            bool checkMotionValidity(size_t num_checks = DRGBTConfig::MAX_NUM_VALIDITY_CHECKS);

            std::vector<std::shared_ptr<planning::drbt::HorizonState>> horizon;     // List of all horizon states and their information
            std::shared_ptr<base::State> q_current;                                 // Current robot configuration
            std::shared_ptr<base::State> q_previous;                                // Previous robot configuration
            std::shared_ptr<base::State> q_target;                                  // Target robot configuration to which the robot is currently heading to
            std::shared_ptr<planning::drbt::HorizonState> q_next;                   // Next robot configuration
            std::shared_ptr<planning::drbt::HorizonState> q_next_previous;          // Next robot configuration from the previous iteration
            float d_c;                                                              // Distance-to-obstacles for 'q_current' in the current iteration
            float d_max_mean;                                                       // Averaged maximal distance-to-obstacles through iterations
            size_t horizon_size;                                                    // Number of states that is required to be in the horizon
            bool replanning;                                                        // Whether path replanning is required
            base::State::Status status;                                             // The status of proceeding from 'q_current' towards 'q_next'
            std::vector<std::shared_ptr<base::State>> predefined_path;              // The predefined path that is being followed
            size_t num_lateral_states;                                              // Number of lateral states
            float max_edge_length;                                                  // Maximal edge length when acquiring a new predefined path
            std::shared_ptr<planning::trajectory::Spline> spline_current;           // Current spline that 'q_current' is following in the current iteration
            std::shared_ptr<planning::trajectory::Spline> spline_next;              // Next spline that 'q_current' will follow until the end of current iteration
            bool all_robot_vel_same;                                                // Whether all joint velocities are the same
            size_t max_num_iter_spline_next;                                        // Maximal number of iterations when computing spline_next
            float max_obs_vel;                                                      // Maximal velocity considering all obstacles
        
        private:
            float computeDistanceUnderestimation(const std::shared_ptr<base::State> q, 
	            const std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points, float delta_t);
            void recordTrajectory(bool spline_computed);

        };
    }
}

#endif //RPMPL_DRGBT_H