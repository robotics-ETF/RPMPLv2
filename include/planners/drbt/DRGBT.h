//
// Created by nermin on 13.04.22.
//
#ifndef RPMPL_DRGBT_H
#define RPMPL_DRGBT_H

#include "RGBTConnect.h"
#include "RGBMTStar.h"
#include "HorizonState.h"
#include "Spline5.h"
#include "Spline4.h"

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
            bool computeSplineNext(Eigen::VectorXf &q_current_dot, Eigen::VectorXf &q_current_ddot);
            bool computeSplineSafe(Eigen::VectorXf &q_current_dot, Eigen::VectorXf &q_current_ddot, float t_iter_remain);
            bool computeTargetState(float time = DRGBTConfig::MAX_ITER_TIME);
            bool changeNextState(std::vector<std::shared_ptr<planning::drbt::HorizonState>> &visited_states);
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
            float d_c;                                                              // Distance-to-obstacles for 'q_target' in the current iteration
            float d_max_mean;                                                       // Averaged maximal distance-to-obstacles through iterations
            size_t horizon_size;                                                    // Number of states that is required to be in the horizon
            bool replanning;                                                        // Whether path replanning is required
            base::State::Status status;                                             // The status of proceeding from 'q_target' towards 'q_next'
            std::vector<std::shared_ptr<base::State>> predefined_path;              // The predefined path that is being followed
            size_t num_lateral_states;                                              // Number of lateral states
            float max_edge_length;                                                  // Maximal edge length when acquiring a new predefined path
            std::shared_ptr<planning::trajectory::Spline> spline_current;           // Current spline that 'q_current' is following in the current iteration
            std::shared_ptr<planning::trajectory::Spline> spline_next;              // Next spline that 'q_current' will follow until the end of current iteration
            bool all_robot_vel_same;                                                // Whether all joint velocities are the same
            size_t max_num_iter_spline_next;                                        // Maximal number of iterations when computing spline_next
            float max_obs_vel;                                                      // Maximal velocity considering all obstacles
        };
    }
}

#endif //RPMPL_DRGBT_H