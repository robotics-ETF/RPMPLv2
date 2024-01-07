//
// Created by nermin on 13.04.22.
//
#ifndef RPMPL_DRGBT_H
#define RPMPL_DRGBT_H

#include "RGBTConnect.h"
#include "RGBMTStar.h"
#include "HorizonState.h"

namespace planning
{
    namespace drbt
    {
        class DRGBT : public planning::rbt::RGBTConnect
        {
        public:
            DRGBT(std::shared_ptr<base::StateSpace> ss_);
			DRGBT(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_, std::shared_ptr<base::State> goal_);
            ~DRGBT();                         
            
            bool solve() override;
            bool checkTerminatingCondition();
			void outputPlannerData(std::string filename, bool output_states_and_paths = true, bool append_output = false) const override;
            
		protected:
            void generateHorizon();
            void updateHorizon(float d_c);
            void generateGBur();
            void shortenHorizon(int num);
            void addRandomStates(int num);
            void addLateralStates();
            bool modifyState(std::shared_ptr<HorizonState> &q, int max_num_attempts);
            void computeReachedState(std::shared_ptr<base::State> q_current, std::shared_ptr<HorizonState> q);
            void computeNextState();
            inline void setNextState(std::shared_ptr<HorizonState> q);
            int getIndexInHorizon(std::shared_ptr<HorizonState> q);
            void updateCurrentState();
            bool whetherToReplan();
            std::unique_ptr<planning::AbstractPlanner> initStaticPlanner(int max_planning_time);
            virtual void replan(int max_planning_time);
            void acquirePredefinedPath(const std::vector<std::shared_ptr<base::State>> &path_, float delta_q_max);
            bool checkMotionValidity(int num_checks = DRGBTConfig::MAX_NUM_VALIDITY_CHECKS);

            std::vector<std::shared_ptr<HorizonState>> horizon;             // List of all horizon states and their information
            std::shared_ptr<base::State> q_current;                         // Current robot configuration
            std::shared_ptr<base::State> q_prev;                            // Previous robot configuration
            std::shared_ptr<HorizonState> q_next;                           // Next robot configuration
            std::shared_ptr<HorizonState> q_next_previous;                  // Next robot configuration from the previous iteration
            float d_max_mean;                                               // Averaged maximal distance-to-obstacles through iterations
            int horizon_size;                                               // Number of states that is required to be in the horizon
            bool replanning;                                                // Whether path replanning is required
            base::State::Status status;                                     // The status of proceeding from 'q_curr' towards 'q_next'
            std::vector<std::shared_ptr<base::State>> predefined_path;      // The predefined path that is being followed
            const float hysteresis = 0.1;                                   // Hysteresis size when choosing the next state
            const int num_lateral_states = 2 * ss->getNumDimensions() - 2;  // Number of lateral states
            std::chrono::steady_clock::time_point time_iter_start;
            int replanning_cnt;
        };
    }
}

#endif RPMPL_DRGBT_H