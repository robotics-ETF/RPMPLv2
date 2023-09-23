//
// Created by nermin on 13.04.22.
//
#ifndef RPMPL_DRGBTCONNECT_H
#define RPMPL_DRGBTCONNECT_H

#include "RGBTConnect.h"
#include "RGBMTStar.h"
#include "HorizonState.h"

namespace planning
{
    namespace drbt
    {
        class DRGBTConnect : public planning::rbt::RGBTConnect
        {
        public:
            DRGBTConnect(std::shared_ptr<base::StateSpace> ss_);
			DRGBTConnect(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_, 
                         std::shared_ptr<base::State> goal_, const std::string &planner_name_ = "RGBTConnect");
            
            bool solve() override;
            bool checkTerminatingCondition();
			void outputPlannerData(std::string filename, bool output_states_and_paths = true, bool append_output = false) const override;
            
		protected:
            void generateHorizon();
            void computeHorizon();
            void shortenHorizon(int num);
            void addRandomStates(int num);
            void addLateralStates();
            bool modifyState(std::shared_ptr<HorizonState> &q);
            void updateCurrentState();
            void computeReachedState(std::shared_ptr<base::State> q_current, std::shared_ptr<HorizonState> q);
            void computeNextState();
            int getIndexInHorizon(std::shared_ptr<HorizonState> q);
            bool whetherToReplan();
            void replan(float replanning_time);
            std::unique_ptr<planning::AbstractPlanner> initPlanner(float max_planning_time);

            std::vector<std::shared_ptr<HorizonState>> horizon;         // List of all horizon states and their information
            std::shared_ptr<base::State> q_current;                     // Current robot configuration
            std::shared_ptr<HorizonState> q_next;                       // Next robot configuration
            std::shared_ptr<HorizonState> q_next_previous;              // Next robot configuration from the previous iteration
            float d_max_mean;                                           // Averaged maximal distance-to-obstacles through iterations
            int horizon_size;                                           // Number of states that is required to be in the horizon
            bool replanning;                                            // Whether path replanning is required
            base::State::Status status;                                 // The status of proceeding from 'q_curr' towards 'q_next'
            std::vector<std::shared_ptr<base::State>> predefined_path;  // The predefined path that is being followed
            const float hysteresis = 0.1;                               // Hysteresis size when choosing the next state
            const int num_lateral_states = 2 * getStateSpace()->getNumDimensions() - 2;    // Number of lateral states
            std::string planner_name;                                   // Name of static planner (for obtaining the predefined path)
        };
    }
}

#endif RPMPL_DRGBTCONNECT_H