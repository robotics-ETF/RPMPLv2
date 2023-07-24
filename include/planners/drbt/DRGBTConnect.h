//
// Created by nermin on 13.04.22.
//
#ifndef RPMPL_DRGBTCONNECT_H
#define RPMPL_DRGBTCONNECT_H

#include "RGBTConnect.h"

namespace planning
{
    namespace drbt
    {
        class DRGBTConnect : public planning::rbt::RGBTConnect
        {
        public:
            DRGBTConnect(std::shared_ptr<base::StateSpace> ss_);
			DRGBTConnect(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_, std::shared_ptr<base::State> goal_);
            bool solve() override;
            bool checkTerminatingCondition();
			void outputPlannerData(std::string filename, bool output_states_and_paths = true, bool append_output = false) const override;
            
            class HorizonState
            {
            public:
                enum Status {Good, Bad, Critical, Goal};

            private:
                std::shared_ptr<base::State> state;
                std::shared_ptr<base::State> state_reached;     // Reached state when generating spine from 'q_current' towards 'state'
                HorizonState::Status status;                    // Status of 'state_reached': 'Good', 'Bad', 'Critical' or 'Goal'
                int index;                                      // Index of 'state' in the predefined path. It is -1 if 'state' does not belong to the path
                float d_c;                                      // Underestimation of distance-to-obstacles for 'state_reached'
                float d_c_previous;                             // 'd_c' from previous iteration
                float weight;                                   // Weight in range [0, 1] for 'state_reached'
                bool is_reached;                                // Whether 'state_reached' == 'state'
            
            public:
                HorizonState(std::shared_ptr<base::State> state_, int index_);
                HorizonState() {}
                ~HorizonState() {}

                inline std::shared_ptr<base::State> getState() const { return state; }
                inline std::shared_ptr<base::State> getStateReached() const { return state_reached; }
                inline HorizonState::Status getStatus() const { return status; }
                inline int getIndex() const { return index; }
                inline float getDistance() const { return d_c; }
                inline float getDistancePrevious() const { return d_c_previous; }
                inline float getWeight() const { return weight; }
		        inline const Eigen::VectorXf &getCoord() const { return state->getCoord(); }
		        inline float getCoord(int idx) const { return state->getCoord(idx); }
                inline bool getIsReached() const { return is_reached; }

                inline void setStateReached(std::shared_ptr<base::State> state_reached_) { state_reached = state_reached_; }
                inline void setStatus(HorizonState::Status status_) { status = status_; }
                inline void setIndex(int index_) { index = index_; }
                inline void setDistance(float d_c_);
                inline void setDistancePrevious(float d_c_previous_) { d_c_previous = d_c_previous_; }
                inline void setWeight(float weight_);
                inline void setIsReached(bool is_reached_) { is_reached = is_reached_; }

		        friend std::ostream &operator<<(std::ostream &os, const HorizonState *q);
            };

		protected:
            std::vector<std::shared_ptr<HorizonState>> horizon;                 // List of all horizon states and their information
            std::shared_ptr<base::State> q_current;                             // Current robot configuration
            std::shared_ptr<HorizonState> q_next;                               // Next robot configuration
            std::shared_ptr<HorizonState> q_next_previous;                      // Next robot configuration from the previous iteration
            float d_max_mean;                                                   // Averaged maximal distance-to-obstacles through iterations
            int horizon_size;                                                   // Number of states that is required to be in the horizon
            bool replanning;                                                    // Whether path replanning is required
            base::State::Status status;                                         // The status of proceeding from 'q_curr' towards 'q_next'
            std::vector<std::shared_ptr<base::State>> predefined_path;          // The predefined path that is being followed
            const float hysteresis = 0.1;                                       // Hysteresis size when choosing the next state
            const int num_lateral_states = 2 * getSS()->getDimensions() - 2;    // Number of lateral states
            
            void generateHorizon();
            void computeHorizon();
            void shortenHorizon(int num);
            void addRandomStates(int num);
            void addLateralStates();
            bool modifyState(std::shared_ptr<HorizonState> &q);
            void updateCurrentState();
            void computeReachedState(std::shared_ptr<base::State> q_current, std::shared_ptr<HorizonState> q);
            void computeNextState();
            bool whetherToReplan();
            int getIndexInHorizon(std::shared_ptr<HorizonState> q);
        };
    }
}

#endif RPMPL_DRGBTCONNECT_H