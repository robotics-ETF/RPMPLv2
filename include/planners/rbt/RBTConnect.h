//
// Created by nermin on 18.02.22.
//
#ifndef RPMPL_RBTCONNECT_H
#define RPMPL_RBTCONNECT_H

#include "RRTConnect.h"

namespace planning
{
	namespace rbt
	{
		class RBTConnect : public planning::rrt::RRTConnect
		{
		public:
			RBTConnect(std::shared_ptr<base::StateSpace> ss_);
			RBTConnect(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_, std::shared_ptr<base::State> goal_);
			
			bool solve() override;
			bool checkTerminatingCondition(base::State::Status status) override;
			void outputPlannerData(std::string filename, bool output_states_and_paths = true, bool append_output = false) const override;

		protected:
            std::shared_ptr<base::State> getRandomState(std::shared_ptr<base::State> q_center);
			std::tuple<base::State::Status, std::shared_ptr<base::State>> 
				extendSpine(std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e);
			base::State::Status connectSpine(std::shared_ptr<base::Tree> tree, 
											 std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e);
		};
	}
}
#endif //RPMPL_RBTCONNECT_H
