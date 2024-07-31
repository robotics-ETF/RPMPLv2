//
// Created by nermin on 18.02.22.
//

#ifndef RPMPL_RBTCONNECT_H
#define RPMPL_RBTCONNECT_H

#include "RRTConnect.h"
#include "RBTConnectConfig.h"

// #include <glog/log_severity.h>
// #include <glog/logging.h>
// WARNING: You need to be very careful with using LOG(INFO) for console output, due to a possible "stack smashing detected" error.
// If you get this error, just use std::cout for console output.

namespace planning::rbt
{
	class RBTConnect : public planning::rrt::RRTConnect
	{
	public:
		RBTConnect(const std::shared_ptr<base::StateSpace> ss_);
		RBTConnect(const std::shared_ptr<base::StateSpace> ss_, 
				   const std::shared_ptr<base::State> q_start_, const std::shared_ptr<base::State> q_goal_);
		
		bool solve() override;
		bool checkTerminatingCondition(base::State::Status status) override;
		void outputPlannerData(const std::string &filename, bool output_states_and_paths = true, bool append_output = false) const override;

	protected:
		std::shared_ptr<base::State> getRandomState(const std::shared_ptr<base::State> q_center);
		std::tuple<base::State::Status, std::shared_ptr<base::State>> extendSpine
			(const std::shared_ptr<base::State> q, const std::shared_ptr<base::State> q_e);
		base::State::Status connectSpine(const std::shared_ptr<base::Tree> tree, const std::shared_ptr<base::State> q, 
										 const std::shared_ptr<base::State> q_e);
	};
}

#endif //RPMPL_RBTCONNECT_H