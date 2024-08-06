//
// Created by nermin on 19.02.22.
//

#ifndef RPMPL_RGBTCONNECT_H
#define RPMPL_RGBTCONNECT_H

#include "RBTConnect.h"
#include "RGBTConnectConfig.h"

// #include <glog/log_severity.h>
// #include <glog/logging.h>
// WARNING: You need to be very careful with using LOG(INFO) for console output, due to a possible "stack smashing detected" error.
// If you get this error, just use std::cout for console output.

namespace planning::rbt
{
	class RGBTConnect : public planning::rbt::RBTConnect
	{
	public:
		RGBTConnect(const std::shared_ptr<base::StateSpace> ss_);
		RGBTConnect(const std::shared_ptr<base::StateSpace> ss_, 
					const std::shared_ptr<base::State> q_start_, const std::shared_ptr<base::State> q_goal_);
		
		bool solve() override;
		bool checkTerminatingCondition(base::State::Status status) override;
		void outputPlannerData(const std::string &filename, bool output_states_and_paths = true, bool append_output = false) const override;

	protected:
		std::tuple<base::State::Status, std::shared_ptr<base::State>> extendGenSpine
			(const std::shared_ptr<base::State> q, const std::shared_ptr<base::State> q_e);
		std::tuple<base::State::Status, std::shared_ptr<std::vector<std::shared_ptr<base::State>>>> extendGenSpine2
			(const std::shared_ptr<base::State> q, const std::shared_ptr<base::State> q_e);
		base::State::Status connectGenSpine(const std::shared_ptr<base::Tree> tree, const std::shared_ptr<base::State> q, 
											const std::shared_ptr<base::State> q_e);
	};
}

#endif //RPMPL_RGBTCONNECT_H