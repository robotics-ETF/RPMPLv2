//
// Created by dinko on 16.3.21.
// Modified by nermin on 18.02.22.
//

#ifndef RPMPL_RRTCONNECT_H
#define RPMPL_RRTCONNECT_H

#include "AbstractPlanner.h"
#include "Tree.h"
#include "RRTConnectConfig.h"

// #include <glog/log_severity.h>
// #include <glog/logging.h>
// WARNING: You need to be very careful with using LOG(INFO) for console output, due to a possible "stack smashing detected" error.
// If you get this error, just use std::cout for console output.

namespace planning::rrt
{
	class RRTConnect : public AbstractPlanner
	{
	public:
		RRTConnect(const std::shared_ptr<base::StateSpace> ss_);
		RRTConnect(const std::shared_ptr<base::StateSpace> ss_, 
				   const std::shared_ptr<base::State> q_start_, const std::shared_ptr<base::State> q_goal_);
		~RRTConnect();
		
		bool solve() override;
		base::Tree getTree(size_t tree_idx) const;
		const std::vector<std::shared_ptr<base::State>> &getPath() const override;
		bool checkTerminatingCondition(base::State::Status status) override;
		void outputPlannerData(const std::string &filename, bool output_states_and_paths = true, bool append_output = false) const override;
		
	protected:
		std::vector<std::shared_ptr<base::Tree>> trees;
		
		std::tuple<base::State::Status, std::shared_ptr<base::State>> extend
			(const std::shared_ptr<base::State> q, const std::shared_ptr<base::State> q_e);
		base::State::Status connect(const std::shared_ptr<base::Tree> tree, const std::shared_ptr<base::State> q, 
									const std::shared_ptr<base::State> q_e);
		void computePath();
	};
}

#endif //RPMPL_RRTCONNECT_H