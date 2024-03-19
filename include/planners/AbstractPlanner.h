//
// Created by dinko on 16.3.21..
//
#ifndef RPMPL_ABSTRACTPLANNER_H
#define RPMPL_ABSTRACTPLANNER_H

#include <vector>
#include <string>
#include <memory>
#include <chrono>

#include "StateSpace.h"
#include "PlannerInfo.h"

namespace planning
{
	enum time_unit {s, ms, us, ns};

	class AbstractPlanner
	{
	public:
		AbstractPlanner(const std::shared_ptr<base::StateSpace> ss_);
		AbstractPlanner(const std::shared_ptr<base::StateSpace> ss_, 
						const std::shared_ptr<base::State> q_start_, const std::shared_ptr<base::State> q_goal_);
		virtual ~AbstractPlanner() = 0;
		
		inline std::shared_ptr<base::StateSpace> getStateSpace() const { return ss; }
		inline std::shared_ptr<PlannerInfo> getPlannerInfo() const { return planner_info; }
		virtual const std::vector<std::shared_ptr<base::State>> &getPath() const = 0;

		virtual bool solve() = 0;
		virtual bool checkTerminatingCondition(base::State::Status status) = 0;
		virtual void outputPlannerData(const std::string &filename, bool output_states_and_paths = true, bool append_output = false) const = 0;
		float getElapsedTime(const std::chrono::steady_clock::time_point &time_init, const planning::time_unit time_unit = planning::time_unit::s);

	protected:
		std::shared_ptr<base::StateSpace> ss;
		std::shared_ptr<PlannerInfo> planner_info;
		std::shared_ptr<base::State> q_start;
		std::shared_ptr<base::State> q_goal;
		std::vector<std::shared_ptr<base::State>> path;
		std::chrono::steady_clock::time_point time_alg_start;		// Start time point of the used algorithm
		std::chrono::steady_clock::time_point time_iter_start;   	// Start time point at each iteration
	};
}
#endif //RPMPL_ABSTRACTPLANNER_H
