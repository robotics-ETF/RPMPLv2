//
// Created by dinko on 26.5.21..
//

#include "PlannerInfo.h"

PlannerInfo::~PlannerInfo()
{
	iteration_times.clear();
	state_times.clear();
	cost_convergence.clear();
	routine_times.clear();
}

void PlannerInfo::addIterationTime(int time)
{
	iteration_times.emplace_back(time);
}

void PlannerInfo::addStateTimes(const std::vector<int> &state_times)
{
	for (size_t i = 0; i < state_times.size(); i++)
		PlannerInfo::state_times.emplace_back(state_times[i]);
}

void PlannerInfo::addCostConvergence(const std::vector<float> &cost_convergence)
{
	for (size_t i = 0; i < cost_convergence.size(); i++)
		PlannerInfo::cost_convergence.emplace_back(cost_convergence[i]);
}

void PlannerInfo::addRoutineTime(int time, int idx)
{
	for (int i = routine_times.size(); i <= idx; i++)
		routine_times.emplace_back(std::vector<int>());
	routine_times[idx].emplace_back(time);
}

void PlannerInfo::clearPlannerInfo()
{
	iteration_times.clear();
	state_times.clear();
	cost_convergence.clear();
	routine_times.clear();
	planning_time = 0;
	num_collision_queries = 0;
	num_distance_queries = 0;
	num_states = 0;
	num_iterations = 0;
}
