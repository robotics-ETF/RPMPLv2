//
// Created by dinko on 26.5.21..
//

#ifndef RPMPL_PLANNERINFO_H
#define RPMPL_PLANNERINFO_H

#include <vector>

class PlannerInfo
{
protected:
	std::vector<float> iteration_times;
	std::vector<float> state_times;
	std::vector<float> cost_convergence;			// Cost vs state convergence rate (cost-state curve)
	std::vector<std::vector<float>> routine_times; 	// Running times for the specified routine
	float planning_time;
	int num_collision_queries;
	int num_distance_queries;
	int num_states;
	int num_iterations;
	bool success_state = false;						// Did the planner succeed to find a solution?

public:
	PlannerInfo() {}
	~PlannerInfo();
	void addIterationTime(float time);
	void addStateTimes(const std::vector<float> &state_times);
	void addCostConvergence(const std::vector<float> &cost_convergence);
	void addRoutineTime(float time, int idx);
	void setPlanningTime(float planning_time_) { planning_time = planning_time_; }
	void setNumCollisionQueries(int num_collision_queries_) { num_collision_queries = num_collision_queries_; }
	void setNumDistanceQueries(int num_distance_queries_) { num_distance_queries = num_distance_queries_; }
	void setNumStates(int num_states_) { num_states = num_states_; }
	void setNumIterations(int num_iterations_) { num_iterations = num_iterations_; }
	void setSuccessState(bool success_state_) { success_state = success_state_; }

	const std::vector<float> &getIterationTimes() const { return iteration_times; }
	const std::vector<float> &getStateTimes() const { return state_times; }
	const std::vector<float> &getCostConvergence() const { return cost_convergence; }
	const std::vector<std::vector<float>> &getRoutineTimes() const {return routine_times; }
	float getPlanningTime() const { return planning_time; }
	int getNumCollisionQueries() const { return num_collision_queries; }
	int getNumDistanceQueries() const { return num_distance_queries; }
	int getNumStates() const { return num_states; }
	int getNumIterations() const { return num_iterations; }
	bool getSuccessState() const { return success_state; }

	void clearPlannerInfo();

};

#endif //RPMPL_PLANNERINFO_H
