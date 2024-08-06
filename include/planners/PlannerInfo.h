//
// Created by dinko on 26.5.21.
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
	float optimal_cost;
	float planning_time;
	size_t num_collision_queries;
	size_t num_distance_queries;
	size_t num_states;
	size_t num_iterations;
	bool success_state;								// Did the planner succeed to find a solution?
	bool task1_interrupted;							// Whether Task 1 is interrupted

public:
	PlannerInfo();
	~PlannerInfo();
	
	void addIterationTime(float time);
	void addStateTimes(const std::vector<float> &state_times);
	void addCostConvergence(const std::vector<float> &cost_convergence);
	void addRoutineTime(float time, size_t idx);
	inline void setOptimalCost(float optimal_cost_) { optimal_cost = optimal_cost_; }
	inline void setPlanningTime(float planning_time_) { planning_time = planning_time_; }
	inline void setNumCollisionQueries(size_t num_collision_queries_) { num_collision_queries = num_collision_queries_; }
	inline void setNumDistanceQueries(size_t num_distance_queries_) { num_distance_queries = num_distance_queries_; }
	inline void setNumStates(size_t num_states_) { num_states = num_states_; }
	inline void setNumIterations(size_t num_iterations_) { num_iterations = num_iterations_; }
	inline void setSuccessState(bool success_state_) { success_state = success_state_; }
	inline void setTask1Interrupted(bool task1_interrupted_) { task1_interrupted = task1_interrupted_; }

	inline const std::vector<float> &getIterationTimes() const { return iteration_times; }
	inline const std::vector<float> &getStateTimes() const { return state_times; }
	inline const std::vector<float> &getCostConvergence() const { return cost_convergence; }
	inline const std::vector<std::vector<float>> &getRoutineTimes() const {return routine_times; }
	inline float getOptimalCost() const { return optimal_cost; }
	inline float getPlanningTime() const { return planning_time; }
	inline size_t getNumCollisionQueries() const { return num_collision_queries; }
	inline size_t getNumDistanceQueries() const { return num_distance_queries; }
	inline size_t getNumStates() const { return num_states; }
	inline size_t getNumIterations() const { return num_iterations; }
	inline bool getSuccessState() const { return success_state; }
	inline bool getTask1Interrupted() const { return task1_interrupted; }

	void clearPlannerInfo();

};

#endif //RPMPL_PLANNERINFO_H