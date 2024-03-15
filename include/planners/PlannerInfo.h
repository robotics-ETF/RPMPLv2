//
// Created by dinko on 26.5.21..
//

#ifndef RPMPL_PLANNERINFO_H
#define RPMPL_PLANNERINFO_H

#include <vector>

class PlannerInfo
{
protected:
	std::vector<int> iteration_times;
	std::vector<int> state_times;
	std::vector<float> cost_convergence;			// Cost vs state convergence rate (cost-state curve)
	std::vector<std::vector<int>> routine_times; 	// Running times for the specified routine
	int planning_time;
	int num_collision_queries;
	int num_distance_queries;
	int num_states;
	int num_iterations;
	bool success_state = false;						// Did the planner succeed to find a solution?
	bool task1_interrupted;							// Whether Task 1 is interrupted

public:
	PlannerInfo() {}
	~PlannerInfo();
	
	void addIterationTime(int time);
	void addStateTimes(const std::vector<int> &state_times);
	void addCostConvergence(const std::vector<float> &cost_convergence);
	void addRoutineTime(int time, int idx);
	inline void setPlanningTime(int planning_time_) { planning_time = planning_time_; }
	inline void setNumCollisionQueries(int num_collision_queries_) { num_collision_queries = num_collision_queries_; }
	inline void setNumDistanceQueries(int num_distance_queries_) { num_distance_queries = num_distance_queries_; }
	inline void setNumStates(int num_states_) { num_states = num_states_; }
	inline void setNumIterations(int num_iterations_) { num_iterations = num_iterations_; }
	inline void setSuccessState(bool success_state_) { success_state = success_state_; }
	inline void setTask1Interrupted(bool task1_interrupted_) { task1_interrupted = task1_interrupted_; }

	inline const std::vector<int> &getIterationTimes() const { return iteration_times; }
	inline const std::vector<int> &getStateTimes() const { return state_times; }
	inline const std::vector<float> &getCostConvergence() const { return cost_convergence; }
	inline const std::vector<std::vector<int>> &getRoutineTimes() const {return routine_times; }
	inline int getPlanningTime() const { return planning_time; }
	inline int getNumCollisionQueries() const { return num_collision_queries; }
	inline int getNumDistanceQueries() const { return num_distance_queries; }
	inline size_t getNumStates() const { return num_states; }
	inline size_t getNumIterations() const { return num_iterations; }
	inline bool getSuccessState() const { return success_state; }
	inline bool getTask1Interrupted() const { return task1_interrupted; }

	void clearPlannerInfo();

};

#endif //RPMPL_PLANNERINFO_H
