//
// Created by dinko on 16.3.21..
//

#include "AbstractPlanner.h"

planning::AbstractPlanner::AbstractPlanner(std::shared_ptr<base::StateSpace> ss_)
{
    ss = ss_;
    start = nullptr;
    goal = nullptr;
    planner_info = std::make_shared<PlannerInfo>();
}

planning::AbstractPlanner::AbstractPlanner(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_, 
                                                                                  std::shared_ptr<base::State> goal_)
{
    ss = ss_;
    start = start_;
    goal = goal_;
    planner_info = std::make_shared<PlannerInfo>();
}

planning::AbstractPlanner::~AbstractPlanner() {}

// Get elapsed time (defaultly in milliseconds) from 'time_start' to 'time_current'
float planning::AbstractPlanner::getElapsedTime(const std::chrono::steady_clock::time_point &time_start,
												const std::chrono::steady_clock::time_point &time_current,
												const std::string &time_unit)
{
	if (time_unit == "milliseconds")
		return std::chrono::duration_cast<std::chrono::milliseconds>(time_current - time_start).count();
	else if (time_unit == "microseconds")
		return std::chrono::duration_cast<std::chrono::microseconds>(time_current - time_start).count();
	else
		std::cout << "Error in measuring time!" << std::endl;
}