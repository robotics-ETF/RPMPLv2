//
// Created by dinko on 16.3.21..
//

#include "AbstractPlanner.h"

planning::AbstractPlanner::AbstractPlanner(std::shared_ptr<base::StateSpace> ss_)
{
    ss = ss_;
    q_start = nullptr;
    q_goal = nullptr;
    planner_info = std::make_shared<PlannerInfo>();
}

planning::AbstractPlanner::AbstractPlanner(const std::shared_ptr<base::StateSpace> ss_, const std::shared_ptr<base::State> q_start_, 
                                           const std::shared_ptr<base::State> q_goal_)
{
    ss = ss_;
    q_start = q_start_;
    q_goal = q_goal_;
    planner_info = std::make_shared<PlannerInfo>();
}

planning::AbstractPlanner::~AbstractPlanner() {}

// Get elapsed time (in milliseconds by default) from 'time_init' to 'time_current'
int planning::AbstractPlanner::getElapsedTime(const std::chrono::steady_clock::time_point &time_init,
											  const std::chrono::steady_clock::time_point &time_current,
										      const std::string &time_unit)
{
	try
	{
		if (time_unit == "ms")
			return std::chrono::duration_cast<std::chrono::milliseconds>(time_current - time_init).count();
		else if (time_unit == "us")
			return std::chrono::duration_cast<std::chrono::microseconds>(time_current - time_init).count();
		else if (time_unit == "ns")
			return std::chrono::duration_cast<std::chrono::nanoseconds>(time_current - time_init).count();
		else
			throw std::runtime_error("Error in measuring time! ");
	}
	catch(std::exception &e)
	{
		std::cout << e.what() << std::endl;
		return -1;
	}	
}