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

/// @brief Get elapsed time from 'time_init' to now.
/// @param time_init Time point from which measuring time starts.
/// @param time_unit Time unit in which elapsed time is returned.
/// @return Elapsed time in specified 'time_unit'. Default: seconds.
float planning::AbstractPlanner::getElapsedTime(const std::chrono::steady_clock::time_point &time_init, const planning::time_unit time_unit)
{
	try
	{
		auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - time_init).count();

		switch (time_unit)
		{
		case planning::time_unit::s:
			return time * 1e-9;

		case planning::time_unit::ms:
			return time * 1e-6;

		case planning::time_unit::us:
			return time * 1e-3;

		case planning::time_unit::ns:
			return float(time);
		
		default:
			throw std::runtime_error("Error in measuring time! ");
		}
	}
	catch(std::exception &e)
	{
		std::cout << e.what() << std::endl;
		return -1;
	}	
}
