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
