#include "DRGBT.h"

/// @brief Decide whether to replan the predefined path from a current to the goal configuration.
/// @return Decision whether the replanning is needed.
bool planning::drbt::DRGBT::whetherToReplan()
{
    if (replanning)
        return true;
    
    float weight_max { 0 };
    float weight_sum { 0 };
    for (size_t i = 0; i < horizon.size(); i++)
    {
        weight_max = std::max(weight_max, horizon[i]->getWeight());
        weight_sum += horizon[i]->getWeight();
    }
    return (weight_max <= DRGBTConfig::TRESHOLD_WEIGHT && 
            weight_sum / horizon.size() <= DRGBTConfig::TRESHOLD_WEIGHT) 
            ? true : false;
}

/// @brief Initialize a static planner to plan a path from 'q_target' to 'q_goal' during a specified time limit 'max_planning_time'.
/// @param max_planning_time Maximal planning time.
/// @return Static planner that will be used for (re)planning.
std::unique_ptr<planning::AbstractPlanner> planning::drbt::DRGBT::initStaticPlanner(float max_planning_time)
{
    // std::cout << "Static planner (for replanning): " << DRGBTConfig::STATIC_PLANNER_TYPE << "\n";
    switch (DRGBTConfig::STATIC_PLANNER_TYPE)
    {
    case planning::PlannerType::RGBMTStar:
        RGBMTStarConfig::MAX_PLANNING_TIME = max_planning_time;
        return std::make_unique<planning::rbt_star::RGBMTStar>(ss, q_target, q_goal);

    case planning::PlannerType::RGBTConnect:
        RGBTConnectConfig::MAX_PLANNING_TIME = max_planning_time;
        return std::make_unique<planning::rbt::RGBTConnect>(ss, q_target, q_goal);
    
    case planning::PlannerType::RBTConnect:
        RBTConnectConfig::MAX_PLANNING_TIME = max_planning_time;
        return std::make_unique<planning::rbt::RBTConnect>(ss, q_target, q_goal);

    case planning::PlannerType::RRTConnect:
        RRTConnectConfig::MAX_PLANNING_TIME = max_planning_time;
        return std::make_unique<planning::rrt::RRTConnect>(ss, q_target, q_goal);

    default:
        throw std::domain_error("The requested static planner is not found! ");
    }
}

/// @brief Try to replan the predefined path from 'q_target' to 'q_goal' during a specified time limit 'max_planning_time'.
/// @param max_planning_time Maximal planning time.
void planning::drbt::DRGBT::replan(float max_planning_time)
{
    std::unique_ptr<planning::AbstractPlanner> planner { nullptr };
    bool result { false };

    try
    {
        if (max_planning_time <= 0)
            throw std::runtime_error("Not enough time for replanning! ");

        switch (DRGBTConfig::REAL_TIME_SCHEDULING)
        {
        case planning::RealTimeScheduling::FPS:
            // std::cout << "Replanning with Fixed Priority Scheduling \n";
            // std::cout << "Trying to replan in " << max_planning_time * 1e3 << " [ms]... \n";
            planner = initStaticPlanner(max_planning_time);
            result = planner->solve();
            break;
        
        case planning::RealTimeScheduling::None:
            // std::cout << "Replanning without real-time scheduling \n";
            // std::cout << "Trying to replan in " << max_planning_time * 1e3 << " [ms]... \n";
            planner = initStaticPlanner(max_planning_time);
            result = planner->solve();
            break;
        }

        // New path is found within the specified time limit, thus update the predefined path to the goal
        if (result && planner->getPlannerInfo()->getPlanningTime() <= max_planning_time)
        {
            // std::cout << "The path has been replanned in " << planner->getPlannerInfo()->getPlanningTime() * 1000 << " [ms]. \n";
            ss->preprocessPath(planner->getPath(), predefined_path, max_edge_length);
            clearHorizon(base::State::Status::Reached, false);
            q_next = std::make_shared<planning::drbt::HorizonState>(q_target, 0);
            planner_info->addRoutineTime(planner->getPlannerInfo()->getPlanningTime() * 1e3, 0);  // replan
        }
        else    // New path is not found, and just continue with the previous motion. We can also impose the robot to stop.
            throw std::runtime_error("New path is not found! ");
    }
    catch (std::exception &e)
    {
        // std::cout << "Replanning is required. " << e.what() << "\n";
        replanning = true;
    }
}
