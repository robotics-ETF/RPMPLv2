#include "DRGBT.h"

/// @brief In case DRGBTConfig::TRAJECTORY_INTERPOLATION == "Spline", discretely check the validity of motion 
/// when the robot moves over splines 'splines->spline_current' and 'splines->spline_next' during the current iteration.
/// @brief In case DRGBTConfig::TRAJECTORY_INTERPOLATION == "None", discretely check the validity of motion 
/// when the robot moves from 'q_previous' to 'q_current'. 
/// During this checking (in both cases) obstacles are moving simultaneously. 
/// Finally, environment is updated within this function.
/// @param num_checks Number of checks of motion validity, which depends on maximal velocity of obstacles.
/// @return Validity of motion.
/// @note In reality, this motion would happen continuously during the execution of the current algorithm iteration.
bool planning::drbt::DRGBT::checkMotionValidity(size_t num_checks)
{
    // std::cout << "Checking the validity of motion while updating environment... \n";
    bool is_valid { true };
    std::shared_ptr<base::State> q_temp { nullptr };

    switch (DRGBTConfig::TRAJECTORY_INTERPOLATION)
    {
    case planning::TrajectoryInterpolation::Spline:
    {
        size_t num_checks1 = std::ceil((splines->spline_current->getTimeCurrent() - splines->spline_current->getTimeBegin()) * num_checks / DRGBTConfig::MAX_ITER_TIME);
        size_t num_checks2 { num_checks - num_checks1 };
        float delta_time1 { (splines->spline_current->getTimeCurrent() - splines->spline_current->getTimeBegin()) / num_checks1 };
        float delta_time2 { (splines->spline_next->getTimeEnd() - splines->spline_next->getTimeCurrent()) / num_checks2 };
        float t { 0 };
        
        // std::cout << "Current spline times:   " << splines->spline_current->getTimeBegin() * 1000 << " [ms] \t"
        //                                         << splines->spline_current->getTimeCurrent() * 1000 << " [ms] \t"
        //                                         << splines->spline_current->getTimeEnd() * 1000 << " [ms] \n";
        // std::cout << "Next spline times:      " << splines->spline_next->getTimeBegin() * 1000 << " [ms] \t"
        //                                         << splines->spline_next->getTimeCurrent() * 1000 << " [ms] \t"
        //                                         << splines->spline_next->getTimeEnd() * 1000 << " [ms] \n";

        for (size_t num_check = 1; num_check <= num_checks; num_check++)
        {
            if (num_check <= num_checks1)
            {
                t = splines->spline_current->getTimeBegin() + num_check * delta_time1;
                q_temp = ss->getNewState(splines->spline_current->getPosition(t));
                // std::cout << "t: " << t * 1000 << " [ms]\t from curr. spline \t" << q_temp << "\n";
                ss->env->updateEnvironment(delta_time1);
            }
            else
            {
                t = splines->spline_next->getTimeCurrent() + (num_check - num_checks1) * delta_time2;
                q_temp = ss->getNewState(splines->spline_next->getPosition(t));
                // std::cout << "t: " << t * 1000 << " [ms]\t from next  spline \t" << q_temp << "\n";
                ss->env->updateEnvironment(delta_time2);
            }

            path.emplace_back(q_temp);
            is_valid = ss->isValid(q_temp) && !ss->robot->checkSelfCollision(q_temp);
            if (!is_valid || ss->isEqual(q_temp, q_goal))
                break;
        }
        break;
    }

    case planning::TrajectoryInterpolation::None:
    {
        float dist { ss->getNorm(q_previous, q_current) };
        float delta_time { DRGBTConfig::MAX_ITER_TIME / num_checks };

        for (size_t num_check = 1; num_check <= num_checks; num_check++)
        {
            ss->env->updateEnvironment(delta_time);
            q_temp = ss->interpolateEdge(q_previous, q_current, dist * num_check / num_checks, dist);
            path.emplace_back(q_temp);
            is_valid = ss->isValid(q_temp) && !ss->robot->checkSelfCollision(q_temp);
            if (!is_valid)
                break;
        }
        break;
    }
    }

    // std::cout << "Environment objects: \n";
    // for (size_t i = 0; i < ss->env->getNumObjects(); i++)
    //     std::cout << "i = " << i << " : " << ss->env->getObject(i)->getPosition().transpose() << "\n";

    return is_valid;
}
