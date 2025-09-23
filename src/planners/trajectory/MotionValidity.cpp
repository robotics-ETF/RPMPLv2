#include "MotionValidity.h"

planning::trajectory::MotionValidity::MotionValidity(const std::shared_ptr<base::StateSpace> &ss_, 
    planning::TrajectoryInterpolation traj_interpolation_, float resolution_coll_check_, 
    std::vector<std::shared_ptr<base::State>>* path_, float max_iter_time_)
{
    ss = ss_;
    traj_interpolation = traj_interpolation_;
    resolution_coll_check = resolution_coll_check_;
    path = path_;
    max_iter_time = max_iter_time_;
    traj = nullptr;

    float max_vel_obs { 0 };
    for (size_t i = 0; i < ss->env->getNumObjects(); i++)
    {
        if (ss->env->getObject(i)->getMaxVel() > max_vel_obs)
            max_vel_obs = ss->env->getObject(i)->getMaxVel();
    }

    num_checks = std::ceil((max_vel_obs * max_iter_time) / resolution_coll_check);  // In order to obtain check when obstacle moves at most 'resolution_coll_check' [m]
    if (num_checks < 2)
        num_checks = 2;   // Default value in case 'max_vel_obs' is too small
}

/// @brief Discretely check the validity of motion. During this checking obstacles are moving simultaneously. 
/// Finally, environment is updated within this function.
/// @return Validity of motion.
/// @note In reality, this motion would happen continuously during the execution of the current algorithm iteration.
bool planning::trajectory::MotionValidity::check(const std::shared_ptr<base::State> &q_previous, const std::shared_ptr<base::State> &q_current)
{
    switch (traj_interpolation)
    {
    case planning::TrajectoryInterpolation::None:
        return check_v1(q_previous, q_current);
    
    case planning::TrajectoryInterpolation::Spline:
        return check_v2();

    default:
        return false;
    }
}

// In case traj_interpolation == "None", discretely check the validity of motion 
// when the robot moves from 'q_previous' to 'q_current'. 
bool planning::trajectory::MotionValidity::check_v1(const std::shared_ptr<base::State> &q_previous, const std::shared_ptr<base::State> &q_current)
{
    // std::cout << "Checking the validity of motion while updating environment... \n";
    std::shared_ptr<base::State> q_temp { nullptr };
    float dist { ss->getNorm(q_previous, q_current) };
    float delta_time { max_iter_time / num_checks };

    for (size_t num_check = 1; num_check <= num_checks; num_check++)
    {
        ss->env->updateEnvironment(delta_time);
        q_temp = ss->interpolateEdge(q_previous, q_current, dist * num_check / num_checks, dist);
        if (!ss->isValid(q_temp) || ss->robot->checkSelfCollision(q_temp))
            break;
    }

    // std::cout << "Environment objects: \n";
    // for (size_t i = 0; i < ss->env->getNumObjects(); i++)
    //     std::cout << "i = " << i << " : " << ss->env->getObject(i)->getPosition().transpose() << "\n";

    path->emplace_back(q_temp);
    return true;
}

// In case traj_interpolation == "Spline", discretely check the validity of motion 
// when the robot moves over splines 'traj->spline_current' and 'traj->spline_next' during the current iteration.
bool planning::trajectory::MotionValidity::check_v2()
{
    // std::cout << "Checking the validity of motion while updating environment... \n";
    std::shared_ptr<base::State> q_temp { nullptr };
    size_t num_checks1 = std::ceil((traj->spline_current->getTimeCurrent() - traj->spline_current->getTimeBegin()) * num_checks / max_iter_time);
    size_t num_checks2 { num_checks - num_checks1 };
    float delta_time1 { (traj->spline_current->getTimeCurrent() - traj->spline_current->getTimeBegin()) / num_checks1 };
    float delta_time2 { (traj->spline_next->getTimeEnd() - traj->spline_next->getTimeCurrent()) / num_checks2 };
    float t { 0 };
    
    // std::cout << "Current spline times:   " << traj->spline_current->getTimeBegin() * 1000 << " [ms] \t"
    //                                         << traj->spline_current->getTimeCurrent() * 1000 << " [ms] \t"
    //                                         << traj->spline_current->getTimeEnd() * 1000 << " [ms] \n";
    // std::cout << "Next spline times:      " << traj->spline_next->getTimeBegin() * 1000 << " [ms] \t"
    //                                         << traj->spline_next->getTimeCurrent() * 1000 << " [ms] \t"
    //                                         << traj->spline_next->getTimeEnd() * 1000 << " [ms] \n";

    for (size_t num_check = 1; num_check <= num_checks; num_check++)
    {
        if (num_check <= num_checks1)
        {
            t = traj->spline_current->getTimeBegin() + num_check * delta_time1;
            q_temp = ss->getNewState(traj->spline_current->getPosition(t));
            // std::cout << "t: " << t * 1000 << " [ms]\t from curr. spline \t" << q_temp << "\n";
            ss->env->updateEnvironment(delta_time1);
        }
        else
        {
            t = traj->spline_next->getTimeCurrent() + (num_check - num_checks1) * delta_time2;
            q_temp = ss->getNewState(traj->spline_next->getPosition(t));
            // std::cout << "t: " << t * 1000 << " [ms]\t from next  spline \t" << q_temp << "\n";
            ss->env->updateEnvironment(delta_time2);
        }

        if (!ss->isValid(q_temp) || ss->robot->checkSelfCollision(q_temp))
            return false;
    }
    
    // std::cout << "Environment objects: \n";
    // for (size_t i = 0; i < ss->env->getNumObjects(); i++)
    //     std::cout << "i = " << i << " : " << ss->env->getObject(i)->getPosition().transpose() << "\n";

    // std::cout << q_temp << "\n";
    path->emplace_back(q_temp);
    return true;
}
