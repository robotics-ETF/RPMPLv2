#include "MotionValidity.h"

planning::trajectory::MotionValidity::MotionValidity(const std::shared_ptr<base::StateSpace> &ss_, 
    float resolution_coll_check_, float max_iter_time_, std::vector<std::shared_ptr<base::State>>* path_)
{
    ss = ss_;
    resolution_coll_check = resolution_coll_check_;
    max_iter_time = max_iter_time_;
    path = path_;

    float max_vel_obs { 0 };
    for (size_t i = 0; i < ss->env->getNumObjects(); i++)
    {
        if (ss->env->getObject(i)->getMaxVel() > max_vel_obs)
            max_vel_obs = ss->env->getObject(i)->getMaxVel();
    }

    // Maximal number of validity points from trajectory when robot moves from previous to current configuration, 
    // in order to obtain the check when obstacle moves at most 'resolution_coll_check' [m] (while the obstacles are moving simultaneously).
    TrajectoryConfig::NUM_VALIDITY_POINTS_CHECK = std::ceil((max_vel_obs * max_iter_time) / resolution_coll_check);
    if (TrajectoryConfig::NUM_VALIDITY_POINTS_CHECK < 2)
        TrajectoryConfig::NUM_VALIDITY_POINTS_CHECK = 2;   // Default value in case 'max_vel_obs' is too small
}

/// @brief In case traj_interpolation == "None", discretely check the validity of motion 
/// when the robot moves from 'q_previous' to 'q_current'. 
/// During this checking obstacles are moving simultaneously. Finally, environment is updated within this function.
/// @return Validity of motion.
/// @note In reality, this motion would happen continuously during the execution of the current algorithm iteration.
bool planning::trajectory::MotionValidity::check(const std::shared_ptr<base::State> &q_previous, const std::shared_ptr<base::State> &q_current)
{
    // std::cout << "Checking the validity of motion while updating environment... \n";
    std::shared_ptr<base::State> q { nullptr };
    float dist { ss->getNorm(q_previous, q_current) };
    float delta_time { max_iter_time / TrajectoryConfig::NUM_VALIDITY_POINTS_CHECK };

    for (size_t num_check = 1; num_check <= TrajectoryConfig::NUM_VALIDITY_POINTS_CHECK; num_check++)
    {
        ss->env->updateEnvironment(delta_time);

        q = ss->interpolateEdge(q_previous, q_current, dist * num_check / TrajectoryConfig::NUM_VALIDITY_POINTS_CHECK, dist);
        if (!ss->isValid(q) || ss->robot->checkSelfCollision(q))
            return false;

        path->emplace_back(q);
    }

    // std::cout << "Environment objects: \n";
    // for (size_t i = 0; i < ss->env->getNumObjects(); i++)
    //     std::cout << "i = " << i << " : " << ss->env->getObject(i)->getPosition().transpose() << "\n";

    return true;
}

/// @brief In case traj_interpolation == "Spline" or "Ruckig", discretely check the validity of motion 
/// when the robot moves over the current trajectory during the current iteration.
/// During this checking obstacles are moving simultaneously. Finally, environment is updated within this function.
/// @return Validity of motion.
/// @note In reality, this motion would happen continuously during the execution of the current algorithm iteration.
bool planning::trajectory::MotionValidity::check(const std::vector<Eigen::VectorXf> &traj_points_current_iter)
{
    // std::cout << "Checking the validity of motion while updating environment... \n";
    std::shared_ptr<base::State> q { nullptr };
    float delta_time { max_iter_time / TrajectoryConfig::NUM_VALIDITY_POINTS_CHECK };

    for (Eigen::VectorXf point : traj_points_current_iter)
    {
        ss->env->updateEnvironment(delta_time);

        q = ss->getNewState(point);
        if (!ss->isValid(q) || ss->robot->checkSelfCollision(q))
            return false;

        path->emplace_back(q);
    }
    
    // std::cout << "Environment objects: \n";
    // for (size_t i = 0; i < ss->env->getNumObjects(); i++)
    //     std::cout << "i = " << i << " : " << ss->env->getObject(i)->getPosition().transpose() << "\n";

    return true;
}