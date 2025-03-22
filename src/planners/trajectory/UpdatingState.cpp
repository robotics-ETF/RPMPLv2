#include "UpdatingState.h"
#include "DRGBT.h"

planning::trajectory::UpdatingState::UpdatingState(const std::shared_ptr<base::StateSpace> &ss_, 
    planning::TrajectoryInterpolation traj_interpolation_, float max_iter_time_)
{
    ss = ss_;
    traj_interpolation = traj_interpolation_;
    max_iter_time = max_iter_time_;

    all_robot_vel_same = true;
    for (size_t i = 1; i < ss->num_dimensions; i++)
    {
        if (std::abs(ss->robot->getMaxVel(i) - ss->robot->getMaxVel(i-1)) > RealVectorSpaceConfig::EQUALITY_THRESHOLD)
        {
            all_robot_vel_same = false;
            break;
        }
    }

    splines = nullptr;
    guaranteed_safe_motion = false;
    non_zero_final_vel = true;
    max_remaining_iter_time = 0;
    time_iter_start = std::chrono::steady_clock::now();
    measure_time = false;
    remaining_time = 0;
    q_next = nullptr;
    drgbt_instance = nullptr;
}

void planning::trajectory::UpdatingState::update(std::shared_ptr<base::State> &q_previous, std::shared_ptr<base::State> &q_current, 
    const std::shared_ptr<base::State> &q_next_reached, base::State::Status &status)
{
    switch (traj_interpolation)
    {
    case planning::TrajectoryInterpolation::None:
        update_v1(q_previous, q_current, q_next_reached, status);
        break;
    
    case planning::TrajectoryInterpolation::Spline:
        update_v2(q_previous, q_current, q_next_reached, status);
        break;

    default:
        break;
    }
}

/// @brief Update a current state 'q_current' to a new desired state 'q_current_new' from the edge [q_current - q_next_reached], 
/// such that it can be reached within max_iter_time time, while considering robot maximal velocity.
/// In other words, 'q_current_new' is determined using an advancing step size which depends on robot's maximal velocity.
/// Move 'q_current' to 'q_current_new' meaning that 'q_current' will be updated to a robot position from the end of current iteration.
/// @note If 'q_next' is different from 'q_next_reached', the user is required to set 'q_next' via 'setNextState' function.
void planning::trajectory::UpdatingState::update_v1(std::shared_ptr<base::State> &q_previous, std::shared_ptr<base::State> &q_current, 
    const std::shared_ptr<base::State> &q_next_reached, base::State::Status &status)
{
    q_previous = q_current;
    if (status == base::State::Status::Trapped)     // Current robot position will not be updated! 
    {                                               // We must wait for successful replanning to change 'status' to 'Reached'
        // std::cout << "Status: Trapped! \n";
        return;
    }
    if (q_next == nullptr)
        q_next = q_next_reached;

    std::shared_ptr<base::State> q_current_new { ss->getNewState(q_next_reached->getCoord()) };
    if (!ss->isEqual(q_current, q_current_new))
    {
        if (all_robot_vel_same)
        {
            float max_edge_length_ { ss->robot->getMaxVel(0) * max_iter_time };
            if (ss->getNorm(q_current, q_current_new) > max_edge_length_)
                q_current_new = ss->pruneEdge2(q_current, q_current_new, max_edge_length_);
        }
        else
        {
            std::vector<std::pair<float, float>> limits {};
            for (size_t i = 0; i < ss->num_dimensions; i++)
            {
                limits.emplace_back(std::pair<float, float>
                (q_current->getCoord(i) - ss->robot->getMaxVel(i) * max_iter_time, 
                    q_current->getCoord(i) + ss->robot->getMaxVel(i) * max_iter_time));
            }
            q_current_new = ss->pruneEdge(q_current, q_current_new, limits);
        }
    }

    q_current = q_current_new;   // Current robot position at the end of iteration
    if (ss->isEqual(q_current, q_next))
        status = base::State::Status::Reached;      // 'q_next' must be reached, and not only 'q_next_reached'
    else
        status = base::State::Status::Advanced;
    
    q_next = nullptr;

    // std::cout << "q_current: " << q_current << "\n";
    // std::cout << "Status: " << (status == base::State::Status::Advanced ? "Advanced" : "")
    //                         << (status == base::State::Status::Reached  ? "Reached"  : "") << "\n";
}

/// @brief Update a current state of the robot using 'splines->spline_current'.
/// Compute a new spline 'splines->spline_next', or remain 'splines->spline_current'.
/// Move 'q_current' towards 'q_next_reached' while following 'splines->spline_next'.
/// 'q_current' will be updated to a robot position from the end of current iteration.
/// @note The new spline will be computed in a way that all constraints on robot's maximal velocity, 
/// acceleration and jerk are surely always satisfied.
/// @note If 'q_next' is different from 'q_next_reached', the user is required to set 'q_next' via 'setNextState' function.
void planning::trajectory::UpdatingState::update_v2(std::shared_ptr<base::State> &q_previous, std::shared_ptr<base::State> &q_current, 
    const std::shared_ptr<base::State> &q_next_reached, base::State::Status &status)
{
    splines->spline_current = splines->spline_next;
    q_previous = q_current;

    float t_spline_max { (guaranteed_safe_motion ? SplinesConfig::MAX_TIME_COMPUTE_SAFE : SplinesConfig::MAX_TIME_COMPUTE_REGULAR) - 
                        SplinesConfig::MAX_TIME_PUBLISH * measure_time };
    float t_iter { getElapsedTime() };
    if (max_iter_time - max_remaining_iter_time - t_iter < t_spline_max)
        t_spline_max = max_iter_time - max_remaining_iter_time - t_iter;
    
    float t_iter_remain { max_iter_time - t_iter - t_spline_max };
    float t_spline_current { measure_time ? 
                            splines->spline_current->getTimeCurrent(true) + t_spline_max :
                            splines->spline_current->getTimeEnd() + t_iter + t_spline_max };

    splines->spline_current->setTimeBegin(splines->spline_current->getTimeEnd());
    splines->spline_current->setTimeCurrent(t_spline_current);

    // std::cout << "Iter. time:        " << t_iter * 1000 << " [ms] \n";
    // std::cout << "Max. spline time:  " << t_spline_max * 1000 << " [ms] \n";
    // std::cout << "Remain. time:      " << t_iter_remain * 1000 << " [ms] \n";
    // std::cout << "Begin spline time: " << splines->spline_current->getTimeBegin() * 1000 << " [ms] \n";
    // std::cout << "Curr. spline time: " << t_spline_current * 1000 << " [ms] \n";
    // ----------------------------------------------------------------------------------------- //
    
    bool spline_computed { false };
    float t_spline_remain {};
    Eigen::VectorXf current_pos { splines->spline_current->getPosition(t_spline_current) };
    Eigen::VectorXf current_vel { splines->spline_current->getVelocity(t_spline_current) };
    Eigen::VectorXf current_acc { splines->spline_current->getAcceleration(t_spline_current) };

    // std::cout << "Curr. pos: " << current_pos.transpose() << "\n";
    // std::cout << "Curr. vel: " << current_vel.transpose() << "\n";
    // std::cout << "Curr. acc: " << current_acc.transpose() << "\n";

    do
    {
        t_spline_remain = t_spline_max - (getElapsedTime() - t_iter);
        // std::cout << "t_spline_remain: " << t_spline_remain * 1e3 << " [ms] \n";
        if (t_spline_remain < 0)
            break;

        splines->setCurrentState(q_current);
        splines->setTargetState(q_next_reached);
        // std::cout << "q_next: " << q_next << "\n";
        if (splines->spline_current->isFinalConf(q_next_reached->getCoord()))  // Spline to such 'q_next_reached' already exists!
            break;

        if (guaranteed_safe_motion)
            spline_computed = splines->computeSafe(current_pos, current_vel, current_acc, t_iter_remain, t_spline_remain);
        else
            spline_computed = splines->computeRegular(current_pos, current_vel, current_acc, t_iter_remain, t_spline_remain, non_zero_final_vel);
    }
    while (!spline_computed && invokeChangeNextState());
    // std::cout << "Elapsed time for spline computing: " << (getElapsedTime() - t_iter) * 1e3 << " [ms] \n";

    if (spline_computed)
    {
        // std::cout << "New spline is computed! \n";
        splines->spline_current->setTimeEnd(t_spline_current);
        splines->spline_next->setTimeEnd(t_iter_remain);
    }
    else
    {
        // std::cout << "Continuing with the previous spline! \n";
        splines->spline_next = splines->spline_current;
        splines->spline_next->setTimeEnd(t_spline_current + t_iter_remain);
    }
    
    // splines->recordTrajectory(spline_computed);   // Only for debugging

    q_current = ss->getNewState(splines->spline_next->getPosition(splines->spline_next->getTimeEnd()));   // Current robot position at the end of iteration
    if (status != base::State::Status::Trapped)
    {
        if (splines->spline_next->getTimeFinal() < splines->spline_next->getTimeEnd() + 2*max_iter_time - max_remaining_iter_time)
            status = base::State::Status::Reached;  // 'q_next' must be reached, and not only 'q_next_reached'
        else
            status = base::State::Status::Advanced;
    }

    // std::cout << "Spline next: \n" << splines->spline_next << "\n";
    // std::cout << "q_current: " << q_current << "\n";
    // std::cout << "Status: " << (status == base::State::Status::Advanced ? "Advanced" : "")
    //                         << (status == base::State::Status::Trapped  ? "Trapped"  : "")
    //                         << (status == base::State::Status::Reached  ? "Reached"  : "") << "\n";

    remaining_time = t_spline_max + SplinesConfig::MAX_TIME_PUBLISH * measure_time - (getElapsedTime() - t_iter);
}

bool planning::trajectory::UpdatingState::invokeChangeNextState() 
{
    if (drgbt_instance != nullptr) 
        return drgbt_instance->changeNextState();
    
    return false;
}

float planning::trajectory::UpdatingState::getElapsedTime()
{
    float t = std::chrono::duration_cast<std::chrono::nanoseconds>
              (std::chrono::steady_clock::now() - time_iter_start).count() * 1e-9;
    return t;
}
