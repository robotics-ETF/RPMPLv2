#include "DRGBT.h"

/// @brief Update a current state of the robot using 'splines->spline_current'.
/// Compute a new spline 'splines->spline_next', or remain 'splines->spline_current'.
/// Move 'q_current' towards 'q_next' while following 'splines->spline_next'.
/// 'q_current' will be updated to a robot position from the end of current iteration.
/// @param measure_time If true, elapsed time when computing a spline will be exactly measured. 
/// If false, elapsed time will be computed (default: false).
/// @return Remaining time in [s] after which the new spline 'splines->spline_next' will become active.
/// @note The new spline will be computed in a way that all constraints on robot's maximal velocity, 
/// acceleration and jerk are surely always satisfied.
/// @note 'measure_time' should always be false when simulation pacing is used, since then a time measuring will not be correct! 
/// In such case, it is assumed that user was previously set 'measure_time' to a correct value.
float planning::drbt::DRGBT::updateCurrentState(bool measure_time)
{
    splines->spline_current = splines->spline_next;
    q_previous = q_current;

    float t_spline_max { (DRGBTConfig::GUARANTEED_SAFE_MOTION ? SplinesConfig::MAX_TIME_COMPUTE_SAFE : SplinesConfig::MAX_TIME_COMPUTE_REGULAR) - 
                         SplinesConfig::MAX_TIME_PUBLISH * measure_time };
    float t_iter { getElapsedTime(time_iter_start) };
    if (DRGBTConfig::MAX_TIME_TASK1 - t_iter < t_spline_max)
        t_spline_max = DRGBTConfig::MAX_TIME_TASK1 - t_iter;
    
    float t_iter_remain { DRGBTConfig::MAX_ITER_TIME - t_iter - t_spline_max };
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
    std::vector<std::shared_ptr<planning::drbt::HorizonState>> visited_states { q_next };

    // std::cout << "Curr. pos: " << current_pos.transpose() << "\n";
    // std::cout << "Curr. vel: " << current_vel.transpose() << "\n";
    // std::cout << "Curr. acc: " << current_acc.transpose() << "\n";

    do
    {
        t_spline_remain = t_spline_max - (getElapsedTime(time_iter_start) - t_iter);
        // std::cout << "t_spline_remain: " << t_spline_remain * 1e3 << " [ms] \n";
        if (t_spline_remain < 0)
            break;

        splines->setCurrentState(q_current);
        splines->setTargetState(q_next->getStateReached());
        // std::cout << "q_next: " << q_next << "\n";
        if (splines->spline_current->isFinalConf(q_next->getCoordReached()))  // Spline to such 'q_next->getCoordReached()' already exists!
            break;

        if (DRGBTConfig::GUARANTEED_SAFE_MOTION)
            spline_computed = splines->computeSafe(current_pos, current_vel, current_acc, t_iter_remain, t_spline_remain);
        else
            spline_computed = splines->computeRegular(current_pos, current_vel, current_acc, t_iter_remain, t_spline_remain, 
                                                      q_next->getIsReached() && q_next->getIndex() != -1 && 
                                                      q_next->getStatus() != planning::drbt::HorizonState::Status::Goal);
    }
    while (!spline_computed && changeNextState(visited_states));
    // std::cout << "Elapsed time for spline computing: " << (getElapsedTime(time_iter_start) - t_iter) * 1e3 << " [ms] \n";

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
        if (splines->spline_next->getTimeFinal() < splines->spline_next->getTimeEnd() + DRGBTConfig::MAX_ITER_TIME + DRGBTConfig::MAX_TIME_TASK1)
            status = base::State::Status::Reached;  // 'q_next->getState()' must be reached, and not only 'q_next->getStateReached()'
        else
            status = base::State::Status::Advanced;
    }

    // std::cout << "Spline next: \n" << splines->spline_next << "\n";
    // std::cout << "q_current: " << q_current << "\n";
    // std::cout << "Status: " << (status == base::State::Status::Advanced ? "Advanced" : "")
    //                         << (status == base::State::Status::Trapped  ? "Trapped"  : "")
    //                         << (status == base::State::Status::Reached  ? "Reached"  : "") << "\n";

    return t_spline_max + SplinesConfig::MAX_TIME_PUBLISH * measure_time - (getElapsedTime(time_iter_start) - t_iter);
}

/// @brief Update a current state 'q_current' to a new desired state 'q_current_new' from the edge [q_current - q_next_reached], 
/// such that it can be reached within DRGBTConfig::MAX_ITER_TIME time, while considering robot maximal velocity.
/// In other words, 'q_current_new' is determined using an advancing step size which depends on robot's maximal velocity.
/// Move 'q_current' to 'q_current_new' meaning that 'q_current' will be updated to a robot position from the end of current iteration.
void planning::drbt::DRGBT::updateCurrentState()
{
    q_previous = q_current;
    if (status == base::State::Status::Trapped)     // Current robot position will not be updated! 
    {                                               // We must wait for successful replanning to change 'status' to 'Reached'
        // std::cout << "Status: Trapped! \n";
        return;
    }

    std::shared_ptr<base::State> q_current_new { ss->getNewState(q_next->getCoordReached()) };
    if (!ss->isEqual(q_current, q_current_new))
    {
        if (all_robot_vel_same)
        {
            float max_edge_length_ { ss->robot->getMaxVel(0) * DRGBTConfig::MAX_ITER_TIME };
            if (ss->getNorm(q_current, q_current_new) > max_edge_length_)
                q_current_new = ss->pruneEdge2(q_current, q_current_new, max_edge_length_);
        }
        else
        {
            std::vector<std::pair<float, float>> limits {};
            for (size_t i = 0; i < ss->num_dimensions; i++)
            {
                limits.emplace_back(std::pair<float, float>
                (q_current->getCoord(i) - ss->robot->getMaxVel(i) * DRGBTConfig::MAX_ITER_TIME, 
                    q_current->getCoord(i) + ss->robot->getMaxVel(i) * DRGBTConfig::MAX_ITER_TIME));
            }
            q_current_new = ss->pruneEdge(q_current, q_current_new, limits);
        }
    }

    q_current = q_current_new;   // Current robot position at the end of iteration
    if (ss->isEqual(q_current, q_next->getState()))
        status = base::State::Status::Reached;      // 'q_next->getState()' must be reached, and not only 'q_next->getStateReached()'
    else
        status = base::State::Status::Advanced;

    // std::cout << "q_current: " << q_current << "\n";
    // std::cout << "Status: " << (status == base::State::Status::Advanced ? "Advanced" : "")
    //                         << (status == base::State::Status::Reached  ? "Reached"  : "") << "\n";
}

/// @brief Choose the best state from the horizon so that it does not belong to 'visited_states'.
/// @param visited_states Set of visited states.
/// @return Success of a change.
bool planning::drbt::DRGBT::changeNextState(std::vector<std::shared_ptr<planning::drbt::HorizonState>> &visited_states)
{
    // std::cout << "Change of q_next is required! \n";
    std::shared_ptr<planning::drbt::HorizonState> q_new { nullptr };
    float weight_max { 0 };
    bool visited { false };

    for (std::shared_ptr<planning::drbt::HorizonState> q : horizon)
    {
        if (q->getWeight() < DRGBTConfig::TRESHOLD_WEIGHT)
            continue;

        visited = false;
        for (std::shared_ptr<planning::drbt::HorizonState> q_visited : visited_states)
        {
            if (q == q_visited)
            {
                visited = true;
                break;
            }
        }

        if (!visited && q->getWeight() > weight_max)
        {
            q_new = q;
            weight_max = q->getWeight();
        }
    }

    if (q_new != nullptr)
    {
        visited_states.emplace_back(q_new);
        q_next = q_new;
        return true;
    }

    return false;
}
