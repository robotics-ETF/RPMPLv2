#include "DRGBT.h"

/// @brief Update a current state of the robot using 'spline_current'.
/// Compute a new spline 'spline_next', or remain 'spline_current'.
/// Determine a new target state 'q_target' (a new desired current state) of the robot 
/// by moving from 'q_current' towards 'q_next' while following 'spline_next'.
/// @param measure_time If true, elapsed time when computing a spline will be exactly measured. 
/// If false, elapsed time will be computed (default: false).
/// @return Remaining time in [s] after which the new spline 'spline_next' will become active.
/// @note The new spline will be computed in a way that all constraints on robot's maximal velocity, 
/// acceleration and jerk are surely always satisfied.
/// @note 'measure_time' should always be false when simulation pacing is used, since then a time measuring will not be correct! 
/// In such case, it is assumed that user was previously set 'measure_time' to a correct value.
float planning::drbt::DRGBT::updateCurrentState(bool measure_time)
{
    spline_current = spline_next;

    const float t_publish_max { 1e-3 };     // 1 [ms] is reserved for publishing trajectory
    float t_spline_max { DRGBTConfig::MAX_TIME_UPDATE_CURRENT_STATE };
    float t_iter { getElapsedTime(time_iter_start) };
    if (DRGBTConfig::MAX_TIME_TASK1 - t_iter < t_spline_max)
        t_spline_max = DRGBTConfig::MAX_TIME_TASK1 - t_iter;
    
    float t_iter_remain { DRGBTConfig::MAX_ITER_TIME - t_iter - t_spline_max };
    float t_spline_current { measure_time ? 
                             spline_next->getTimeCurrent(true) + t_spline_max :
                             spline_next->getTimeEnd() + t_iter + t_spline_max };

    spline_current->setTimeBegin(spline_next->getTimeEnd());
    spline_current->setTimeCurrent(t_spline_current);

    q_current = ss->getNewState(spline_current->getPosition(t_spline_current));

    // std::cout << "Iter. time:        " << t_iter * 1000 << " [ms] \n";
    // std::cout << "Max. spline time:  " << t_spline_max * 1000 << " [ms] \n";
    // std::cout << "Remain. time:      " << t_iter_remain * 1000 << " [ms] \n";
    // std::cout << "Begin spline time: " << spline_current->getTimeBegin() * 1000 << " [ms] \n";
    // std::cout << "Curr. spline time: " << t_spline_current * 1000 << " [ms] \n";
    // std::cout << "q_current: " << q_current << "\n";
    // std::cout << "q_target:  " << q_target << "\n";
    // std::cout << "q_next:    " << q_next << "\n";

    if (ss->isEqual(q_current, q_goal))
    {
        spline_next->setTimeEnd(t_spline_current + t_iter_remain);
        status = base::State::Status::Reached;
        return t_spline_max;
    }

    bool found { false };
    if (spline_current->isFinalConf(q_next->getStateReached()->getCoord()))     // Coordinates of 'q_next->getStateReached()' did not change!
    {
        std::cout << "Not computing a new spline! \n";
        found = false;
    }
    else
    {
        std::vector<std::shared_ptr<planning::drbt::HorizonState>> visited_states { q_next };
        spline_next = std::make_shared<planning::trajectory::Spline5>
        (
            ss->robot, 
            q_current->getCoord(),
            spline_current->getVelocity(t_spline_current),
            spline_current->getAcceleration(t_spline_current)
        );
        std::cout << "curr pos: " << q_current->getCoord().transpose() << "\n";
        std::cout << "curr vel: " << spline_current->getVelocity(t_spline_current).transpose() << "\n";
        std::cout << "curr acc: " << spline_current->getAcceleration(t_spline_current).transpose() << "\n";

        do
        {
            std::cout << "q_final: " << q_next->getStateReached()->getCoord().transpose() << "\t idx: " << q_next->getIndex() << "\n";
            
            if (spline_current->isFinalConf(q_next->getStateReached()->getCoord()))     // Spline to such 'q_next->getStateReached()' already exists!
                break;

            if (q_next->getIsReached() && 
                q_next->getIndex() != -1 && 
                q_next->getStatus() != planning::drbt::HorizonState::Status::Goal)
            {
                // Eigen::VectorXf q_final_dot { spline_current->getVelocity(t_spline_current) };  // Try to hold an initial velocity
                float delta_t_max { ((q_next->getStateReached()->getCoord() - q_current->getCoord()).cwiseQuotient(ss->robot->getMaxVel())).cwiseAbs().maxCoeff() };
                Eigen::VectorXf q_final_dot { (q_next->getStateReached()->getCoord() - q_current->getCoord()) / delta_t_max };
                size_t num_iter { 0 }, max_num_iter { 5 };

                do
                {
                    std::cout << "num_iter: " << num_iter << "\t q_final_dot: " << q_final_dot.transpose() << "\n";

                    found = spline_next->compute(q_next->getStateReached()->getCoord(), q_final_dot);
                    if (found) std::cout << "\t Spline computed with NON-ZERO final velocity. \n";

                    // Reducing the final velocity
                    if (++num_iter < max_num_iter - 1)
                        q_final_dot *= 0.5;
                    else
                        q_final_dot = Eigen::VectorXf::Zero(ss->num_dimensions);
                }
                while (!found && 
                       num_iter < max_num_iter && 
                       getElapsedTime(time_iter_start) - t_iter < t_spline_max - t_publish_max * measure_time);
            }
            else
            {
                found = spline_next->compute(q_next->getStateReached()->getCoord());
                if (found) std::cout << "\t Spline computed with ZERO final velocity. \n";
            }
        }
        while (!found && 
               changeNextState(visited_states) && 
               getElapsedTime(time_iter_start) - t_iter < t_spline_max - t_publish_max * measure_time);
        // std::cout << "Elapsed time for spline computing: " << (getElapsedTime(time_iter_start) - t_iter) * 1e6 << " [us] \n";
    }

    if (found)
    {
        std::cout << "New spline is computed! \n";
        spline_current->setTimeEnd(t_spline_current);
        spline_next->setTimeEnd(t_iter_remain);
    }
    else
    {
        std::cout << "Continuing with the previous spline! \n";
        spline_next = spline_current;
        spline_next->setTimeEnd(t_spline_current + t_iter_remain);
    }

    // TODO: New spline needs to be validated on collision, at least during the current iteration!
    
    q_target = ss->getNewState(spline_next->getPosition(spline_next->getTimeEnd() + DRGBTConfig::MAX_TIME_TASK1));
    q_target->setParent(q_current);
    
    if (status != base::State::Status::Trapped)
    {
        if (spline_next->getTimeFinal() < spline_next->getTimeEnd() + DRGBTConfig::MAX_TIME_TASK1 + DRGBTConfig::MAX_ITER_TIME)
            status = base::State::Status::Reached;      // 'q_next' must be reached, and not only 'q_next->getStateReached()'
        else
            status = base::State::Status::Advanced;
    }

    // std::cout << "q_target:      " << q_target << "\n";
    // std::cout << "q_target time: " << t_target * 1000 << " [ms] \n";
    // std::cout << "Spline next: \n" << spline_next << "\n";
    std::cout << "Status: " << (status == base::State::Status::Advanced ? "Advanced" : "")
                            << (status == base::State::Status::Trapped  ? "Trapped"  : "")
                            << (status == base::State::Status::Reached  ? "Reached"  : "") << "\n";

    return t_spline_max - (getElapsedTime(time_iter_start) - t_iter);
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

/// @brief Update a current state 'q_current' to become 'q_target'.
/// Determine a new target state 'q_target' (a new desired current state) of the robot 
/// by moving from 'q_current' towards 'q_next' for an advancing step size determined as 
/// ss->robot->getMaxVel(i) * DRGBTConfig::MAX_ITER_TIME, i.d., using the maximal robot's velocity.
void planning::drbt::DRGBT::updateCurrentState()
{
    q_previous = q_current;
    if (status == base::State::Status::Trapped)
    {
        // std::cout << "Status: Trapped! \n";
        return;
    }

    q_current = q_target;   // Current position at the end of iteration
    if (ss->isEqual(q_current, q_goal))
    {
        status = base::State::Status::Reached;
        return;
    }

    q_target = ss->getNewState(q_next->getStateReached()->getCoord());

    // Option 1: If all velocities are the same, the following can be used:
    // float max_edge_length_ = ss->robot->getMaxVel(0) * DRGBTConfig::MAX_ITER_TIME;
    // if (ss->getNorm(q_current, q_target) > max_edge_length_)    // Check whether 'q_target' can be reached considering robot max. velocity
    //     q_target = ss->pruneEdge2(q_current, q_target, max_edge_length_);

    // Option 2: If all velocities are not the same, the following can be used:
    std::vector<std::pair<float, float>> limits;
    for (size_t i = 0; i < ss->num_dimensions; i++)
    {
        limits.emplace_back(std::pair<float, float>
            (q_current->getCoord(i) - ss->robot->getMaxVel(i) * DRGBTConfig::MAX_ITER_TIME, 
            q_current->getCoord(i) + ss->robot->getMaxVel(i) * DRGBTConfig::MAX_ITER_TIME));
    }
    q_target = ss->pruneEdge(q_current, q_target, limits);  // Check whether 'q_target' can be reached considering robot max. velocity
    q_target->setParent(q_current);

    if (ss->isEqual(q_target, q_next->getState()))
        status = base::State::Status::Reached;      // 'q_next' must be reached, and not only 'q_next->getStateReached()'
    else
        status = base::State::Status::Advanced;

    // std::cout << "q_current: " << q_current << "\n";
    // std::cout << "q_target:  " << q_target << "\n";
    // std::cout << "Status: " << (status == base::State::Status::Advanced ? "Advanced" : "")
    //                         << (status == base::State::Status::Reached  ? "Reached"  : "") << "\n";
}
