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

    traj = nullptr;
    guaranteed_safe_motion = false;
    non_zero_final_vel = true;
    max_remaining_iter_time = 0;
    time_iter_start = std::chrono::steady_clock::now();
    waiting_time = 0;
    q_next = nullptr;
    q_next_reached = nullptr;
    drgbt_instance = nullptr;
}

bool planning::trajectory::UpdatingState::update(std::shared_ptr<base::State> &q_previous, std::shared_ptr<base::State> &q_current, 
    const std::shared_ptr<base::State> q_next_, base::State::Status &status)
{
    switch (traj_interpolation)
    {
    case planning::TrajectoryInterpolation::None:
        return update_v1(q_previous, q_current, q_next_, q_next_, status);
    
    default:
        return update_v2(q_previous, q_current, q_next_, q_next_, status);
    }
}

bool planning::trajectory::UpdatingState::update(std::shared_ptr<base::State> &q_previous, std::shared_ptr<base::State> &q_current, 
    const std::shared_ptr<base::State> q_next_, const std::shared_ptr<base::State> q_next_reached_, base::State::Status &status)
{
    switch (traj_interpolation)
    {
    case planning::TrajectoryInterpolation::None:
        return update_v1(q_previous, q_current, q_next_, q_next_reached_, status);
    
    default:
        return update_v2(q_previous, q_current, q_next_, q_next_reached_, status);
    }
}

/// @brief Update a current state 'q_current' to a new desired state 'q_current_new' from the edge [q_current - q_next_reached], 
/// such that it can be reached within max_iter_time time, while considering robot maximal velocity.
/// In other words, 'q_current_new' is determined using an advancing step size which depends on robot's maximal velocity.
/// Move 'q_current' to 'q_current_new' meaning that 'q_current' will be updated to a robot position from the end of current iteration.
/// @return Whether a new current state is computed.
/// @note If 'q_next_reached' is not relevant in the algorithm, pass 'q_next' instead of it.
bool planning::trajectory::UpdatingState::update_v1(std::shared_ptr<base::State> &q_previous, std::shared_ptr<base::State> &q_current, 
    const std::shared_ptr<base::State> q_next_, const std::shared_ptr<base::State> q_next_reached_, base::State::Status &status)
{
    q_previous = q_current;
    q_next = q_next_;
    q_next_reached = q_next_reached_;
        
    if (status == base::State::Status::Trapped)     // Current robot position will not be updated! 
    {                                               // We must wait for successful replanning to change 'status' to 'Reached'
        // std::cout << "Status: Trapped! \n";
        return false;
    }

    std::shared_ptr<base::State> q_current_new { ss->getNewState(q_next_reached->getCoord()) };
    if (!ss->isEqual(q_current, q_current_new))
    {
        if (all_robot_vel_same)
        {
            float max_delta_q { ss->robot->getMaxVel(0) * max_iter_time };
            if (ss->getNorm(q_current, q_current_new) > max_delta_q)
                q_current_new = ss->pruneEdge2(q_current, q_current_new, max_delta_q);
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

    // std::cout << "q_current: " << q_current << "\n";
    // std::cout << "Status: " << status << "\n";

    return true;
}

/// @brief Update a current state of the robot by computing a trajectory 'traj'.
/// Move 'q_current' towards 'q_next_reached'.
/// 'q_current' will be updated to a robot position from the end of current iteration.
/// @return True if a new trajectory is computed. False if the previous trajectory is retained.
/// @note The new trajectory will be computed in a way that all constraints on robot's maximal velocity, 
/// acceleration and jerk are surely always satisfied.
/// @note If 'q_next_reached' is not relevant in the algorithm, pass 'q_next' instead of it.
bool planning::trajectory::UpdatingState::update_v2(std::shared_ptr<base::State> &q_previous, std::shared_ptr<base::State> &q_current, 
    const std::shared_ptr<base::State> q_next_, const std::shared_ptr<base::State> q_next_reached_, base::State::Status &status)
{
    q_previous = q_current;
    q_next = q_next_;
    q_next_reached = q_next_reached_;

    float t_traj_max    // Maximal available time for computing a new trajectory
    { 
        guaranteed_safe_motion ? 
            TrajectoryConfig::MAX_TIME_COMPUTE_SAFE : 
            TrajectoryConfig::MAX_TIME_COMPUTE_REGULAR
    };

    float t_iter { getElapsedTime() };      // Elapsed time in [s] from the beginning of the current iteration
    if (max_iter_time - max_remaining_iter_time - t_iter < t_traj_max)
        t_traj_max = max_iter_time - max_remaining_iter_time - t_iter;
    
    float t_iter_remain { max_iter_time - t_iter - t_traj_max };    // Remaining time in the current iteration

    traj->setTimeBegin(traj->getTimeEnd());
    traj->setTimeCurrent(traj->getTimeBegin() + t_iter + t_traj_max);

    // std::cout << "Max. trajectory time:  " << t_traj_max * 1000 << " [ms] \n";
    // std::cout << "Iter. time:            " << t_iter * 1000 << " [ms] \n";
    // std::cout << "Remain. iter. time:    " << t_iter_remain * 1000 << " [ms] \n";
    // std::cout << "Begin trajectory time: " << traj->getTimeBegin() * 1000 << " [ms] \n";
    // std::cout << "Curr. trajectory time: " << traj->getTimeCurrent() * 1000 << " [ms] \n";
    
    // ----------------------------------------------------------------------------------------- //
    // Store trajectory points from the current iteration to be validated later within 'MotionValidity'
    traj->clearTrajPointCurrentIter();
    size_t num_checks1 = std::floor((traj->getTimeCurrent() - traj->getTimeBegin()) / 
                                    max_iter_time * TrajectoryConfig::NUM_VALIDITY_POINTS_CHECK );
    float delta_time1 { (traj->getTimeCurrent() - traj->getTimeBegin()) / num_checks1 };
    float t { 0 };
    for (size_t num_check = 1; num_check <= num_checks1; num_check++)
    {
        t = traj->getTimeBegin() + num_check * delta_time1;
        traj->addTrajPointCurrentIter(traj->getPosition(t));
        // std::cout << "t: " << t * 1000 << " [ms]\t"
        //           << "pos: " << traj->getPosition(t).transpose() << "\t" 
        //           << "vel: " << traj->getVelocity(t).transpose() << "\t"
        //           << "acc: " << traj->getAcceleration(t).transpose() << "\n";
    }
    // ----------------------------------------------------------------------------------------- //
    
    bool traj_computed { false };
    float t_traj_remain {};
    planning::trajectory::State current
    (
        traj->getPosition(traj->getTimeCurrent()),
        traj->getVelocity(traj->getTimeCurrent()),
        traj->getAcceleration(traj->getTimeCurrent())
    );
    planning::trajectory::State target(ss->num_dimensions);
    // std::cout << "Curr. pos: " << current.pos.transpose() << "\n";
    // std::cout << "Curr. vel: " << current.vel.transpose() << "\n";
    // std::cout << "Curr. acc: " << current.acc.transpose() << "\n";

    do
    {
        t_traj_remain = t_traj_max - (getElapsedTime() - t_iter);
        // std::cout << "t_traj_remain: " << t_traj_remain * 1e3 << " [ms] \n";
        if (t_traj_remain < 0)
            break;

        if (TrajectoryConfig::SCALE_TARGET)
        {
            float step = std::max(ss->robot->getMaxVel().norm() * max_iter_time,
                                  current.vel.norm() / ss->robot->getMaxVel().norm() * TrajectoryConfig::MAX_RADIUS);
            target.pos = (std::get<1>(ss->interpolateEdge2(q_current, q_next_reached, step)))->getCoord();
        }
        else
        target.pos = q_next_reached->getCoord();

        // std::cout << "target pos:     " << target.pos.transpose() << "\n";
        // std::cout << "q_next:         " << q_next << "\n";
        // std::cout << "q_next_reached: " << q_next_reached << "\n";

        if (guaranteed_safe_motion)
            traj_computed = traj->computeSafe(current, target, t_iter_remain, t_traj_remain, non_zero_final_vel, q_current);
        else
        {
            if (traj->isFinalConf(target.pos))  // Spline to such 'target.pos' already exists!
                break;
            traj_computed = traj->computeRegular(current, target, t_iter_remain, t_traj_remain, non_zero_final_vel);
        }
    }
    while (!traj_computed && invokeChangeNextState());

    traj->setTimeEnd(!traj_computed * traj->getTimeCurrent() + t_iter_remain);
    // std::cout << "New trajectory is " << (traj_computed ? "computed!\n" : "NOT computed! Continuing with the previous trajectory!\n");
    // traj->recordTrajectory(traj_computed, t_iter + t_traj_max);   // Only for debugging

    q_current = ss->getNewState(traj->getPosition(traj->getTimeEnd()));   // Current robot position at the end of iteration
    
    if (status != base::State::Status::Trapped)
    {
        if (ss->getNorm(q_current, q_next) <=        // 'q_next' must be reached within the computed radius, and not only 'q_next_reached'
            traj->getVelocity(traj->getTimeEnd()).norm() / ss->robot->getMaxVel().norm() * TrajectoryConfig::MAX_RADIUS)
            status = base::State::Status::Reached;
        else
            status = base::State::Status::Advanced;
    }

    // std::cout << "Elapsed time for trajectory computing: " << (getElapsedTime() - t_iter) * 1e6 << " [us] \n";
    // std::cout << "q_current: " << q_current << "\n";
    // std::cout << "Status: " << status << "\n";
    // std::cout << "Curr. trajectory time: " << traj->getTimeCurrent() * 1000 << " [ms] \n";
    // std::cout << "End trajectory time:   " << traj->getTimeEnd() * 1000 << " [ms] \n";
    
    // ----------------------------------------------------------------------------------------- //
    // Store trajectory points from the current iteration to be validated later within 'MotionValidity'
    size_t num_checks2 { TrajectoryConfig::NUM_VALIDITY_POINTS_CHECK - num_checks1 };
    float delta_time2 { (traj->getTimeEnd() - traj->getTimeCurrent()) / num_checks2 };
    for (size_t num_check = 1; num_check <= num_checks2; num_check++)
    {
        t = traj->getTimeCurrent() + num_check * delta_time2;
        traj->addTrajPointCurrentIter(traj->getPosition(t));
        // std::cout << "t: " << t * 1000 << " [ms]\t"
        //           << "pos: " << traj->getPosition(t).transpose() << "\t" 
        //           << "vel: " << traj->getVelocity(t).transpose() << "\t"
        //           << "acc: " << traj->getAcceleration(t).transpose() << "\n";
    }
    // ----------------------------------------------------------------------------------------- //

    waiting_time = t_traj_max - (getElapsedTime() - t_iter);
    return traj_computed;
}

// This function will change 'q_next' and 'q_next_reached'
bool planning::trajectory::UpdatingState::invokeChangeNextState() 
{
    if (drgbt_instance != nullptr) 
        return drgbt_instance->changeNextState();
    
    return false;
}

// Get elapsed time in [s] from the beginning of the current iteration.
float planning::trajectory::UpdatingState::getElapsedTime()
{
    float t = std::chrono::duration_cast<std::chrono::nanoseconds>
              (std::chrono::steady_clock::now() - time_iter_start).count() * 1e-9;
    return t;
}
