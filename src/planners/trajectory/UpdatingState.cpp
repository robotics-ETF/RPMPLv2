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
    traj_ruckig = nullptr;
    guaranteed_safe_motion = false;
    non_zero_final_vel = true;
    max_remaining_iter_time = 0;
    time_iter_start = std::chrono::steady_clock::now();
    measure_time = false;
    remaining_time = 0;
    q_next = nullptr;
    q_next_reached = nullptr;
    drgbt_instance = nullptr;
}

void planning::trajectory::UpdatingState::update(std::shared_ptr<base::State> &q_previous, std::shared_ptr<base::State> &q_current, 
    const std::shared_ptr<base::State> q_next_, base::State::Status &status)
{
    switch (traj_interpolation)
    {
    case planning::TrajectoryInterpolation::None:
        update_v1(q_previous, q_current, q_next_, q_next_, status);
        break;
    
    case planning::TrajectoryInterpolation::Spline:
        update_v2(q_previous, q_current, q_next_, q_next_, status);
        break;

    case planning::TrajectoryInterpolation::Ruckig:
        update_v3(q_previous, q_current, q_next_, q_next_, status);
        break;
    }
}

void planning::trajectory::UpdatingState::update(std::shared_ptr<base::State> &q_previous, std::shared_ptr<base::State> &q_current, 
    const std::shared_ptr<base::State> q_next_, const std::shared_ptr<base::State> q_next_reached_, base::State::Status &status)
{
    switch (traj_interpolation)
    {
    case planning::TrajectoryInterpolation::None:
        update_v1(q_previous, q_current, q_next_, q_next_reached_, status);
        break;
    
    case planning::TrajectoryInterpolation::Spline:
        update_v2(q_previous, q_current, q_next_, q_next_reached_, status);
        break;

    case planning::TrajectoryInterpolation::Ruckig:
        update_v3(q_previous, q_current, q_next_, q_next_reached_, status);
        break;
    }
}

/// @brief Update a current state 'q_current' to a new desired state 'q_current_new' from the edge [q_current - q_next_reached], 
/// such that it can be reached within max_iter_time time, while considering robot maximal velocity.
/// In other words, 'q_current_new' is determined using an advancing step size which depends on robot's maximal velocity.
/// Move 'q_current' to 'q_current_new' meaning that 'q_current' will be updated to a robot position from the end of current iteration.
/// @note If 'q_next_reached' is not relevant in the algorithm, pass 'q_next' instead of it.
void planning::trajectory::UpdatingState::update_v1(std::shared_ptr<base::State> &q_previous, std::shared_ptr<base::State> &q_current, 
    const std::shared_ptr<base::State> q_next_, const std::shared_ptr<base::State> q_next_reached_, base::State::Status &status)
{
    q_previous = q_current;
    q_next = q_next_;
    q_next_reached = q_next_reached_;
        
    if (status == base::State::Status::Trapped)     // Current robot position will not be updated! 
    {                                               // We must wait for successful replanning to change 'status' to 'Reached'
        // std::cout << "Status: Trapped! \n";
        return;
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
    // std::cout << "Status: " << (status == base::State::Status::Advanced ? "Advanced" : "")
    //                         << (status == base::State::Status::Reached  ? "Reached"  : "") << "\n";
}

/// @brief Update a current state of the robot using 'traj->spline_current'.
/// Compute a new spline 'traj->spline_next', or remain 'traj->spline_current'.
/// Move 'q_current' towards 'q_next_reached' while following 'traj->spline_next'.
/// 'q_current' will be updated to a robot position from the end of current iteration.
/// @note The new trajectory will be computed in a way that all constraints on robot's maximal velocity, 
/// acceleration and jerk are surely always satisfied.
/// @note If 'q_next_reached' is not relevant in the algorithm, pass 'q_next' instead of it.
void planning::trajectory::UpdatingState::update_v2(std::shared_ptr<base::State> &q_previous, std::shared_ptr<base::State> &q_current, 
    const std::shared_ptr<base::State> q_next_, const std::shared_ptr<base::State> q_next_reached_, base::State::Status &status)
{
    q_previous = q_current;
    q_next = q_next_;
    q_next_reached = q_next_reached_;
    traj->spline_current = traj->spline_next;

    float t_traj_max { (guaranteed_safe_motion ? TrajectoryConfig::MAX_TIME_COMPUTE_SAFE : TrajectoryConfig::MAX_TIME_COMPUTE_REGULAR) - 
                        TrajectoryConfig::MAX_TIME_PUBLISH * measure_time };
    float t_iter { getElapsedTime() };
    if (max_iter_time - max_remaining_iter_time - t_iter < t_traj_max)
        t_traj_max = max_iter_time - max_remaining_iter_time - t_iter;
    
    float t_iter_remain { max_iter_time - t_iter - t_traj_max };
    float t_traj_current { measure_time ? 
                           traj->spline_current->getTimeCurrent(true) + t_traj_max :
                           traj->spline_current->getTimeEnd() + t_iter + t_traj_max };

    traj->spline_current->setTimeBegin(traj->spline_current->getTimeEnd());
    traj->spline_current->setTimeCurrent(t_traj_current);

    // std::cout << "Iter. time:            " << t_iter * 1000 << " [ms] \n";
    // std::cout << "Max. trajectory time:  " << t_traj_max * 1000 << " [ms] \n";
    // std::cout << "Remain. time:          " << t_iter_remain * 1000 << " [ms] \n";
    // std::cout << "Begin trajectory time: " << traj->spline_current->getTimeBegin() * 1000 << " [ms] \n";
    // std::cout << "Curr. trajectory time: " << t_traj_current * 1000 << " [ms] \n";
    // ----------------------------------------------------------------------------------------- //
    
    bool traj_computed { false };
    float t_traj_remain {};
    Eigen::VectorXf current_pos { traj->spline_current->getPosition(t_traj_current) };
    Eigen::VectorXf current_vel { traj->spline_current->getVelocity(t_traj_current) };
    Eigen::VectorXf current_acc { traj->spline_current->getAcceleration(t_traj_current) };
    // std::cout << "Curr. pos: " << current_pos.transpose() << "\n";
    // std::cout << "Curr. vel: " << current_vel.transpose() << "\n";
    // std::cout << "Curr. acc: " << current_acc.transpose() << "\n";

    do
    {
        t_traj_remain = t_traj_max - (getElapsedTime() - t_iter);
        // std::cout << "t_traj_remain: " << t_traj_remain * 1e3 << " [ms] \n";
        if (t_traj_remain < 0)
            break;

        float step { std::max(ss->robot->getMaxVel().norm() * max_iter_time,
                              current_vel.norm() / ss->robot->getMaxVel().norm() * TrajectoryConfig::MAX_RADIUS) };
        std::shared_ptr<base::State> q_target { std::get<1>(ss->interpolateEdge2(q_current, q_next_reached, step)) };

        traj->setCurrentState(q_current);
        traj->setTargetState(q_target);
        // std::cout << "q_target:       " << q_target << "\n";
        // std::cout << "q_next:         " << q_next << "\n";
        // std::cout << "q_next_reached: " << q_next_reached << "\n";

        if (guaranteed_safe_motion)
            traj_computed = traj->computeSafe(current_pos, current_vel, current_acc, t_iter_remain, t_traj_remain);
        else
        {
            if (traj->spline_current->isFinalConf(q_target->getCoord()))  // Spline to such 'q_target' already exists!
                break;
            traj_computed = traj->computeRegular(current_pos, current_vel, current_acc, t_iter_remain, t_traj_remain, non_zero_final_vel);
        }
    }
    while (!traj_computed && invokeChangeNextState());

    if (traj_computed)
    {
        // std::cout << "New trajectory is computed! \n";
        traj->spline_current->setTimeEnd(t_traj_current);
        traj->spline_next->setTimeEnd(t_iter_remain);
    }
    else
    {
        // std::cout << "Continuing with the previous trajectory! \n";
        traj->spline_next = traj->spline_current;
        traj->spline_next->setTimeEnd(t_traj_current + t_iter_remain);
    }
    
    // traj->recordTrajectory(traj_computed);   // Only for debugging

    q_current = ss->getNewState(traj->spline_next->getPosition(traj->spline_next->getTimeEnd()));   // Current robot position at the end of iteration
    if (status != base::State::Status::Trapped)
    {
        if (ss->getNorm(q_current, q_next) <=        // 'q_next' must be reached within the computed radius, and not only 'q_next_reached'
            traj->spline_next->getVelocity(traj->spline_next->getTimeEnd()).norm() / ss->robot->getMaxVel().norm() * TrajectoryConfig::MAX_RADIUS)
            status = base::State::Status::Reached;
        else
            status = base::State::Status::Advanced;
    }

    // std::cout << "Elapsed time for trajectory computing: " << (getElapsedTime() - t_iter) * 1e3 << " [ms] \n";
    // std::cout << "Spline next: \n" << traj->spline_next << "\n";
    // std::cout << "q_current: " << q_current << "\n";
    // std::cout << "Status: " << (status == base::State::Status::Advanced ? "Advanced" : "")
    //                         << (status == base::State::Status::Trapped  ? "Trapped"  : "")
    //                         << (status == base::State::Status::Reached  ? "Reached"  : "") << "\n";
    // std::cout << "Current spline times:   " << traj->spline_current->getTimeBegin() * 1000 << " [ms] \t"
    //                                         << traj->spline_current->getTimeCurrent() * 1000 << " [ms] \t"
    //                                         << traj->spline_current->getTimeEnd() * 1000 << " [ms] \n";
    // std::cout << "Next spline times:      " << traj->spline_next->getTimeBegin() * 1000 << " [ms] \t"
    //                                         << traj->spline_next->getTimeCurrent() * 1000 << " [ms] \t"
    //                                         << traj->spline_next->getTimeEnd() * 1000 << " [ms] \n";

    // ----------------------------------------------------------------------------------------- //
    // Store trajectory points from the current iteration to be validated later within 'MotionValidity'
    traj->clearTrajPointCurrentIter();
    size_t num_checks1 = std::floor((traj->spline_current->getTimeCurrent() - traj->spline_current->getTimeBegin()) / 
                                    max_iter_time * TrajectoryConfig::NUM_VALIDITY_POINTS_CHECK );
    float delta_time1 { (traj->spline_current->getTimeCurrent() - traj->spline_current->getTimeBegin()) / num_checks1 };
    float t { 0 };
    for (size_t num_check = 1; num_check <= num_checks1; num_check++)
    {
        t = traj->spline_current->getTimeBegin() + num_check * delta_time1;
        traj->addTrajPointCurrentIter(traj->spline_current->getPosition(t));
        // std::cout << "t: " << t * 1000 << " [ms]\t"
        //           << "pos: " << traj->spline_current->getPosition(t).transpose() << "\t" 
        //           << "vel: " << traj->spline_current->getVelocity(t).transpose() << "\t"
        //           << "acc: " << traj->spline_current->getAcceleration(t).transpose() << "\n";
    }
    
    size_t num_checks2 { TrajectoryConfig::NUM_VALIDITY_POINTS_CHECK - num_checks1 };
    float delta_time2 { (traj->spline_next->getTimeEnd() - traj->spline_next->getTimeCurrent()) / num_checks2 };
    for (size_t num_check = 1; num_check <= num_checks2; num_check++)
    {
        t = traj->spline_next->getTimeCurrent() + num_check * delta_time2;
        traj->addTrajPointCurrentIter(traj->spline_next->getPosition(t));
        // std::cout << "t: " << t * 1000 << " [ms]\t"
        //           << "pos: " << traj->spline_next->getPosition(t).transpose() << "\t" 
        //           << "vel: " << traj->spline_next->getVelocity(t).transpose() << "\t"
        //           << "acc: " << traj->spline_next->getAcceleration(t).transpose() << "\n";
    }
    // ----------------------------------------------------------------------------------------- //

    remaining_time = t_traj_max + TrajectoryConfig::MAX_TIME_PUBLISH * measure_time - (getElapsedTime() - t_iter);
}

/// @brief Update a current state of the robot using Ruckig library.
/// Move 'q_current' towards 'q_next_reached'.
/// 'q_current' will be updated to a robot position from the end of current iteration.
/// @note All constraints on robot's maximal velocity and acceleration are surely always satisfied.
/// @note If 'q_next_reached' is not relevant in the algorithm, pass 'q_next' instead of it.
void planning::trajectory::UpdatingState::update_v3(std::shared_ptr<base::State> &q_previous, std::shared_ptr<base::State> &q_current, 
    const std::shared_ptr<base::State> q_next_, const std::shared_ptr<base::State> q_next_reached_, base::State::Status &status)
{
    q_previous = q_current;
    q_next = q_next_;
    q_next_reached = q_next_reached_;

    float t_traj_max { (guaranteed_safe_motion ? TrajectoryConfig::MAX_TIME_COMPUTE_SAFE : TrajectoryConfig::MAX_TIME_COMPUTE_REGULAR) - 
                        TrajectoryConfig::MAX_TIME_PUBLISH * measure_time };
    float t_iter { getElapsedTime() };
    if (max_iter_time - max_remaining_iter_time - t_iter < t_traj_max)
        t_traj_max = max_iter_time - max_remaining_iter_time - t_iter;
    
    float t_iter_remain { max_iter_time - t_iter - t_traj_max };
    float t_traj_current { measure_time ? 
                           traj_ruckig->getTimeCurrent(true) + t_traj_max :
                           traj_ruckig->getTimeEnd() + t_iter + t_traj_max };

    traj_ruckig->setTimeBegin(traj_ruckig->getTimeEnd());
    traj_ruckig->setTimeCurrent(t_traj_current);

    // std::cout << "Iter. time:            " << t_iter * 1000 << " [ms] \n";
    // std::cout << "Max. trajectory time:  " << t_traj_max * 1000 << " [ms] \n";
    // std::cout << "Remain. time:          " << t_iter_remain * 1000 << " [ms] \n";
    // std::cout << "Curr. trajectory time: " << t_traj_current * 1000 << " [ms] \n";
    
    // ----------------------------------------------------------------------------------------- //
    // Store trajectory points from the current iteration to be validated later within 'MotionValidity'
    traj_ruckig->clearTrajPointCurrentIter();
    size_t num_checks1 = std::floor((traj_ruckig->getTimeCurrent() - traj_ruckig->getTimeBegin()) / 
                                    max_iter_time * TrajectoryConfig::NUM_VALIDITY_POINTS_CHECK );
    float delta_time1 { (traj_ruckig->getTimeCurrent() - traj_ruckig->getTimeBegin()) / num_checks1 };
    float t { 0 };
    for (size_t num_check = 1; num_check <= num_checks1; num_check++)
    {
        t = traj_ruckig->getTimeBegin() + num_check * delta_time1;
        traj_ruckig->addTrajPointCurrentIter(traj_ruckig->getPosition(t));
        std::cout << "t: " << t * 1000 << " [ms]\t"
                  << "pos: " << traj_ruckig->getPosition(t).transpose() << "\t" 
                  << "vel: " << traj_ruckig->getVelocity(t).transpose() << "\t"
                  << "acc: " << traj_ruckig->getAcceleration(t).transpose() << "\n";
    }
    // ----------------------------------------------------------------------------------------- //

    bool traj_computed { false };
    float t_traj_remain {};
    Eigen::VectorXf current_pos { traj_ruckig->getPosition(t_traj_current) };
    Eigen::VectorXf current_vel { traj_ruckig->getVelocity(t_traj_current) };
    Eigen::VectorXf current_acc { traj_ruckig->getAcceleration(t_traj_current) };
    // std::cout << "Curr. pos: " << current_pos.transpose() << "\n";
    // std::cout << "Curr. vel: " << current_vel.transpose() << "\n";
    // std::cout << "Curr. acc: " << current_acc.transpose() << "\n";

    do
    {
        t_traj_remain = t_traj_max - (getElapsedTime() - t_iter);
        // std::cout << "t_traj_remain: " << t_traj_remain * 1e3 << " [ms] \n";
        if (t_traj_remain < 0)
            break;

        float step { std::max(ss->robot->getMaxVel().norm() * max_iter_time,
                              current_vel.norm() / ss->robot->getMaxVel().norm() * TrajectoryConfig::MAX_RADIUS) };
        std::shared_ptr<base::State> q_target { std::get<1>(ss->interpolateEdge2(q_current, q_next_reached, step)) };

        traj_ruckig->setCurrentState(q_current->getCoord());
        traj_ruckig->setTargetState(q_target->getCoord());
        // std::cout << "q_target:       " << q_target << "\n";
        // std::cout << "q_next:         " << q_next << "\n";
        // std::cout << "q_next_reached: " << q_next_reached << "\n";

        if (traj_ruckig->isFinalConf(q_target->getCoord()))  // Trajectory to such 'q_target' already exists!
            break;
        traj_computed = traj_ruckig->computeRegular(current_pos, current_vel, current_acc, t_iter_remain, t_traj_remain, non_zero_final_vel);
    }
    while (!traj_computed && invokeChangeNextState());

    if (traj_computed)
    {
        // std::cout << "New trajectory is computed! \n";
        traj_ruckig->setTimeEnd(t_iter_remain);
    }
    else
    {
        // std::cout << "Continuing with the previous trajectory! \n";
        traj_ruckig->setTimeEnd(t_traj_current + t_iter_remain);
    }

    q_current = ss->getNewState(traj_ruckig->getPosition(traj_ruckig->getTimeEnd()));   // Current robot position at the end of iteration
    if (status != base::State::Status::Trapped)
    {
        if (ss->getNorm(q_current, q_next) <=        // 'q_next' must be reached within the computed radius, and not only 'q_next_reached'
            traj_ruckig->getVelocity(traj_ruckig->getTimeEnd()).norm() / ss->robot->getMaxVel().norm() * TrajectoryConfig::MAX_RADIUS)
            status = base::State::Status::Reached;
        else
            status = base::State::Status::Advanced;
    }

    std::cout << "Elapsed time for trajectory computing: " << (getElapsedTime() - t_iter) * 1e3 << " [ms] \n";
    // std::cout << "q_current: " << q_current << "\n";
    // std::cout << "Status: " << (status == base::State::Status::Advanced ? "Advanced" : "")
    //                         << (status == base::State::Status::Trapped  ? "Trapped"  : "")
    //                         << (status == base::State::Status::Reached  ? "Reached"  : "") << "\n";

    // ----------------------------------------------------------------------------------------- //
    // Store trajectory points from the current iteration to be validated later within 'MotionValidity'
    size_t num_checks2 { TrajectoryConfig::NUM_VALIDITY_POINTS_CHECK - num_checks1 };
    float delta_time2 { (traj_ruckig->getTimeEnd() - traj_ruckig->getTimeCurrent()) / num_checks2 };
    for (size_t num_check = 1; num_check <= num_checks2; num_check++)
    {
        t = traj_ruckig->getTimeCurrent() + num_check * delta_time2;
        traj_ruckig->addTrajPointCurrentIter(traj_ruckig->getPosition(t));
        std::cout << "t: " << t * 1000 << " [ms]\t"
                  << "pos: " << traj_ruckig->getPosition(t).transpose() << "\t" 
                  << "vel: " << traj_ruckig->getVelocity(t).transpose() << "\t"
                  << "acc: " << traj_ruckig->getAcceleration(t).transpose() << "\n";
    }
    // ----------------------------------------------------------------------------------------- //

    remaining_time = t_traj_max + TrajectoryConfig::MAX_TIME_PUBLISH * measure_time - (getElapsedTime() - t_iter);
}

// This function will change 'q_next' and 'q_next_reached'
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
