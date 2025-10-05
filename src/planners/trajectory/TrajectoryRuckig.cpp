#include "TrajectoryRuckig.h"

planning::trajectory::TrajectoryRuckig::TrajectoryRuckig(const std::shared_ptr<base::StateSpace> &ss_, 
                                                         const Eigen::VectorXf &q_current_, float max_iter_time_) : 
    input(ss_->num_dimensions), 
    traj(ss_->num_dimensions), 
    traj_temp(ss_->num_dimensions)
{
    ss = ss_;
    q_current = q_current_;
    max_iter_time = max_iter_time_;
    time_current = 0;
    time_final = 0;
    time_final_temp = 0;
    time_begin = 0;
    time_end = 0;
    time_start = std::chrono::steady_clock::now();
    time_start_offset = 0;
    is_zero_final_vel = true;
    traj_points_current_iter = {};

    for (size_t i = 0; i < ss->num_dimensions; i++)
    {
        input.current_position[i] = q_current(i);
        input.current_velocity[i] = 0;
        input.current_acceleration[i] = 0;

        input.max_velocity[i] = ss->robot->getMaxVel(i);
        input.max_acceleration[i] = ss->robot->getMaxAcc(i);
        input.max_jerk[i] = ss->robot->getMaxJerk(i);
    }
    input.synchronization = ruckig::Synchronization::Time;

    if (!input.validate())
        throw std::runtime_error("Invalid input parameters for Ruckig");

    all_robot_vel_same = true;
    for (size_t i = 1; i < ss->num_dimensions; i++)
    {
        if (std::abs(ss->robot->getMaxVel(i) - ss->robot->getMaxVel(i-1)) > RealVectorSpaceConfig::EQUALITY_THRESHOLD)
        {
            all_robot_vel_same = false;
            break;
        }
    }

    max_num_iter_trajectory = all_robot_vel_same ? 
        std::ceil(std::log2(2 * ss->robot->getMaxVel(0) / TrajectoryConfig::FINAL_VELOCITY_STEP)) :
        std::ceil(std::log2(2 * ss->robot->getMaxVel().maxCoeff() / TrajectoryConfig::FINAL_VELOCITY_STEP));
}

/// @brief Compute a regular trajectory that is not surely safe for environment, meaning that,
/// if collision eventually occurs, it may be at robot's non-zero velocity.
/// @param current_pos Current robot's position
/// @param current_vel Current robot's velocity
/// @param current_acc Current robot's acceleration
/// @param t_iter_remain Remaining time in [s] in the current iteration
/// @param t_max Maximal available time in [s] for a trajectory computing
/// @param non_zero_final_vel Whether final velocity can be non-zero
/// @return The success of a trajectory computation
bool planning::trajectory::TrajectoryRuckig::computeRegular(const Eigen::VectorXf &current_pos, const Eigen::VectorXf &current_vel, 
    const Eigen::VectorXf &current_acc, float t_iter_remain, float t_max, bool non_zero_final_vel)
{
    std::chrono::steady_clock::time_point time_start_ { std::chrono::steady_clock::now() };
    float t_remain { t_max };
    bool traj_computed { false };
    ruckig::Result result { ruckig::Result::Working };
    ruckig::Ruckig<ruckig::DynamicDOFs> otg(ss->num_dimensions);
    setCurrentState(current_pos, current_vel, current_acc);

    if (non_zero_final_vel)
    {
        is_zero_final_vel = false;
        size_t num_iter { 0 };
        float delta_t_max { ((q_target - q_current).cwiseQuotient(ss->robot->getMaxVel())).cwiseAbs().maxCoeff() };
        Eigen::VectorXf q_final_dot_max { (q_target - q_current) / delta_t_max };
        Eigen::VectorXf q_final_dot_min { Eigen::VectorXf::Zero(ss->num_dimensions) };
        Eigen::VectorXf q_final_dot { q_final_dot_max };
        
        while (!traj_computed && num_iter++ < max_num_iter_trajectory && t_remain > 0)
        {
            setTargetState(q_target, q_final_dot, Eigen::VectorXf::Zero(ss->num_dimensions));
            result = otg.calculate(input, traj_temp);
            
            if (result == ruckig::Result::Working || result == ruckig::Result::Finished)
            {
                traj = traj_temp;
                time_current = 0;
                time_final = traj.get_duration();
                traj_computed = true;
            }
            else
                q_final_dot_max = q_final_dot;

            q_final_dot = (q_final_dot_max + q_final_dot_min) / 2;
            t_remain -= std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start_).count() / 1e6;
            // std::cout << "Num. iter. " << num_iter << "\t q_final_dot: " << q_final_dot.transpose() << "\n";
        }
    }

    // Possible current position at the end of iteration
    Eigen::VectorXf new_current_pos { getPosition(time_current + t_iter_remain) };
    
    // If trajectory was not computed or robot is getting away from 'new_current_pos'
    if ((!traj_computed || (new_current_pos - q_target).norm() > (current_pos - q_target).norm()) && t_remain > 0)
    {
        is_zero_final_vel = true;
        setTargetState(q_target, Eigen::VectorXf::Zero(ss->num_dimensions), Eigen::VectorXf::Zero(ss->num_dimensions));
        result = otg.calculate(input, traj_temp);

        if (result == ruckig::Result::Working || result == ruckig::Result::Finished)
        {
            traj = traj_temp;
            time_current = 0;
            time_final = traj.get_duration();
            traj_computed = true;
        }
    }

    // std::cout << "\t Trajectory " << (!traj_computed ? "NOT " : "") << "computed with " 
    //           << (!is_zero_final_vel ? "NON-" : "") <<  "ZERO final velocity. \n";
    return traj_computed;
}

void planning::trajectory::TrajectoryRuckig::setCurrentState(const Eigen::VectorXf &current_pos, 
    const Eigen::VectorXf &current_vel, const Eigen::VectorXf &current_acc)
{
    for (size_t i = 0; i < ss->num_dimensions; i++)
    {
        input.current_position[i] = current_pos(i);
        input.current_velocity[i] = current_vel(i);
        input.current_acceleration[i] = current_acc(i);
    }
}

void planning::trajectory::TrajectoryRuckig::setTargetState(const Eigen::VectorXf &target_pos, 
    const Eigen::VectorXf &target_vel, const Eigen::VectorXf &target_acc)
{
    for (size_t i = 0; i < ss->num_dimensions; i++)
    {
        input.target_position[i] = target_pos(i);
        input.target_velocity[i] = target_vel(i);
        input.target_acceleration[i] = target_acc(i);
    }
}

Eigen::VectorXf planning::trajectory::TrajectoryRuckig::getPosition(float t)
{
    ruckig::StandardVector<double, ruckig::DynamicDOFs> pos(ss->num_dimensions);

    if (time_final == 0)
        pos = input.current_position;
    else
        traj.at_time(t, pos);

    Eigen::VectorXf ret(ss->num_dimensions);
    for (size_t i = 0; i < ss->num_dimensions; i++)
        ret(i) = pos[i];

    return ret;
}

Eigen::VectorXf planning::trajectory::TrajectoryRuckig::getVelocity(float t)
{
    ruckig::StandardVector<double, ruckig::DynamicDOFs> vel(ss->num_dimensions);
    [[maybe_unused]] ruckig::StandardVector<double, ruckig::DynamicDOFs> pos(ss->num_dimensions);
    [[maybe_unused]] ruckig::StandardVector<double, ruckig::DynamicDOFs> acc(ss->num_dimensions);

    if (time_final == 0)
        vel = input.current_velocity;
    else
        traj.at_time(t, pos, vel, acc);

    Eigen::VectorXf ret(ss->num_dimensions);
    for (size_t i = 0; i < ss->num_dimensions; i++)
        ret(i) = vel[i];

    return ret;
}

Eigen::VectorXf planning::trajectory::TrajectoryRuckig::getAcceleration(float t)
{
    ruckig::StandardVector<double, ruckig::DynamicDOFs> acc(ss->num_dimensions);
    [[maybe_unused]] ruckig::StandardVector<double, ruckig::DynamicDOFs> pos(ss->num_dimensions);
    [[maybe_unused]] ruckig::StandardVector<double, ruckig::DynamicDOFs> vel(ss->num_dimensions);

    if (time_final == 0)
        acc = input.current_acceleration;
    else
        traj.at_time(t, pos, vel, acc);
    
    Eigen::VectorXf ret(ss->num_dimensions);
    for (size_t i = 0; i < ss->num_dimensions; i++)
        ret(i) = acc[i];

    return ret;
}

void planning::trajectory::TrajectoryRuckig::addTrajPointCurrentIter(const Eigen::VectorXf &pos)
{
    traj_points_current_iter.emplace_back(pos);
}

void planning::trajectory::TrajectoryRuckig::clearTrajPointCurrentIter()
{
    traj_points_current_iter.clear();
}

/// @brief Check whether 'q' is a final configuration of the trajectory.
/// @param q Configuration to be checked.
/// @return True if yes, false if not.
bool planning::trajectory::TrajectoryRuckig::isFinalConf(const Eigen::VectorXf &q)
{
    return ((q - getPosition(time_final)).norm() < RealVectorSpaceConfig::EQUALITY_THRESHOLD) ? true : false;
}

/// @brief Get current time of a spline.
/// @param measure_time If true, current time will be automatically computed/measured (default: false).
/// @note 'measure_time' should always be false when simulation pacing is used, since then a time measuring will not be correct! 
/// In such case, it is assumed that user was previously set 'measure_time' to a correct value.
float planning::trajectory::TrajectoryRuckig::getTimeCurrent(bool measure_time)
{
    if (!measure_time)
        return time_current;
    
    time_current = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - time_start).count() * 1e-9 
                   - time_start_offset;
    return time_current;
}

void planning::trajectory::TrajectoryRuckig::setTimeStart(float time_start_offset_)
{
    time_start = std::chrono::steady_clock::now();
    time_start_offset = time_start_offset_;
}
