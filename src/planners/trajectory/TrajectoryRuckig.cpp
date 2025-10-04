#include "TrajectoryRuckig.h"

planning::trajectory::TrajectoryRuckig::TrajectoryRuckig(const std::shared_ptr<base::StateSpace> &ss_, const Eigen::VectorXf &q_current_, float max_iter_time_) : 
    input(ss->num_dimensions), 
    traj(ss->num_dimensions), 
    traj_temp(ss->num_dimensions),
    otg(ss->num_dimensions)
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

    input.current_position = q_current.cast<double>();
    input.current_velocity = Eigen::VectorXd::Zero(ss->num_dimensions);
    input.current_acceleration = Eigen::VectorXd::Zero(ss->num_dimensions);

    input.max_velocity = ss->robot->getMaxVel().cast<double>();
    input.max_acceleration = ss->robot->getMaxAcc().cast<double>();
    input.max_jerk = ss->robot->getMaxJerk().cast<double>();

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

    input.current_position = current_pos.cast<double>();
    input.current_velocity = current_vel.cast<double>();
    input.current_acceleration = current_acc.cast<double>();

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
            input.target_position = q_target.cast<double>();
            input.target_velocity = q_final_dot.cast<double>();
            input.target_acceleration = Eigen::VectorXd::Zero(ss->num_dimensions);
            result = otg.calculate(input, traj_temp);

            if (result == ruckig::Result::Finished)
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
            std::cout << "Num. iter. " << num_iter << "\t q_final_dot: " << q_final_dot.transpose() << "\n";
        }
    }

    // Possible current position at the end of iteration
    Eigen::VectorXf new_current_pos { getPosition(time_current + t_iter_remain) };
    
    // If trajectory was not computed or robot is getting away from 'new_current_pos'
    if ((!traj_computed || (new_current_pos - q_target).norm() > (current_pos - q_target).norm()) && t_remain > 0)
    {
        is_zero_final_vel = true;
        input.target_velocity = Eigen::VectorXd::Zero(ss->num_dimensions);
        result = otg.calculate(input, traj_temp);

        if (result == ruckig::Result::Finished)
        {
            traj = traj_temp;
            time_current = 0;
            time_final = traj.get_duration();
            traj_computed = true;
            std::cout << "\t Trajectory computed with ZERO final velocity. \n";
        }
    }
    else std::cout << "\t Trajectory computed with NON-ZERO final velocity. \n";

    return traj_computed;
}

Eigen::VectorXf planning::trajectory::TrajectoryRuckig::getPosition(float t)
{
    ruckig::EigenVector<double, ruckig::DynamicDOFs> pos;
    traj.at_time(t, pos);
    return pos.cast<float>();
}

Eigen::VectorXf planning::trajectory::TrajectoryRuckig::getVelocity(float t)
{
    [[maybe_unused]] ruckig::EigenVector<double, ruckig::DynamicDOFs> pos, acc;
    ruckig::EigenVector<double, ruckig::DynamicDOFs> vel;

    traj.at_time(t, pos, vel, acc);
    return vel.cast<float>();
}

Eigen::VectorXf planning::trajectory::TrajectoryRuckig::getAcceleration(float t)
{
    [[maybe_unused]] ruckig::EigenVector<double, ruckig::DynamicDOFs> pos, vel;
    ruckig::EigenVector<double, ruckig::DynamicDOFs> acc;

    traj.at_time(t, pos, vel, acc);
    return acc.cast<float>();
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
