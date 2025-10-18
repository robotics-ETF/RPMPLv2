#include "TrajectoryRuckig.h"

planning::trajectory::TrajectoryRuckig::TrajectoryRuckig(const std::shared_ptr<base::StateSpace> &ss_) : 
    AbstractTrajectory(ss_), 
    input(ss_->num_dimensions), 
    traj(ss_->num_dimensions)
{}

planning::trajectory::TrajectoryRuckig::TrajectoryRuckig
    (const std::shared_ptr<base::StateSpace> &ss_, planning::trajectory::State current, float max_iter_time_) : 
        AbstractTrajectory(ss_, max_iter_time_), 
        input(ss_->num_dimensions), 
        traj(ss_->num_dimensions)
{
    setCurrentState(current);
    
    for (size_t i = 0; i < ss->num_dimensions; i++)
    {
        input.max_velocity[i] = ss->robot->getMaxVel(i);
        input.max_acceleration[i] = ss->robot->getMaxAcc(i);
        input.max_jerk[i] = ss->robot->getMaxJerk(i);
    }
    input.synchronization = ruckig::Synchronization::Time;

    if (!input.validate())
        throw std::runtime_error("Invalid input parameters for Ruckig!");
}

planning::trajectory::TrajectoryRuckig::~TrajectoryRuckig() {}

/// @brief Compute a regular trajectory that is not surely safe for environment, meaning that,
/// if collision eventually occurs, it may be at robot's non-zero velocity.
/// @param current Current robot's state
/// @param target Target robot's state
/// @param t_iter_remain Remaining time in [s] in the current iteration
/// @param t_max Maximal available time in [s] for a trajectory computing
/// @param non_zero_final_vel Whether final velocity can be non-zero
/// @return The success of a trajectory computation
bool planning::trajectory::TrajectoryRuckig::computeRegular(planning::trajectory::State current, 
    planning::trajectory::State target, float t_iter_remain, float t_max, bool non_zero_final_vel)
{
    std::chrono::steady_clock::time_point time_start_ { std::chrono::steady_clock::now() };
    float t_remain { t_max };
    bool traj_computed { false };
    ruckig::Result result { ruckig::Result::Working };
    ruckig::Ruckig<ruckig::DynamicDOFs> otg(ss->num_dimensions);
    ruckig::Trajectory<ruckig::DynamicDOFs> traj_new(ss->num_dimensions);
    setCurrentState(current);
    setTargetState(target);

    if (non_zero_final_vel)
    {
        is_zero_final_vel = false;
        size_t num_iter { 0 };
        float delta_t_max { ((target.pos - current.pos).cwiseQuotient(ss->robot->getMaxVel())).cwiseAbs().maxCoeff() };
        Eigen::VectorXf target_vel_max { (target.pos - current.pos) / delta_t_max };
        Eigen::VectorXf target_vel_min { Eigen::VectorXf::Zero(ss->num_dimensions) };
        Eigen::VectorXf target_vel { target_vel_max };
        
        while (!traj_computed && num_iter++ < max_num_iter_trajectory && t_remain > 0)
        {
            target.vel = target_vel;
            setTargetState(target);
            result = otg.calculate(input, traj_new);
            
            if (result == ruckig::Result::Working || result == ruckig::Result::Finished)
            {
                traj = traj_new;
                time_current = 0;
                time_final = traj.get_duration();
                traj_computed = true;
            }
            else
                target_vel_max = target_vel;

            target_vel = (target_vel_max + target_vel_min) / 2;
            t_remain -= std::chrono::duration_cast<std::chrono::microseconds>
                        (std::chrono::steady_clock::now() - time_start_).count() / 1e6;
            // std::cout << "Num. iter. " << num_iter << "\t target_vel: " << target_vel.transpose() << "\n";
        }
    }

    // Possible current position at the end of iteration
    Eigen::VectorXf new_current_pos { getPosition(time_current + t_iter_remain) };
    
    // If trajectory was not computed or robot is getting away from 'new_current_pos'
    if ((!traj_computed || (new_current_pos - target.pos).norm() > (current.pos - target.pos).norm()) && t_remain > 0)
    {
        is_zero_final_vel = true;
        target.vel = Eigen::VectorXf::Zero(ss->num_dimensions);
        setTargetState(target);
        result = otg.calculate(input, traj_new);

        if (result == ruckig::Result::Working || result == ruckig::Result::Finished)
        {
            traj = traj_new;
            time_current = 0;
            time_final = traj.get_duration();
            traj_computed = true;
        }
    }

    // std::cout << "\t Trajectory " << (!traj_computed ? "NOT " : "") << "computed with " 
    //           << (!is_zero_final_vel ? "NON-" : "") <<  "ZERO final velocity. \n";
    return traj_computed;
}

/// @brief Compute a safe trajectory that will render a robot motion surely safe for environment. 
/// If collision eventually occurs, it will be at robot's zero velocity, meaning that an obstacle hit the robot, and not vice versa. 
/// @param current Current robot's state
/// @param target Target robot's state
/// @param t_iter_remain Remaining time in [s] in the current iteration
/// @param t_max Maximal available time in [s] for a trajectory computing
/// @param q_current Current robot's configuration
/// @return The success of a trajectory computation
bool planning::trajectory::TrajectoryRuckig::computeSafe([[maybe_unused]] planning::trajectory::State current, 
    [[maybe_unused]] planning::trajectory::State target, [[maybe_unused]] float t_iter_remain, [[maybe_unused]] float t_max, 
    [[maybe_unused]] const std::shared_ptr<base::State> q_current)
{
    // TODO
    return false;
}

void planning::trajectory::TrajectoryRuckig::setCurrentState(const planning::trajectory::State &current)
{
    for (size_t i = 0; i < ss->num_dimensions; i++)
    {
        input.current_position[i] = current.pos(i);
        input.current_velocity[i] = current.vel(i);
        input.current_acceleration[i] = current.acc(i);
    }
}

void planning::trajectory::TrajectoryRuckig::setTargetState(const planning::trajectory::State &target)
{
    for (size_t i = 0; i < ss->num_dimensions; i++)
    {
        input.target_position[i] = target.pos(i);
        input.target_velocity[i] = target.vel(i);
        input.target_acceleration[i] = target.acc(i);
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
