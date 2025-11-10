#include "TrajectoryRuckig.h"

planning::trajectory::TrajectoryRuckig::TrajectoryRuckig(const std::shared_ptr<base::StateSpace> &ss_, size_t num_waypoints) : 
    AbstractTrajectory(ss_), 
    input(ss_->num_dimensions, num_waypoints), 
    output(ss_->num_dimensions, num_waypoints), 
    traj(ss_->num_dimensions, num_waypoints), 
    traj_emg(ss_->num_dimensions, num_waypoints)
{
    setParams();
}

planning::trajectory::TrajectoryRuckig::TrajectoryRuckig
    (const std::shared_ptr<base::StateSpace> &ss_, planning::trajectory::State current, float max_iter_time_) : 
        AbstractTrajectory(ss_, max_iter_time_), 
        input(ss_->num_dimensions), 
        output(ss_->num_dimensions), 
        traj(ss_->num_dimensions), 
        traj_emg(ss_->num_dimensions)
{
    setCurrentState(current);
    setParams();
}

void planning::trajectory::TrajectoryRuckig::setParams()
{
    for (size_t i = 0; i < ss->num_dimensions; i++)
    {
        input.max_velocity[i] = ss->robot->getMaxVel(i);
        input.max_acceleration[i] = ss->robot->getMaxAcc(i);
        input.max_jerk[i] = ss->robot->getMaxJerk(i);
    }
    input.synchronization = ruckig::Synchronization::Time;
    time_join = -1;

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
        target.vel = target_vel_max;
        
        while (!traj_computed && num_iter++ < max_num_iter_trajectory && t_remain > 0)
        {
            setTargetState(target);
            result = otg.calculate(input, traj_new);
            
            if (result == ruckig::Result::Working || result == ruckig::Result::Finished)
            {
                setTraj(traj_new);
                traj_computed = true;
            }
            else
                target_vel_max = target.vel;

            target.vel = (target_vel_max + target_vel_min) / 2;
            t_remain -= std::chrono::duration_cast<std::chrono::microseconds>
                        (std::chrono::steady_clock::now() - time_start_).count() / 1e6;
            // std::cout << "Num. iter. " << num_iter << "\t target.vel: " << target.vel.transpose() << "\n";
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
            setTraj(traj_new);
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
bool planning::trajectory::TrajectoryRuckig::computeSafe(planning::trajectory::State current, planning::trajectory::State target, 
    float t_iter_remain, float t_max, const std::shared_ptr<base::State> q_current)
{
    std::chrono::steady_clock::time_point time_start_ { std::chrono::steady_clock::now() };
    ruckig::Result result { ruckig::Result::Working };
    ruckig::Ruckig<ruckig::DynamicDOFs> otg(ss->num_dimensions);
    ruckig::Trajectory<ruckig::DynamicDOFs> traj_new(ss->num_dimensions);
    ruckig::Trajectory<ruckig::DynamicDOFs> traj_temp(ss->num_dimensions);
    ruckig::Trajectory<ruckig::DynamicDOFs> traj_emg_new(ss->num_dimensions);
    ruckig::Trajectory<ruckig::DynamicDOFs> traj_emg_temp(ss->num_dimensions);
    
    float rho_robot {};
    float rho_obs {};
    bool is_safe {};
    bool traj_computed { false };
    bool traj_emg_computed { false };
    float t_iter { max_iter_time - t_iter_remain };
    float t_traj_max { t_iter_remain + (max_iter_time - max_remaining_iter_time) };
    int num_iter { 0 };
    int max_num_iter = std::ceil(std::log2(RealVectorSpaceConfig::NUM_INTERPOLATION_VALIDITY_CHECKS * 
                                           (current.pos - target.pos).norm() / RRTConnectConfig::EPS_STEP));
    if (max_num_iter <= 0) max_num_iter = 1;
    Eigen::VectorXf target_pos_min { current.pos };
    Eigen::VectorXf target_pos_max { target.pos };
    time_join = -1;

    auto computeRho = [&](Eigen::VectorXf pos) -> float
    {
        float rho { 0 };
        std::shared_ptr<Eigen::MatrixXf> skeleton { ss->robot->computeSkeleton(ss->getNewState(pos)) };
        for (size_t k = 1; k <= ss->robot->getNumLinks(); k++)
            rho = std::max(rho, (q_current->getSkeleton()->col(k) - skeleton->col(k)).norm());

        return rho;
    };
    
    while (num_iter++ < max_num_iter &&
           std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start_).count() < t_max * 1e6)
    {
        is_safe = false;
        // std::cout << "Num. iter.  " << num_iter << "\n";
        // std::cout << "target.pos: " << target.pos.transpose() << "\n";

        input.control_interface = ruckig::ControlInterface::Position;
        input.synchronization = ruckig::Synchronization::Time;
        setCurrentState(current);
        setTargetState(target);
        result = otg.calculate(input, traj_temp);

        if (result == ruckig::Result::Working || result == ruckig::Result::Finished)
        {
            // std::cout << "Trajectory is computed! \n";
            if (traj_temp.get_duration() < t_traj_max)
            {
                rho_obs = max_obs_vel * (t_iter + traj_temp.get_duration());
                if (rho_obs < q_current->getDistance())
                {
                    rho_robot = computeRho(target.pos);
                    if (rho_obs + rho_robot < q_current->getDistance())
                    {
                        traj_emg_computed = false;
                        is_safe = true;
                    }
                }
            }
            else
            {
                // Synchronization is disabled so that each DoF stops as fast as possible independently
                input.control_interface = ruckig::ControlInterface::Velocity;
                input.synchronization = ruckig::Synchronization::None;
                planning::trajectory::State current_temp
                (
                    getPos(traj_temp, t_traj_max), 
                    getVel(traj_temp, t_traj_max), 
                    getAcc(traj_temp, t_traj_max)
                );
                setCurrentState(current_temp);
                result = otg.calculate(input, traj_emg_temp);

                if (result == ruckig::Result::Working || result == ruckig::Result::Finished)
                {
                    // std::cout << "Emergency trajectory is computed! \n";
                    rho_obs = max_obs_vel * (t_iter + t_traj_max + traj_emg_temp.get_duration());
                    if (rho_obs < q_current->getDistance())
                    {
                        rho_robot = computeRho(getPos(traj_emg_temp, traj_emg_temp.get_duration()));
                        if (rho_obs + rho_robot < q_current->getDistance())
                        {
                            traj_emg_new = traj_emg_temp;
                            traj_emg_computed = true;
                            is_safe = true;
                        }
                    }
                }
            }
        }

        // std::cout << "\trho_obs: " << rho_obs << " [m]\t";
        // std::cout << "rho_robot: " << rho_robot << " [m]\t";
        // std::cout << "rho_sum: " << rho_obs + rho_robot << " [m]\t";
        // std::cout << "d_c: " << q_current->getDistance() << " [m]\n";
        // rho_obs = 0; rho_robot = 0;     // Reset only for console output
        
        if (is_safe)
        {
            // std::cout << "\tRobot is safe! \n";
            traj_new = traj_temp;
            traj_computed = true;
            target_pos_min = target.pos;
            if (num_iter == 1) 
                break;
        }
        else
        {
            // std::cout << "\tRobot is NOT safe! \n";
            target_pos_max = target.pos;
        }
        target.pos = (target_pos_min + target_pos_max) / 2;
    }

    if (traj_computed)    // Check whether computed trajectories are collision-free
    {
        std::vector<Eigen::VectorXf> pos_points {};
        if (traj_emg_computed)
        {
            for (float t = TrajectoryConfig::TIME_STEP; t <= t_traj_max; t += TrajectoryConfig::TIME_STEP)
                pos_points.emplace_back(getPos(traj_new, t));

            for (float t = TrajectoryConfig::TIME_STEP; t <= traj_emg_new.get_duration(); t += TrajectoryConfig::TIME_STEP)
                pos_points.emplace_back(getPos(traj_emg_new, t));
        }
        else
        {
            for (float t = TrajectoryConfig::TIME_STEP; t <= traj_new.get_duration(); t += TrajectoryConfig::TIME_STEP)
                pos_points.emplace_back(getPos(traj_new, t));
        }

        traj_computed = isSafe(pos_points, q_current, t_iter);
        // std::cout << "traj_computed: " << traj_computed << "\n";

        if (traj_computed)
        {
            setTraj(traj_new);
            traj_emg = traj_emg_new;
            time_final = traj_emg_computed ? (t_traj_max + traj_emg_new.get_duration()) : traj_new.get_duration();
            time_join = traj_emg_computed ? t_traj_max : -1;
        }
    }

    return traj_computed;
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

void planning::trajectory::TrajectoryRuckig::setTraj(const ruckig::Trajectory<ruckig::DynamicDOFs> &traj_)
{
    traj = traj_;
    time_current = 0;
    time_final = traj.get_duration();
}

Eigen::VectorXf planning::trajectory::TrajectoryRuckig::getPos(const ruckig::Trajectory<ruckig::DynamicDOFs> &traj_, float t)
{
    ruckig::StandardVector<double, ruckig::DynamicDOFs> pos(ss->num_dimensions);

    if (time_final == 0)
        pos = input.current_position;
    else
        traj_.at_time(t, pos);

    Eigen::VectorXf ret(ss->num_dimensions);
    for (size_t i = 0; i < ss->num_dimensions; i++)
        ret(i) = pos[i];

    return ret;
}

Eigen::VectorXf planning::trajectory::TrajectoryRuckig::getPosition(float t)
{
    if (time_join > 0)
    {
        if (t < time_join)
            return getPos(traj, t);
        else
            return getPos(traj_emg, t - time_join);
    }

    return getPos(traj, t);
}

Eigen::VectorXf planning::trajectory::TrajectoryRuckig::getVel(const ruckig::Trajectory<ruckig::DynamicDOFs> &traj_, float t)
{
    ruckig::StandardVector<double, ruckig::DynamicDOFs> pos(ss->num_dimensions);
    ruckig::StandardVector<double, ruckig::DynamicDOFs> vel(ss->num_dimensions);
    ruckig::StandardVector<double, ruckig::DynamicDOFs> acc(ss->num_dimensions);

    if (time_final == 0)
        vel = input.current_velocity;
    else
        traj_.at_time(t, pos, vel, acc);

    Eigen::VectorXf ret(ss->num_dimensions);
    for (size_t i = 0; i < ss->num_dimensions; i++)
        ret(i) = vel[i];

    return ret;
}

Eigen::VectorXf planning::trajectory::TrajectoryRuckig::getVelocity(float t)
{
    if (time_join > 0)
    {
        if (t < time_join)
            return getVel(traj, t);
        else
            return getVel(traj_emg, t - time_join);
    }

    return getVel(traj, t);
}

Eigen::VectorXf planning::trajectory::TrajectoryRuckig::getAcc(const ruckig::Trajectory<ruckig::DynamicDOFs> &traj_, float t)
{
    ruckig::StandardVector<double, ruckig::DynamicDOFs> pos(ss->num_dimensions);
    ruckig::StandardVector<double, ruckig::DynamicDOFs> vel(ss->num_dimensions);
    ruckig::StandardVector<double, ruckig::DynamicDOFs> acc(ss->num_dimensions);

    if (time_final == 0)
        acc = input.current_acceleration;
    else
        traj_.at_time(t, pos, vel, acc);
    
    Eigen::VectorXf ret(ss->num_dimensions);
    for (size_t i = 0; i < ss->num_dimensions; i++)
        ret(i) = acc[i];

    return ret;
}

Eigen::VectorXf planning::trajectory::TrajectoryRuckig::getAcceleration(float t)
{
    if (time_join > 0)
    {
        if (t < time_join)
            return getAcc(traj, t);
        else
            return getAcc(traj_emg, t - time_join);
    }

    return getAcc(traj, t);
}

bool planning::trajectory::TrajectoryRuckig::convertPathToTraj(const std::vector<std::shared_ptr<base::State>> &path)
{
    planning::trajectory::State start(path.front()->getCoord());
    planning::trajectory::State goal(path.back()->getCoord());
    setCurrentState(start);
    setTargetState(goal);

    if (path.size() > 2)
    {
        ruckig::StandardVector<double, ruckig::DynamicDOFs> pos(ss->num_dimensions);
        for (size_t idx = 1; idx < path.size()-1; idx++)
        {
            for (size_t i = 0; i < ss->num_dimensions; i++)
                pos[i] = path[idx]->getCoord(i);
            
            input.intermediate_positions.emplace_back(pos);
        }
    }

    ruckig::Result result { ruckig::Result::Working };
    ruckig::Ruckig<ruckig::DynamicDOFs> otg(ss->num_dimensions);

    result = otg.calculate(input, traj);
    // std::cout << "result: " << result << "\n";
    
    if (result == ruckig::Result::Working || result == ruckig::Result::Finished)
    {
        traj = output.trajectory;
        time_current = 0;
        time_final = traj.get_duration();
        return true;
    }

    return false;
}
