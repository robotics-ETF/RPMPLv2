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
    input.control_interface = ruckig::ControlInterface::Position;
    input.synchronization = ruckig::Synchronization::Time;
    time_join = -1;

    if (!input.validate())
        throw std::runtime_error("Invalid input parameters for Ruckig!");
}

planning::trajectory::TrajectoryRuckig::~TrajectoryRuckig() {}

bool planning::trajectory::TrajectoryRuckig::computeRegularTraj(const planning::trajectory::State &current, 
    const planning::trajectory::State &target)
{
    // std::cout << "Trying to compute a regular trajectory...\n";
    ruckig::Result result { ruckig::Result::Working };
    ruckig::Ruckig<ruckig::DynamicDOFs> otg(ss->num_dimensions);
    ruckig::Trajectory<ruckig::DynamicDOFs> traj_new(ss->num_dimensions);
    input.control_interface = ruckig::ControlInterface::Position;
    input.synchronization = ruckig::Synchronization::Time;
    setCurrentState(current);
    setTargetState(target);

    result = otg.calculate(input, traj_new);

    if (result == ruckig::Result::Working || result == ruckig::Result::Finished)
    {
        setTraj(traj_new);
        return true;
    }

    return false;
}

bool planning::trajectory::TrajectoryRuckig::computeSafeTraj(const planning::trajectory::State &current, 
    const planning::trajectory::State &target, float t_iter, float t_traj_max, const std::shared_ptr<base::State> q_current)
{
    // std::cout << "Trying to compute a safe trajectory...\n";
    ruckig::Result result { ruckig::Result::Working };
    ruckig::Ruckig<ruckig::DynamicDOFs> otg(ss->num_dimensions);
    ruckig::Trajectory<ruckig::DynamicDOFs> traj_new(ss->num_dimensions);
    input.control_interface = ruckig::ControlInterface::Position;
    input.synchronization = ruckig::Synchronization::Time;
    setCurrentState(current);
    setTargetState(target);

    result = otg.calculate(input, traj_new);

    if (result != ruckig::Result::Working && result != ruckig::Result::Finished)
    {
        // std::cout << "Could not compute trajectory!\n";
        return false;
    }

    auto computeRho = [&](Eigen::VectorXf pos) -> float
    {
        float rho { 0 };
        std::shared_ptr<Eigen::MatrixXf> skeleton { ss->robot->computeSkeleton(ss->getNewState(pos)) };
        for (size_t k = 1; k <= ss->robot->getNumLinks(); k++)
            rho = std::max(rho, (q_current->getSkeleton()->col(k) - skeleton->col(k)).norm());

        return rho;
    };

    ruckig::Trajectory<ruckig::DynamicDOFs> traj_emg_new(ss->num_dimensions);
    bool is_safe { false };
    bool traj_emg_computed { false };
    float rho_robot {};
    float rho_obs {};

    if (traj_new.get_duration() < t_traj_max && target.vel.norm() < RealVectorSpaceConfig::EQUALITY_THRESHOLD)
    {
        rho_obs = max_obs_vel * (t_iter + traj_new.get_duration());
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
            getPos(traj_new, t_traj_max), 
            getVel(traj_new, t_traj_max), 
            getAcc(traj_new, t_traj_max)
        );
        planning::trajectory::State target_temp(ss->num_dimensions);
        setCurrentState(current_temp);
        setTargetState(target_temp);

        result = otg.calculate(input, traj_emg_new);
        
        if (result == ruckig::Result::Working || result == ruckig::Result::Finished)
        {
            // std::cout << "Emergency trajectory is computed! \n";
            rho_obs = max_obs_vel * (t_iter + t_traj_max + traj_emg_new.get_duration());
            if (rho_obs < q_current->getDistance())
            {
                rho_robot = computeRho(getPos(traj_emg_new, traj_emg_new.get_duration()));
                if (rho_obs + rho_robot < q_current->getDistance())
                {
                    traj_emg_computed = true;
                    is_safe = true;
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
        // std::cout << "\tTrajectory is computed! \n";
        std::vector<Eigen::VectorXf> pos_points {};
        if (traj_emg_computed)
        {
            for (float t = 0; t <= t_traj_max; t += TrajectoryConfig::TIME_STEP)
                pos_points.emplace_back(getPos(traj_new, t));

            for (float t = 0; t <= traj_emg_new.get_duration(); t += TrajectoryConfig::TIME_STEP)
                pos_points.emplace_back(getPos(traj_emg_new, t));
        }
        else
        {
            for (float t = 0; t <= traj_new.get_duration(); t += TrajectoryConfig::TIME_STEP)
                pos_points.emplace_back(getPos(traj_new, t));
        }

        is_safe = isSafe(pos_points, q_current, t_iter);
        if (is_safe)
        {
            setTraj(traj_new);
            traj_emg = traj_emg_new;
            time_final = traj_emg_computed ? (t_traj_max + traj_emg_new.get_duration()) : traj_new.get_duration();
            time_join = traj_emg_computed ? t_traj_max : -1;
        }
    }

    // std::cout << "Trajectory is " << (is_safe ? "" : "NOT ") << "safe!\n";
    return is_safe;
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

Eigen::VectorXf planning::trajectory::TrajectoryRuckig::getJerk_(const ruckig::Trajectory<ruckig::DynamicDOFs> &traj_, float t)
{
    ruckig::StandardVector<double, ruckig::DynamicDOFs> pos(ss->num_dimensions);
    ruckig::StandardVector<double, ruckig::DynamicDOFs> vel(ss->num_dimensions);
    ruckig::StandardVector<double, ruckig::DynamicDOFs> acc(ss->num_dimensions);
    ruckig::StandardVector<double, ruckig::DynamicDOFs> jerk(ss->num_dimensions);
    size_t new_section;

    if (time_final > 0)
        traj_.at_time(t, pos, vel, acc, jerk, new_section);
    
    Eigen::VectorXf ret(ss->num_dimensions);
    for (size_t i = 0; i < ss->num_dimensions; i++)
        ret(i) = jerk[i];

    return ret;
}

Eigen::VectorXf planning::trajectory::TrajectoryRuckig::getJerk(float t)
{
    if (time_join > 0)
    {
        if (t < time_join)
            return getJerk_(traj, t);
        else
            return getJerk_(traj_emg, t - time_join);
    }

    return getJerk_(traj, t);
}

bool planning::trajectory::TrajectoryRuckig::convertPathToTraj(const std::vector<std::shared_ptr<base::State>> &path, 
                                                               [[maybe_unused]] bool is_safe)
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
