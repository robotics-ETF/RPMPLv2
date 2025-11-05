#include "AbstractTrajectory.h"

planning::trajectory::State::State(size_t num_DOFs)
{
    pos = Eigen::VectorXf::Zero(num_DOFs);
    vel = Eigen::VectorXf::Zero(num_DOFs);
    acc = Eigen::VectorXf::Zero(num_DOFs);
}

planning::trajectory::State::State(const Eigen::VectorXf &pos_)
{
    pos = pos_;
    vel = Eigen::VectorXf::Zero(pos.size());
    acc = Eigen::VectorXf::Zero(pos.size());
}

planning::trajectory::State::State(const Eigen::VectorXf &pos_, const Eigen::VectorXf &vel_, const Eigen::VectorXf &acc_)
{
    pos = pos_;
    vel = vel_;
    acc = acc_;
}

planning::trajectory::AbstractTrajectory::AbstractTrajectory(const std::shared_ptr<base::StateSpace> &ss_)
{
    ss = ss_;    
    max_iter_time = 0;
    max_remaining_iter_time = INFINITY;
    max_obs_vel = 0;

    setParams();
}

planning::trajectory::AbstractTrajectory::AbstractTrajectory(const std::shared_ptr<base::StateSpace> &ss_, float max_iter_time_)
{
    ss = ss_;
    max_iter_time = max_iter_time_;
    max_remaining_iter_time = 0;
    time_begin = 0;
    time_end = 0;
    time_current = 0;
    time_final = 0;
    is_zero_final_vel = true;

    max_obs_vel = 0;
    for (size_t i = 0; i < ss->env->getNumObjects(); i++)
    {
        if (ss->env->getObject(i)->getMaxVel() > max_obs_vel)
            max_obs_vel = ss->env->getObject(i)->getMaxVel();
    }

    setParams();
}

planning::trajectory::AbstractTrajectory::~AbstractTrajectory() {}

void planning::trajectory::AbstractTrajectory::setParams()
{
    traj_points_current_iter = {};
    
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

/// @brief Check whether 'q' is a final configuration of the trajectory.
/// @param pos Configuration to be checked.
/// @return True if yes, false if not.
bool planning::trajectory::AbstractTrajectory::isFinalConf(const Eigen::VectorXf &pos)
{
    return ((pos - getPosition(time_final)).norm() < RealVectorSpaceConfig::EQUALITY_THRESHOLD) ? true : false;
}

void planning::trajectory::AbstractTrajectory::addTrajPointCurrentIter(const Eigen::VectorXf &pos)
{
    traj_points_current_iter.emplace_back(pos);
}

void planning::trajectory::AbstractTrajectory::clearTrajPointCurrentIter()
{
    traj_points_current_iter.clear();
}

// This function is just for debugging. It operates in real-time by logging all trajectory points. 
// You can hardcode a desired output path for the file to be saved.
void planning::trajectory::AbstractTrajectory::recordTrajectory(bool traj_computed, float t_offset)
{
    std::ofstream output_file {};
    output_file.open("/home/spear/xarm6-etf-lab/src/etf_modules/RPMPLv2/data/planar_2dof/scenario_random_obstacles/visualize_trajectory/test.log", 
        std::ofstream::app);
    
    output_file << "Time offset [s] in the current iteration:\n";
    output_file << t_offset << "\n";
    output_file << "--------------------------------------------------------------------\n";

    if (traj_computed)
    {
        output_file << "New trajectory is computed! \n";
        output_file << "Time [s] (all points): \n";
        for (float t = time_current; t < time_final; t += TrajectoryConfig::TIME_STEP)
            output_file << t << "\n";
        output_file << time_final << "\n";
        output_file << "--------------------------------------------------------------------\n";

        output_file << "Position (all points): \n";
        for (float t = time_current; t < time_final; t += TrajectoryConfig::TIME_STEP)
            output_file << getPosition(t).transpose() << "\n";
        output_file << getPosition(time_final).transpose() << "\n";
        output_file << "--------------------------------------------------------------------\n";

        output_file << "Velocity (all points): \n";
        for (float t = time_current; t < time_final; t += TrajectoryConfig::TIME_STEP)
            output_file << getVelocity(t).transpose() << "\n";
        output_file << getVelocity(time_final).transpose() << "\n";
        output_file << "--------------------------------------------------------------------\n";

        output_file << "Acceleration (all points): \n";
        for (float t = time_current; t < time_final; t += TrajectoryConfig::TIME_STEP)
            output_file << getAcceleration(t).transpose() << "\n";
        output_file << getAcceleration(time_final).transpose() << "\n";
        output_file << "--------------------------------------------------------------------\n";
    }
    else
    {
        output_file << "Continuing with the previous trajectory! \n";
        output_file << INFINITY << "\n";
        output_file << "--------------------------------------------------------------------\n";
    }

    output_file << "Delta time [s] in the current iteration: \n";
    output_file << time_end - time_current << "\n";
    output_file << "--------------------------------------------------------------------\n";

}
