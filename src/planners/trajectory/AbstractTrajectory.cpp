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

/// @brief Check whether the computed trajectory is safe (i.e., collision-free during the time interval [0, 't_final'])
/// @param pos_points Trajectory points which need to be validated.
/// @param q_current Current configuration from where bur spines are generated.
/// @param nearest_points Nearest points between the robot and obstacles.
/// @param t_iter Elapsed time from the beginning of iteration to a time instance when the trajectory is starting.
/// @param time_step Time step when moving over the trajectory (default: TrajectoryConfig::TIME_STEP).
/// @return True if safe. False if not.
/// @note 'q_current' must have a distance-to-obstacles or its underestimation!
bool planning::trajectory::AbstractTrajectory::isSafe
    (const std::vector<Eigen::VectorXf> &pos_points, const std::shared_ptr<base::State> q_current, float t_iter, float time_step)
{
    // std::chrono::steady_clock::time_point time_start_ { std::chrono::steady_clock::now() };
    float rho_robot {};
    float rho_obs {};
    float t_init { 0 };
    std::shared_ptr<base::State> q_temp { q_current };
    std::shared_ptr<base::State> q_final { nullptr };
	Eigen::VectorXf delta_q {};

    for (size_t i = 0; i < pos_points.size(); i++)
    {
        // std::cout << "Considering t: " << t << "\n";
        q_final = ss->getNewState(pos_points[i]);
        if (ss->robot->checkSelfCollision(q_final))
            return false;
        
        t_iter += time_step;
        rho_obs = max_obs_vel * (t_iter - t_init);
        delta_q = (q_final->getCoord() - q_temp->getCoord()).cwiseAbs();
        ss->robot->computeEnclosingRadii(q_temp);

        for (size_t i = 0; i < ss->robot->getNumDOFs(); i++)
        {
            rho_robot = q_temp->getEnclosingRadii()->col(i+1).dot(delta_q);
            if (rho_robot + rho_obs >= q_temp->getDistanceProfile(i))    // Possible collision
            {
                // std::cout << "********** Possible collision ********** \n";
                q_temp = q_final;
                computeDistanceUnderestimation(q_temp, q_current->getNearestPoints(), t_iter);
                t_init = t_iter;

                if (q_temp->getDistance() <= 0)
                {
                    // std::cout << "\t Trajectory is NOT safe! \n";
                    return false;
                }
                break;
            }
            // else std::cout << "\t OK! rho_robot + rho_obs < q_temp->getDistance() \n";
        }
    }
    
    // auto t_elapsed { std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start_).count() };
    // std::cout << "Elapsed time for the trajectory collision checking: " << t_elapsed << " [us] \n";
    return true;
}

/// @brief Compute an underestimation of distance-to-obstacles 'd_c', i.e., a distance-to-planes, for each robot's link,
/// i.e., compute a distance-to-planes profile function, when the robot takes a configuration 'q'.
/// Planes approximate obstacles, and are generated according to 'nearest_points'.
/// Each plane is moved at the maximal obstacle velocity 'ss->env->getObject(j)->getMaxVel()' towards the robot 
/// during the time interval 'delta_t'.
/// @param q Configuration of the robot.
/// @param nearest_points Nearest points between the robot and obstacles.
/// @param delta_t Time interval when planes are moving towards the robot.
/// @return Underestimation of distance-to-obstacles.
/// Note that if 'd_c' is negative, it means that one or more robot's links are penetrating through the plane,
/// or they are located on the other side of the plane.
float planning::trajectory::AbstractTrajectory::computeDistanceUnderestimation(const std::shared_ptr<base::State> q, 
	const std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points, float delta_t)
{
	float d_c_temp {};
    float d_c { INFINITY };
	std::vector<float> d_c_profile(ss->robot->getNumLinks(), 0);
    Eigen::Vector3f R {};		// Robot's nearest point
	Eigen::Vector3f O {};    	// Obstacle's nearest point
    Eigen::Vector3f delta_RO {};
	std::shared_ptr<Eigen::MatrixXf> skeleton { ss->robot->computeSkeleton(q) };
    
    for (size_t i = 0; i < ss->robot->getNumLinks(); i++)
    {
		d_c_profile[i] = INFINITY;
        for (size_t j = 0; j < ss->env->getNumObjects(); j++)
        {
            O = nearest_points->at(j).col(i).tail(3);
			if (O.norm() == INFINITY)
                continue;
			
            R = nearest_points->at(j).col(i).head(3);

            // Move both points, R and O, to retain a normal plane vector 'R - O'
            delta_RO = (R - O).normalized() * ss->env->getObject(j)->getMaxVel() * delta_t;
            R += delta_RO;
            O += delta_RO;

            d_c_temp = std::min((skeleton->col(i) - O).dot((R - O).normalized()), 
                                (skeleton->col(i+1) - O).dot((R - O).normalized())) 
                                - ss->robot->getCapsuleRadius(i);
            if (d_c_temp < 0)
                return 0;

			d_c_profile[i] = std::min(d_c_profile[i], d_c_temp);

            // std::cout << "(i, j) = " << "(" << i << ", " << j << "):" << std::endl;
            // std::cout << "Robot nearest point:    " << R.transpose() << std::endl;
            // std::cout << "Obstacle nearest point: " << O.transpose() << std::endl;
            // std::cout << "d_c: " << d_c_profile[i] << std::endl;
		}
		d_c = std::min(d_c, d_c_profile[i]);
    }

	q->setDistance(d_c);
	q->setDistanceProfile(d_c_profile);
	q->setIsRealDistance(false);

	return d_c;
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
