#include "Splines.h"

planning::drbt::Splines::Splines(const std::shared_ptr<base::StateSpace> ss_, const std::shared_ptr<base::State> q_current_, 
                                 const std::shared_ptr<base::State> q_target_, bool all_robot_vel_same_)
{
    ss = ss_;
    q_current = q_current_;
    q_target = q_target_;
    all_robot_vel_same = all_robot_vel_same_;

    max_obs_vel = 0;
    for (size_t i = 0; i < ss->env->getNumObjects(); i++)
    {
        if (ss->env->getObject(i)->getMaxVel() > max_obs_vel)
            max_obs_vel = ss->env->getObject(i)->getMaxVel();
    }

    spline_current = std::make_shared<planning::trajectory::Spline5>(ss->robot, q_current->getCoord());
    spline_next = spline_current;
    max_num_iter_spline_regular = all_robot_vel_same ? 
        std::ceil(std::log2(2 * ss->robot->getMaxVel(0) / SplinesConfig::FINAL_VELOCITY_STEP)) :
        std::ceil(std::log2(2 * ss->robot->getMaxVel().maxCoeff() / SplinesConfig::FINAL_VELOCITY_STEP));
}

/// @brief Compute a regular spline that is not surely safe for environment, meaning that,
/// if collision eventually occurs, it may be at robot's non-zero velocity.
/// @param current_pos Current robot's position
/// @param current_vel Current robot's velocity
/// @param current_acc Current robot's acceleration
/// @param t_iter_remain Remaining time in [s] in the current iteration
/// @param t_max Maximal available time in [s] for a spline computing
/// @param non_zero_final_vel Whether final spline velocity can be non-zero
/// @return The success of a spline computation
bool planning::drbt::Splines::computeRegular(Eigen::VectorXf &current_pos, Eigen::VectorXf &current_vel, Eigen::VectorXf &current_acc, 
                                             float t_iter_remain, float t_max, bool non_zero_final_vel)
{
    std::chrono::steady_clock::time_point time_start_ { std::chrono::steady_clock::now() };
    spline_next = std::make_shared<planning::trajectory::Spline5>(ss->robot, current_pos, current_vel, current_acc);
    std::shared_ptr<planning::trajectory::Spline> spline_next_new 
        { std::make_shared<planning::trajectory::Spline5>(ss->robot, current_pos, current_vel, current_acc) };
    bool spline_computed { false };
    Eigen::VectorXf new_current_pos {};    // Possible current position at the end of iteration

    if (non_zero_final_vel)
    {
        float num_iter { 0 };
        float delta_t_max { ((q_target->getCoord() - current_pos).cwiseQuotient(ss->robot->getMaxVel())).cwiseAbs().maxCoeff() };
        Eigen::VectorXf q_final_dot_max { (q_target->getCoord() - current_pos) / delta_t_max };
        Eigen::VectorXf q_final_dot_min { Eigen::VectorXf::Zero(ss->num_dimensions) };
        Eigen::VectorXf q_final_dot {};
        
        while (num_iter++ < max_num_iter_spline_regular &&
               std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start_).count() < t_max * 1e6)
        {
            q_final_dot = (q_final_dot_max + q_final_dot_min) / 2;
            // std::cout << "Num. iter. " << num_iter << "\t q_final_dot: " << q_final_dot.transpose() << "\n";

            if (spline_next_new->compute(q_target->getCoord(), q_final_dot)) 
            {
                *spline_next = *spline_next_new;
                q_final_dot_min = q_final_dot;
                spline_computed = true;
            }
            else
                q_final_dot_max = q_final_dot;
        }

        new_current_pos = spline_computed ? 
                          spline_next->getPosition(t_iter_remain) : 
                          spline_current->getPosition(spline_current->getTimeCurrent() + t_iter_remain);
    }
    
    // If spline was not computed or robot is getting away from 'new_current_pos'
    if (!spline_computed || 
        (new_current_pos - q_target->getCoord()).norm() > (current_pos - q_target->getCoord()).norm())
    {
        spline_computed = spline_next_new->compute(q_target->getCoord());
        if (spline_computed)
        {
            spline_next = spline_next_new;
            // std::cout << "\t Spline computed with ZERO final velocity. \n";
        }
    }
    // else std::cout << "\t Spline computed with NON-ZERO final velocity. \n";

    return spline_computed;
}

/// @brief Compute a safe spline that will render a robot motion surely safe for environment. 
/// If collision eventually occurs, it will be at robot's zero velocity, meaning that an obstacle hit the robot, and not vice versa. 
/// @param current_pos Current robot's position
/// @param current_vel Current robot's velocity
/// @param current_acc Current robot's acceleration
/// @param t_iter_remain Remaining time in [s] in the current iteration
/// @param t_max Maximal available time in [s] for a spline computing
/// @return The success of a spline computation
bool planning::drbt::Splines::computeSafe(Eigen::VectorXf &current_pos, Eigen::VectorXf &current_vel, Eigen::VectorXf &current_acc, 
                                          float t_iter_remain, float t_max)
{
    std::chrono::steady_clock::time_point time_start_ { std::chrono::steady_clock::now() };
    std::shared_ptr<planning::trajectory::Spline> spline_next_new
        { std::make_shared<planning::trajectory::Spline5>(ss->robot, current_pos, current_vel, current_acc) };
    std::shared_ptr<planning::trajectory::Spline> spline_next_temp 
        { std::make_shared<planning::trajectory::Spline5>(ss->robot, current_pos, current_vel, current_acc) };
    std::shared_ptr<planning::trajectory::Spline> spline_emergency_new 
        { std::make_shared<planning::trajectory::Spline4>(ss->robot, current_pos, current_vel, current_acc) };
    std::shared_ptr<planning::trajectory::Spline> spline_emergency_temp { nullptr };
    
    float rho_robot {};
    float rho_obs {};
    bool is_safe {};
    bool spline_computed { false };
    bool spline_emergency_computed { false };
    float t_iter { DRGBTConfig::MAX_ITER_TIME - t_iter_remain };
    float t_spline_max { t_iter_remain + DRGBTConfig::MAX_TIME_TASK1 };
    int num_iter { 0 };
    int max_num_iter = std::ceil(std::log2(RealVectorSpaceConfig::NUM_INTERPOLATION_VALIDITY_CHECKS * 
                                           (current_pos - q_target->getCoord()).norm() / RRTConnectConfig::EPS_STEP));
    if (max_num_iter <= 0) max_num_iter = 1;
    Eigen::VectorXf q_final_min { current_pos };
    Eigen::VectorXf q_final_max { q_target->getCoord() };
    Eigen::VectorXf q_final { q_target->getCoord() };

    auto computeRho = [&](Eigen::VectorXf q_coord) -> float
    {
        float rho { 0 };
        std::shared_ptr<Eigen::MatrixXf> skeleton { ss->robot->computeSkeleton(ss->getNewState(q_coord)) };
        for (size_t k = 1; k <= ss->robot->getNumLinks(); k++)
            rho = std::max(rho, (q_current->getSkeleton()->col(k) - skeleton->col(k)).norm());

        return rho;
    };
    
    while (num_iter++ < max_num_iter &&
           std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start_).count() < t_max * 1e6)
    {
        is_safe = false;
        // std::cout << "Num. iter. " << num_iter << "\n";
        // std::cout << "q_final: " << q_final.transpose() << "\n";

        if (spline_next_temp->compute(q_final))
        {
            // std::cout << "Spline is computed! \n";
            if (spline_next_temp->getTimeFinal() < t_spline_max)
            {
                rho_obs = max_obs_vel * (t_iter + spline_next_temp->getTimeFinal());
                if (rho_obs < q_current->getDistance())
                {
                    rho_robot = computeRho(q_final);
                    if (rho_obs + rho_robot < q_current->getDistance())
                    {
                        spline_emergency_computed = false;
                        is_safe = true;
                    }
                }
            }
            else
            {
                spline_emergency_temp = std::make_shared<planning::trajectory::Spline4>
                (
                    ss->robot,
                    spline_next_temp->getPosition(t_spline_max),
                    spline_next_temp->getVelocity(t_spline_max),
                    spline_next_temp->getAcceleration(t_spline_max)
                );

                if (spline_emergency_temp->compute())
                {
                    // std::cout << "Emergency spline is computed! \n";
                    rho_obs = max_obs_vel * (t_iter + t_spline_max + spline_emergency_temp->getTimeFinal());
                    if (rho_obs < q_current->getDistance())
                    {
                        rho_robot = computeRho(spline_emergency_temp->getPosition(INFINITY));
                        if (rho_obs + rho_robot < q_current->getDistance())
                        {
                            *spline_emergency_new = *spline_emergency_temp;
                            spline_emergency_computed = true;
                            is_safe = true;
                            spline_next_temp->setTimeFinal(t_spline_max);
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
            *spline_next_new = *spline_next_temp;
            spline_computed = true;
            q_final_min = q_final;
            if (num_iter == 1) 
                break;
        }
        else
        {
            // std::cout << "\tRobot is NOT safe! \n";
            q_final_max = q_final;
        }
        q_final = (q_final_min + q_final_max) / 2;
    }

    if (spline_computed)    // Check whether computed splines are collision-free
    {
        spline_next = spline_emergency_computed ?
                      std::make_shared<planning::trajectory::CompositeSpline>
                          (std::vector<std::shared_ptr<planning::trajectory::Spline>>({ spline_next_new, spline_emergency_new })) :
                      spline_next_new;

        // std::cout << "spline_next: \n" << spline_next << "\n";
        spline_computed = !checkCollision(ss->getNewState(q_current), t_iter);
    }

    return spline_computed;
}

/// @brief Check whether 'spline_next' is collision-free during the spline time interval [0, 'spline_next->getTimeFinal()']
/// @param q_init Initial configuration from where bur spines are generated.
/// @param t_iter Elapsed time from the beginning of iteration to a time instance when 'spline' is starting.
/// @return True if the collision occurs. False if not.
/// @note 'q_init' must have a distance-to-obstacles or its underestimation!
bool planning::drbt::Splines::checkCollision(std::shared_ptr<base::State> q_init, float t_iter)
{
    // std::chrono::steady_clock::time_point time_start_ { std::chrono::steady_clock::now() };    
    float delta_t { SplinesConfig::TIME_STEP };
    size_t num_iter = std::ceil(spline_next->getTimeFinal() / delta_t);
    delta_t = spline_next->getTimeFinal() / num_iter;
    float rho_robot {};
    float rho_obs {};
    float t_init { 0 };
    Eigen::VectorXf q_final {};
	Eigen::VectorXf delta_q {};

    for (float t = delta_t; t <= spline_next->getTimeFinal() + RealVectorSpaceConfig::EQUALITY_THRESHOLD; t += delta_t)
    {
        // std::cout << "Considering t: " << t << "\n";
        t_iter += delta_t;
        q_final = spline_next->getPosition(t);
        rho_obs = max_obs_vel * (t_iter - t_init);
        rho_robot = 0;
        delta_q = (q_final - q_init->getCoord()).cwiseAbs();
        for (size_t i = 0; i < ss->robot->getNumDOFs(); i++)
            rho_robot += q_init->getEnclosingRadii()->col(i+1).dot(delta_q);
        
        if (rho_robot + rho_obs >= q_init->getDistance())    // Possible collision
        {
            // std::cout << "********** Possible collision ********** \n";
            q_init = ss->getNewState(q_final);
            q_init->setDistance(computeDistanceUnderestimation(q_init, q_current->getNearestPoints(), t_iter));
            ss->robot->computeEnclosingRadii(q_init);
            t_init = t_iter;

            if (q_init->getDistance() <= 0)
            {
                // std::cout << "\t Spline is NOT guaranteed collision-free! \n";
                return true;
            }
        }
        // else std::cout << "\t OK! rho_robot + rho_obs < q_init->getDistance() \n";
    }
    
    // auto t_elapsed { std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start_).count() };
    // std::cout << "Elapsed time for spline checking: " << t_elapsed << " [us] \n";
    return false;
}

/// @brief Compute an underestimation of distance-to-obstacles 'd_c', i.e., a distance-to-planes, for each robot's link,
/// when the robot takes a configuration 'q'. 
/// Planes approximate obstacles, and are generated according to 'nearest_points'.
/// Each plane is moved at the maximal obstacle velocity 'ss->env->getObject(j)->getMaxVel()' towards the robot 
/// during the time interval 'delta_t'.
/// @param q Configuration of the robot.
/// @param nearest_points Nearest points between the robot and obstacles.
/// @param delta_t Time interval when planes are moving towards the robot.
/// @return Underestimation of distance-to-obstacles.
/// Note that if 'd_c' is negative, it means that one or more robot's links are penetrating through the plane,
/// or they are located on the other side of the plane.
float planning::drbt::Splines::computeDistanceUnderestimation(const std::shared_ptr<base::State> q, 
	const std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points, float delta_t)
{
	float d_c_temp {};
    float d_c { INFINITY };
    Eigen::Vector3f R {};		// Robot's nearest point
	Eigen::Vector3f O {};    	// Obstacle's nearest point
    Eigen::Vector3f delta_RO {};
	std::shared_ptr<Eigen::MatrixXf> skeleton { ss->robot->computeSkeleton(q) };
    
    for (size_t i = 0; i < ss->robot->getNumLinks(); i++)
    {
        for (size_t j = 0; j < ss->env->getNumObjects(); j++)
        {
            O = nearest_points->at(j).col(i).tail(3);
			if (O.norm() == INFINITY)
                continue;
			
            R = nearest_points->at(j).col(i).head(3);

            // Move both points, R and O, to maintain a normal plane vector 'R - O'
            delta_RO = (R - O).normalized() * ss->env->getObject(j)->getMaxVel() * delta_t;
            R += delta_RO;
            O += delta_RO;

            d_c_temp = std::min((skeleton->col(i) - O).dot((R - O).normalized()), 
                                (skeleton->col(i+1) - O).dot((R - O).normalized())) 
                                - ss->robot->getCapsuleRadius(i);
            if (d_c_temp < 0)
                return 0;

            d_c = std::min(d_c, d_c_temp);

            // std::cout << "(i, j) = " << "(" << i << ", " << j << "):" << std::endl;
            // std::cout << "Robot nearest point:    " << R.transpose() << std::endl;
            // std::cout << "Obstacle nearest point: " << O.transpose() << std::endl;
            // std::cout << "d_c: " << d_c_profile[i] << std::endl;
		}
    }

	return d_c;
}

void planning::drbt::Splines::recordTrajectory(bool spline_computed)
{
    // This function is just for debugging. You can set a desired path for the file to be saved.
    std::ofstream output_file {};
    output_file.open("/home/spear/xarm6-etf-lab/src/etf_modules/RPMPLv2/data/planar_2dof/scenario_real_time/visualize_trajectory.log", 
        std::ofstream::app);
    
    output_file << "q_current - q_target \n";
    if (spline_computed)
    {
        output_file << q_current->getCoord().transpose() << "\n";
        output_file << q_target->getCoord().transpose() << "\n";
    }
    else
        output_file << INFINITY << "\n";

    output_file << "q_spline: \n";
    for (float t = spline_next->getTimeCurrent(); t < spline_next->getTimeFinal(); t += 0.001)
        output_file << spline_next->getPosition(t).transpose() << "\n";
    output_file << spline_next->getPosition(spline_next->getTimeFinal()).transpose() << "\n";

    output_file << "q_spline (realized): \n";
    for (float t = spline_current->getTimeBegin(); t < spline_current->getTimeCurrent(); t += 0.001)
        output_file << spline_current->getPosition(t).transpose() << "\n";    
    for (float t = spline_next->getTimeCurrent(); t < spline_next->getTimeEnd(); t += 0.001)
        output_file << spline_next->getPosition(t).transpose() << "\n";
    output_file << spline_next->getPosition(spline_next->getTimeEnd()).transpose() << "\n";
    output_file << "--------------------------------------------------------------------\n";
}
