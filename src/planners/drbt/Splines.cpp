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
    max_num_iter_spline_next = all_robot_vel_same ? 
        std::ceil(std::log2(2 * ss->robot->getMaxVel(0) / SplinesConfig::FINAL_VELOCITY_STEP)) :
        std::ceil(std::log2(2 * ss->robot->getMaxVel().maxCoeff() / SplinesConfig::FINAL_VELOCITY_STEP));
}

bool planning::drbt::Splines::computeSplineNext(Eigen::VectorXf &current_pos, Eigen::VectorXf &current_vel, Eigen::VectorXf &current_acc, 
    float t_iter_remain, bool non_zero_final_vel)
{
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
        
        while (num_iter++ < max_num_iter_spline_next)
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


bool planning::drbt::Splines::computeSplineSafe(Eigen::VectorXf &current_pos, Eigen::VectorXf &current_vel, Eigen::VectorXf &current_acc, 
    float t_iter_remain)
{
    std::shared_ptr<planning::trajectory::Spline> spline_new 
        { std::make_shared<planning::trajectory::Spline5>(ss->robot, current_pos, current_vel, current_acc) };
    std::shared_ptr<planning::trajectory::Spline> spline_emergency 
        { std::make_shared<planning::trajectory::Spline4>(ss->robot, current_pos, current_vel, current_acc) };
    std::shared_ptr<planning::trajectory::Spline> spline_emergency_new { nullptr };
    
    float rho_robot {};
    float rho_robot_emergency {};
    float rho_obs {};
    bool is_safe {};
    bool spline_computed { false };
    bool spline_emergency_computed { false };
    float t_max { t_iter_remain + DRGBTConfig::MAX_TIME_TASK1 };
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
    
    while (num_iter++ < max_num_iter)
    {
        is_safe = false;
        // std::cout << "Num. iter. " << num_iter << "\n";
        // std::cout << "q_final: " << q_final.transpose() << "\n";

        if (spline_new->compute(q_final))
        {
            rho_obs = max_obs_vel * spline_new->getTimeFinal();
            if (rho_obs < q_current->getDistance())
            {
                rho_robot = computeRho(q_final);
                if (rho_obs + rho_robot < q_current->getDistance())
                {
                    spline_emergency_computed = false;
                    is_safe = true;
                }
                else if (spline_new->getTimeFinal() > t_max)
                {
                    spline_emergency_new = std::make_shared<planning::trajectory::Spline4>
                    (
                        ss->robot,
                        spline_new->getPosition(t_max),
                        spline_new->getVelocity(t_max),
                        spline_new->getAcceleration(t_max)
                    );

                    if (spline_emergency_new->compute())
                    {
                        rho_obs = max_obs_vel * (t_max + spline_emergency_new->getTimeFinal());
                        rho_robot_emergency = computeRho(spline_emergency_new->getPosition(INFINITY));
                        if (rho_obs + rho_robot + rho_robot_emergency < q_current->getDistance())
                        {
                            *spline_emergency = *spline_emergency_new;
                            spline_emergency_computed = true;
                            is_safe = true;
                        }
                    }
                }
            }
        }

        // std::cout << "\trho_obs: " << rho_obs << " [m]\t";
        // std::cout << "\trho_robot: " << rho_robot << " [m]\t";
        // std::cout << "\trho_robot_emergency: " << rho_robot_emergency << " [m]\t";
        // std::cout << "\trho_sum: " << rho_obs + rho_robot + rho_robot_emergency << " [m]\n";
        // rho_robot_emergency = 0; rho_robot = 0;     // Reset only for console output
        
        if (is_safe)
        {
            // std::cout << "\tRobot is safe! \n";
            *spline_next = *spline_new;
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

    if (spline_computed)
    {
        if (!spline_emergency_computed) std::cout << "Provjera samo obicnog splinea \n";

        std::shared_ptr<base::State> q_init { ss->getNewState(q_current) };
        if (!spline_emergency_computed)
            spline_computed = !checkSplineCollision(spline_next, DRGBTConfig::MAX_ITER_TIME - t_iter_remain, spline_next->getTimeFinal(), q_init);
        
        if (!spline_computed)   // This occurs rarely, yet it is worth considering.
        {
            std::cout << "Nermin - Promijenila se odluka! \n";
            spline_emergency = std::make_shared<planning::trajectory::Spline4>
            (
                ss->robot,
                spline_next->getPosition(t_max),
                spline_next->getVelocity(t_max),
                spline_next->getAcceleration(t_max)
            );

            if (spline_emergency->compute())
                spline_emergency_computed = true;
        }

        if (spline_emergency_computed)
        {
            std::cout << "Provjera obicnog i emergency splinea \n";
            q_init = ss->getNewState(q_current);
            spline_computed = !(checkSplineCollision(spline_next, DRGBTConfig::MAX_ITER_TIME - t_iter_remain, t_max, q_init) ||
                checkSplineCollision(spline_emergency, DRGBTConfig::MAX_ITER_TIME - t_iter_remain + t_max, spline_emergency->getTimeFinal(), q_init));
        }
    }

    if (!spline_computed)
    {
        spline_emergency = std::make_shared<planning::trajectory::Spline4>(ss->robot, current_pos, current_vel, current_acc);        
        if (spline_emergency->compute())
        {
            rho_obs = max_obs_vel * spline_emergency->getTimeFinal();
            rho_robot_emergency = computeRho(spline_emergency->getPosition(INFINITY));

            // std::cout << "\trho_obs: " << rho_obs << " [m]\t";
            // std::cout << "\trho_robot_emergency: " << rho_robot_emergency << " [m]\t";
            // std::cout << "\trho_sum: " << rho_obs + rho_robot_emergency << " [m]\n";
            if (rho_obs + rho_robot_emergency < q_current->getDistance())
            {
                // std::cout << "\tRobot is safe! \n";
                std::cout << "Provjera emergency splinea \n";
                std::shared_ptr<base::State> q_init { ss->getNewState(q_current) };
                if (!checkSplineCollision(spline_emergency, DRGBTConfig::MAX_ITER_TIME - t_iter_remain, spline_emergency->getTimeFinal(), q_init))
                {
                    spline_next = spline_emergency;
                    spline_computed = true;
                }
            }
            // else std::cout << "\tRobot is NOT safe! \n";
        }
    }

    return spline_computed;
}

/// @brief Check whether 'spline' is collision-free during the spline time interval [0, 't_max']
/// @param spline Spline that is checked on collision.
/// @param t_offset Elapsed time from the beginning of iteration to a time instance when 'spline' is starting.
/// @param t_max Maximal spline time from 'spline' to determine when to stop with the collision checking.
/// @param q_init Initial configuration from where bur spines are generated.
/// @return True if the collision occurs. False if not.
/// @note 'q_init' must have a distance-to-obstacles or its underestimation!
bool planning::drbt::Splines::checkSplineCollision(std::shared_ptr<planning::trajectory::Spline> spline, float t_offset, float t_max, 
    std::shared_ptr<base::State> &q_init)
{
    std::cout << "Inside checkSplineCollision ............................................ \n";
    std::chrono::steady_clock::time_point time_start_ { std::chrono::steady_clock::now() };
    if (t_max < RealVectorSpaceConfig::EQUALITY_THRESHOLD)
        return false;
    
    float delta_t { SplinesConfig::TIME_STEP_COLLISION_CHECK };
    size_t num_iter = std::ceil(t_max / delta_t);
    delta_t = t_max / num_iter;
    float rho_robot {};
    float rho_obs {};
    float t_init { 0 };
    Eigen::VectorXf q_final {};
    
    std::cout << "spline: \n" << spline;
    std::cout << "num_iter: " << num_iter << "\n";
    std::cout << "delta_t:  " << delta_t << "\n";
    std::cout << "t_offset: " << t_offset << "\n";
    std::cout << "t_max:    " << t_max << "\n";

    for (float t = delta_t; t <= t_max + RealVectorSpaceConfig::EQUALITY_THRESHOLD; t += delta_t)
    {
        t_offset += delta_t;
        q_final = spline->getPosition(t);
        rho_obs = max_obs_vel * (t_offset - t_init);
        rho_robot = 0;
        for (size_t i = 0; i < ss->robot->getNumDOFs(); i++)
            rho_robot += q_init->getEnclosingRadii()->col(i+1).dot((q_final - q_init->getCoord()).cwiseAbs());
        
        std::cout << "Considering t: " << t << " ------------------------ \n";
        std::cout << "\t q_init:  " << q_init->getCoord().transpose() << "\n";
        std::cout << "\t q_final: " << q_final.transpose() << "\n";
        std::cout << "\t t_offset: " << t_offset << "\n";
        std::cout << "\t t_init:   " << t_init << "\n";
        std::cout << "\t rho_robot: " << rho_robot << "\t";
        std::cout << "\t rho_obs:   " << rho_obs << "\t";
        std::cout << "\t rho_robot + rho_obs:   " << rho_robot + rho_obs << "\t";
        std::cout << "\t q_init->getDistance(): " << q_init->getDistance() << "\n";
        if (rho_robot + rho_obs >= q_init->getDistance())    // Possible collision
        {
            std::cout << "********** Possible collision ********** \n";
            q_init = ss->getNewState(spline->getPosition(t));
            q_init->setDistance(computeDistanceUnderestimation(q_init, q_current->getNearestPoints(), t_offset));
            ss->robot->computeEnclosingRadii(q_init);
            t_init = t_offset;

            std::cout << "\t q_init->getDistance(): " << q_init->getDistance() << "\n";
            if (q_init->getDistance() <= 0)
            {
                std::cout << "\t Spline is NOT guaranteed collision-free! \n";
                return true;
            }
        }
        else std::cout << "\t OK! rho_robot + rho_obs < q_init->getDistance() \n";
    }
    
    auto t_elapsed { std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start_).count() };
    std::cout << "Elapsed time for spline checking: " << t_elapsed << " [us] \n";
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
                return d_c_temp;

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
