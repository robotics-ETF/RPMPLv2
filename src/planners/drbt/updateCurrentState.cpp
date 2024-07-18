#include "DRGBT.h"

/// @brief Update a current state of the robot using 'spline_current'.
/// Compute a new spline 'spline_next', or remain 'spline_current'.
/// Determine a new target state 'q_target' (a new desired current state) of the robot.
/// Move 'q_current' towards 'q_target' while following 'spline_next'.
/// 'q_current' will be updated to a robot position from the end of iteration.
/// @param measure_time If true, elapsed time when computing a spline will be exactly measured. 
/// If false, elapsed time will be computed (default: false).
/// @return Remaining time in [s] after which the new spline 'spline_next' will become active.
/// @note The new spline will be computed in a way that all constraints on robot's maximal velocity, 
/// acceleration and jerk are surely always satisfied.
/// @note 'measure_time' should always be false when simulation pacing is used, since then a time measuring will not be correct! 
/// In such case, it is assumed that user was previously set 'measure_time' to a correct value.
float planning::drbt::DRGBT::updateCurrentState(bool measure_time)
{
    spline_current = spline_next;
    q_previous = q_current;
    if (status != base::State::Status::Trapped)
        status = base::State::Status::Advanced;     // by default

    float t_spline_max { SplinesConfig::MAX_TIME_COMPUTE };
    float t_iter { getElapsedTime(time_iter_start) };
    if (DRGBTConfig::MAX_TIME_TASK1 - t_iter < t_spline_max)
        t_spline_max = DRGBTConfig::MAX_TIME_TASK1 - t_iter;
    
    float t_iter_remain { DRGBTConfig::MAX_ITER_TIME - t_iter - t_spline_max };
    float t_spline_current { measure_time ? 
                             spline_current->getTimeCurrent(true) + t_spline_max :
                             spline_current->getTimeEnd() + t_iter + t_spline_max };

    spline_current->setTimeBegin(spline_current->getTimeEnd());
    spline_current->setTimeCurrent(t_spline_current);

    // std::cout << "Iter. time:        " << t_iter * 1000 << " [ms] \n";
    // std::cout << "Max. spline time:  " << t_spline_max * 1000 << " [ms] \n";
    // std::cout << "Remain. time:      " << t_iter_remain * 1000 << " [ms] \n";
    // std::cout << "Begin spline time: " << spline_current->getTimeBegin() * 1000 << " [ms] \n";
    // std::cout << "Curr. spline time: " << t_spline_current * 1000 << " [ms] \n";
    // ----------------------------------------------------------------------------------------- //
    
    bool spline_computed { false };
    Eigen::VectorXf current_pos { spline_current->getPosition(t_spline_current) };
    Eigen::VectorXf current_vel { spline_current->getVelocity(t_spline_current) };
    Eigen::VectorXf current_acc { spline_current->getAcceleration(t_spline_current) };
    std::vector<std::shared_ptr<planning::drbt::HorizonState>> visited_states { q_next };
    spline_next = std::make_shared<planning::trajectory::Spline5>(ss->robot, current_pos, current_vel, current_acc);

    // std::cout << "Curr. pos: " << current_pos.transpose() << "\n";
    // std::cout << "Curr. vel: " << current_vel.transpose() << "\n";
    // std::cout << "Curr. acc: " << current_acc.transpose() << "\n";

    computeTargetState(DRGBTConfig::MAX_ITER_TIME + DRGBTConfig::MAX_TIME_TASK1);
    if (status != base::State::Status::Trapped && 
        q_next->getIsReached() && 
        ss->isEqual(q_target, q_next->getStateReached()))
    {
        status = base::State::Status::Reached;  // 'q_next->getState()' must be reached, and not only 'q_next->getStateReached()'
        if (q_next->getStatus() != planning::drbt::HorizonState::Status::Goal && changeNextState(visited_states))
            computeTargetState(DRGBTConfig::MAX_ITER_TIME + DRGBTConfig::MAX_TIME_TASK1);
    }

    do
    {
        // std::cout << "q_target: " << q_target->getCoord().transpose() << "\t idx: " << q_next->getIndex() << "\n";        
        if (spline_current->isFinalConf(q_target->getCoord()))  // Spline to such 'q_target' already exists!
            break;

        if (DRGBTConfig::GUARANTEED_SAFE_MOTION)
            spline_computed = computeSplineSafe(current_pos, current_vel, current_acc, t_iter_remain);
        else
            spline_computed = computeSplineNext(current_pos, current_vel, current_acc, t_iter_remain);
    }
    while (!spline_computed && 
            getElapsedTime(time_iter_start) - t_iter < t_spline_max - SplinesConfig::MAX_TIME_PUBLISH * measure_time && 
            changeNextState(visited_states) && 
            computeTargetState(DRGBTConfig::MAX_ITER_TIME + DRGBTConfig::MAX_TIME_TASK1));
    // std::cout << "Elapsed time for spline computing: " << (getElapsedTime(time_iter_start) - t_iter) * 1e3 << " [ms] \n";

    if (spline_computed)
    {
        // std::cout << "New spline is computed! \n";
        spline_current->setTimeEnd(t_spline_current);
        spline_next->setTimeEnd(t_iter_remain);
    }
    else
    {
        // std::cout << "Continuing with the previous spline! \n";
        spline_next = spline_current;
        spline_next->setTimeEnd(t_spline_current + t_iter_remain);
    }
    
    // recordTrajectory(spline_computed);

    q_current = ss->getNewState(spline_next->getPosition(spline_next->getTimeEnd()));   // Current robot position at the end of iteration

    // std::cout << "q_current:     " << q_current << "\n";
    // std::cout << "q_target:      " << q_target << "\n";
    // std::cout << "Spline next: \n" << spline_next << "\n";
    // std::cout << "Status: " << (status == base::State::Status::Advanced ? "Advanced" : "")
    //                         << (status == base::State::Status::Trapped  ? "Trapped"  : "")
    //                         << (status == base::State::Status::Reached  ? "Reached"  : "") << "\n";

    return t_spline_max - (getElapsedTime(time_iter_start) - t_iter);
}

/// @brief Choose the best state from the horizon so that it does not belong to 'visited_states'.
/// @param visited_states Set of visited states.
/// @return Success of a change.
bool planning::drbt::DRGBT::changeNextState(std::vector<std::shared_ptr<planning::drbt::HorizonState>> &visited_states)
{
    // std::cout << "Change of q_next is required! \n";
    std::shared_ptr<planning::drbt::HorizonState> q_new { nullptr };
    float weight_max { 0 };
    bool visited { false };

    for (std::shared_ptr<planning::drbt::HorizonState> q : horizon)
    {
        if (q->getWeight() < DRGBTConfig::TRESHOLD_WEIGHT)
            continue;

        visited = false;
        for (std::shared_ptr<planning::drbt::HorizonState> q_visited : visited_states)
        {
            if (q == q_visited)
            {
                visited = true;
                break;
            }
        }

        if (!visited && q->getWeight() > weight_max)
        {
            q_new = q;
            weight_max = q->getWeight();
        }
    }

    if (q_new != nullptr)
    {
        visited_states.emplace_back(q_new);
        q_next = q_new;
        return true;
    }

    return false;
}

bool planning::drbt::DRGBT::computeSplineNext(Eigen::VectorXf &current_pos, Eigen::VectorXf &current_vel, Eigen::VectorXf &current_acc, float t_iter_remain)
{
    bool spline_computed { false };

    if (SplinesConfig::IS_FINAL_VELOCITY_ZERO)
        spline_computed = spline_next->compute(q_target->getCoord());
    else
    {
        std::shared_ptr<planning::trajectory::Spline> spline_new 
            { std::make_shared<planning::trajectory::Spline5>(ss->robot, current_pos, current_vel, current_acc) };

        if (q_next->getIsReached() && q_next->getIndex() != -1 && !ss->isEqual(q_target, q_goal))
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

                if (spline_new->compute(q_target->getCoord(), q_final_dot)) 
                {
                    *spline_next = *spline_new;
                    q_final_dot_min = q_final_dot;
                    spline_computed = true;
                }
                else
                    q_final_dot_max = q_final_dot;
            }
        }

        Eigen::VectorXf new_current_pos    // Possible current position at the end of iteration
        {
            spline_computed ? 
            spline_next->getPosition(t_iter_remain) : 
            spline_current->getPosition(spline_current->getTimeCurrent() + t_iter_remain) 
        };

        // If spline was not computed or robot is getting away from 'new_current_pos'
        if (!spline_computed || 
            (new_current_pos - q_target->getCoord()).norm() > (current_pos - q_target->getCoord()).norm())
        {
            spline_computed = spline_new->compute(q_target->getCoord());
            if (spline_computed)
            {
                spline_next = spline_new;
                // std::cout << "\t Spline computed with ZERO final velocity. \n";
            }
        }
        // else std::cout << "\t Spline computed with NON-ZERO final velocity. \n";
    }

    return spline_computed;
}

bool planning::drbt::DRGBT::computeSplineSafe(Eigen::VectorXf &current_pos, Eigen::VectorXf &current_vel, Eigen::VectorXf &current_acc, float t_iter_remain)
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
            if (rho_obs < d_c)
            {
                rho_robot = computeRho(q_final);
                if (rho_obs + rho_robot < d_c)
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
                        if (rho_obs + rho_robot + rho_robot_emergency < d_c)
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
        if (spline_emergency_computed) std::cout << "Nermin - provjera obicnog i emergency splinea \n";
        else std::cout << "Nermin - provjera samo obicnog splinea \n";

        std::shared_ptr<base::State> q_init { ss->getNewState(q_current) };
        if (!spline_emergency_computed)
            spline_computed = !checkSplineCollision(spline_next, DRGBTConfig::MAX_ITER_TIME - t_iter_remain, spline_next->getTimeFinal(), q_init);
        else
        {
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
            if (rho_obs + rho_robot_emergency < d_c)
            {
                // std::cout << "\tRobot is safe! \n";
                std::cout << "Nermin - provjera emergency splinea \n";
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
bool planning::drbt::DRGBT::checkSplineCollision(std::shared_ptr<planning::trajectory::Spline> spline, float t_offset, float t_max, 
    std::shared_ptr<base::State> &q_init)
{
    std::cout << "Inside checkSplineCollision ............................................ \n";
    if (t_max < RealVectorSpaceConfig::EQUALITY_THRESHOLD)
        return false;
    
    float delta_t { 0.01 };    // Zadavati u config fileu.
    size_t num_iter = std::ceil(t_max / delta_t);
    delta_t = t_max / num_iter;
    float rho {};
    float t_init { 0 };
    Eigen::VectorXf q_final {};
    
    std::cout << "spline: \n" << spline << "\n";
    std::cout << "num_iter: " << num_iter << "\n";
    std::cout << "delta_t:  " << delta_t << "\n";
    std::cout << "t_offset: " << t_offset << "\n";
    std::cout << "t_max:    " << t_max << "\n";

    for (float t = delta_t; t <= t_max + RealVectorSpaceConfig::EQUALITY_THRESHOLD; t += delta_t)
    {
        t_offset += delta_t;
        q_final = spline->getPosition(t);
        std::cout << "Considering t: " << t << " ------------------------ \n";
        std::cout << "q_init:  " << q_init->getCoord().transpose() << "\n";
        std::cout << "q_final: " << q_final.transpose() << "\n";

        rho = 0;
        std::cout << "Radii: " << *q_init->getEnclosingRadii() << "\n";
        for (size_t i = 0; i < ss->robot->getNumDOFs(); i++)
            rho += q_init->getEnclosingRadii()->col(i+1).dot((q_final - q_init->getCoord()).cwiseAbs());
        
        std::cout << "rho:                   " << rho << "\n";
        std::cout << "q_init->getDistance(): " << q_init->getDistance() - max_obs_vel * (t_offset - t_init) << "\n";
        if (rho < q_init->getDistance() - max_obs_vel * (t_offset - t_init))
            std::cout << "OK! rho < q_init->getDistance() \n";
        else    // Possible collision
        {
            std::cout << "Possible collision \n";
            q_init = ss->getNewState(spline->getPosition(t));
            q_init->setDistance(computeDistanceUnderestimation(q_init, q_current->getNearestPoints(), t_offset));
            t_init = t_offset;
            ss->robot->computeEnclosingRadii(q_init);
            std::cout << "q_init->getDistance(): " << q_init->getDistance() << "\n";
            if (q_init->getDistance() <= 0)
            {
                std::cout << "Spline is NOT collision-free! \n";
                return true;
            }
        }
    }

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
float planning::drbt::DRGBT::computeDistanceUnderestimation(const std::shared_ptr<base::State> q, 
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

/// @brief Compute a target configuration 'q_target' from the edge [q_current - q_next_reached], 
/// such that it can be reached within time 'time', while considering robot maximal velocity.
/// @return Success of computing 'q_target'. 
/// If 'q_current' and 'q_next->getStateReached()' are equal, false is returned. Otherwise, true is returned.
bool planning::drbt::DRGBT::computeTargetState(float time)
{
    q_target = ss->getNewState(q_next->getCoordReached());
    if (ss->isEqual(q_current, q_next->getStateReached()))
        return false;

    if (all_robot_vel_same)
    {
        float max_edge_length_ { ss->robot->getMaxVel(0) * time };
        if (ss->getNorm(q_current, q_target) > max_edge_length_)
            q_target = ss->pruneEdge2(q_current, q_target, max_edge_length_);
    }
    else
    {
        std::vector<std::pair<float, float>> limits {};
        for (size_t i = 0; i < ss->num_dimensions; i++)
        {
            limits.emplace_back(std::pair<float, float>
               (q_current->getCoord(i) - ss->robot->getMaxVel(i) * time, 
                q_current->getCoord(i) + ss->robot->getMaxVel(i) * time));
        }
        q_target = ss->pruneEdge(q_current, q_target, limits);
    }

    q_target->setParent(q_current);
    return true;
}

/// @brief Update a current state 'q_current' to become 'q_target'.
/// Determine a new target state 'q_target' (a new desired current state) of the robot.
/// Move 'q_current' towards 'q_next' for an advancing step size determined as 
/// ss->robot->getMaxVel(i) * DRGBTConfig::MAX_ITER_TIME, i.d., using the maximal robot's velocity.
/// 'q_current' will be updated to a robot position from the end of iteration.
void planning::drbt::DRGBT::updateCurrentState()
{
    q_previous = q_current;
    if (status == base::State::Status::Trapped)     // Current robot position will not be updated! 
    {                                               // We must wait for successful replanning to change 'status' to 'Reached'
        // std::cout << "Status: Trapped! \n";
        return;
    }

    computeTargetState();
    q_current = q_target;   // Current robot position at the end of iteration

    if (ss->isEqual(q_current, q_next->getState()))
        status = base::State::Status::Reached;      // 'q_next->getState()' must be reached, and not only 'q_next->getStateReached()'
    else
        status = base::State::Status::Advanced;

    // std::cout << "q_current: " << q_current << "\n";
    // std::cout << "q_target:  " << q_target << "\n";
    // std::cout << "Status: " << (status == base::State::Status::Advanced ? "Advanced" : "")
    //                         << (status == base::State::Status::Reached  ? "Reached"  : "") << "\n";
}

void planning::drbt::DRGBT::recordTrajectory(bool spline_computed)
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
