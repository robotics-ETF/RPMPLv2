#include "Splines.h"

planning::trajectory::Trajectory::Trajectory(const std::shared_ptr<base::StateSpace> &ss_)
{
    ss = ss_;
    q_current = nullptr;
    max_iter_time = 0;
    max_remaining_iter_time = INFINITY;
    max_obs_vel = 0;
    spline_current = nullptr;
    spline_next = nullptr;
    composite_spline = nullptr;
    setParams();
}

planning::trajectory::Trajectory::Trajectory(const std::shared_ptr<base::StateSpace> &ss_, 
    const std::shared_ptr<base::State> &q_current_, float max_iter_time_)
{
    ss = ss_;
    q_current = q_current_;
    max_iter_time = max_iter_time_;
    max_remaining_iter_time = 0;

    max_obs_vel = 0;
    for (size_t i = 0; i < ss->env->getNumObjects(); i++)
    {
        if (ss->env->getObject(i)->getMaxVel() > max_obs_vel)
            max_obs_vel = ss->env->getObject(i)->getMaxVel();
    }

    spline_current = std::make_shared<planning::trajectory::Spline5>(ss->robot, q_current->getCoord());
    spline_next = spline_current;
    composite_spline = nullptr;
    setParams();
}

void planning::trajectory::Trajectory::setParams()
{
    all_robot_vel_same = true;
    for (size_t i = 1; i < ss->num_dimensions; i++)
    {
        if (std::abs(ss->robot->getMaxVel(i) - ss->robot->getMaxVel(i-1)) > RealVectorSpaceConfig::EQUALITY_THRESHOLD)
        {
            all_robot_vel_same = false;
            break;
        }
    }

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
bool planning::trajectory::Trajectory::computeRegular(const Eigen::VectorXf &current_pos, const Eigen::VectorXf &current_vel, 
    const Eigen::VectorXf &current_acc, float t_iter_remain, float t_max, bool non_zero_final_vel)
{
    std::chrono::steady_clock::time_point time_start_ { std::chrono::steady_clock::now() };
    spline_next = std::make_shared<planning::trajectory::Spline5>(ss->robot, current_pos, current_vel, current_acc);
    std::shared_ptr<planning::trajectory::Spline> spline_next_new 
        { std::make_shared<planning::trajectory::Spline5>(ss->robot, current_pos, current_vel, current_acc) };
    bool spline_computed { false };

    if (non_zero_final_vel)
    {
        float num_iter { 0 };
        float delta_t_max { ((q_target->getCoord() - q_current->getCoord()).cwiseQuotient(ss->robot->getMaxVel())).cwiseAbs().maxCoeff() };
        Eigen::VectorXf q_final_dot_max { (q_target->getCoord() - q_current->getCoord()) / delta_t_max };
        Eigen::VectorXf q_final_dot_min { Eigen::VectorXf::Zero(ss->num_dimensions) };
        Eigen::VectorXf q_final_dot { q_final_dot_max };
        
        while (num_iter++ < max_num_iter_spline_regular &&
               std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start_).count() < t_max * 1e6)
        {
            if (spline_next_new->compute(q_target->getCoord(), q_final_dot)) 
            {
                *spline_next = *spline_next_new;
                q_final_dot_min = q_final_dot;
                spline_computed = true;
                if (num_iter == 1)
                    break;
            }
            else
                q_final_dot_max = q_final_dot;

            q_final_dot = (q_final_dot_max + q_final_dot_min) / 2;
            // std::cout << "Num. iter. " << num_iter << "\t q_final_dot: " << q_final_dot.transpose() << "\n";
        }
    }

    Eigen::VectorXf new_current_pos {    // Possible current position at the end of iteration
        spline_computed ? 
        spline_next->getPosition(t_iter_remain) : 
        spline_current->getPosition(spline_current->getTimeCurrent() + t_iter_remain)
    };
    
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
bool planning::trajectory::Trajectory::computeSafe(const Eigen::VectorXf &current_pos, const Eigen::VectorXf &current_vel, 
    const Eigen::VectorXf &current_acc, float t_iter_remain, float t_max)
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
    float t_iter { max_iter_time - t_iter_remain };
    float t_spline_max { t_iter_remain + (max_iter_time - max_remaining_iter_time) };
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
        spline_computed = !checkCollision(q_current, t_iter);
    }

    return spline_computed;
}

/// @brief Check whether 'spline_next' is collision-free during the spline time interval [0, 'spline_next->getTimeFinal()']
/// @param q_init Initial configuration from where bur spines are generated.
/// @param t_iter Elapsed time from the beginning of iteration to a time instance when 'spline' is starting.
/// @return True if the collision occurs. False if not.
/// @note 'q_init' must have a distance-to-obstacles or its underestimation!
bool planning::trajectory::Trajectory::checkCollision(std::shared_ptr<base::State> q_init, float t_iter)
{
    // std::chrono::steady_clock::time_point time_start_ { std::chrono::steady_clock::now() };    
    float delta_t { SplinesConfig::TIME_STEP };
    size_t num_iter = std::ceil(spline_next->getTimeFinal() / delta_t);
    delta_t = spline_next->getTimeFinal() / num_iter;
    float rho_robot {};
    float rho_obs {};
    float t_init { 0 };
    std::shared_ptr<base::State> q_final { nullptr };
	Eigen::VectorXf delta_q {};

    for (float t = delta_t; t <= spline_next->getTimeFinal() + RealVectorSpaceConfig::EQUALITY_THRESHOLD; t += delta_t)
    {
        // std::cout << "Considering t: " << t << "\n";
        q_final = ss->getNewState(spline_next->getPosition(t));
        if (ss->robot->checkSelfCollision(q_final))
            return true;
        
        t_iter += delta_t;
        rho_obs = max_obs_vel * (t_iter - t_init);
        delta_q = (q_final->getCoord() - q_init->getCoord()).cwiseAbs();
        ss->robot->computeEnclosingRadii(q_init);
        for (size_t i = 0; i < ss->robot->getNumDOFs(); i++)
        {
            rho_robot = q_init->getEnclosingRadii()->col(i+1).dot(delta_q);
            if (rho_robot + rho_obs >= q_init->getDistanceProfile(i))    // Possible collision
            {
                // std::cout << "********** Possible collision ********** \n";
                q_init = q_final;
                computeDistanceUnderestimation(q_init, q_current->getNearestPoints(), t_iter);
                t_init = t_iter;

                if (q_init->getDistance() <= 0)
                {
                    // std::cout << "\t Spline is NOT guaranteed collision-free! \n";
                    return true;
                }
                break;
            }
            // else std::cout << "\t OK! rho_robot + rho_obs < q_init->getDistance() \n";
        }
    }
    
    // auto t_elapsed { std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start_).count() };
    // std::cout << "Elapsed time for spline checking: " << t_elapsed << " [us] \n";
    return false;
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
float planning::trajectory::Trajectory::computeDistanceUnderestimation(const std::shared_ptr<base::State> q, 
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

            // Move both points, R and O, to maintain a normal plane vector 'R - O'
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

/// @brief A method (v1) to convert a path 'path' to a corresponding trajectory.
/// Converting this path to trajectory (i.e., assigning time instances to these points) will be automatically done by this function.
/// This is done by creating a sequence of quintic splines in a way that all constraints on robot's maximal velocity, 
/// acceleration and jerk are surely always satisfied.
/// @param path Path containing all points that robot should visit.
/// @note Be careful since the distance between each two adjacent points from 'path' should not be too long! 
/// The robot motion between them is generally not a straight line in C-space. 
/// Consider using 'preprocessPath' function from 'base::RealVectorSpace' class before using this function.
void planning::trajectory::Trajectory::path2traj_v1(const std::vector<std::shared_ptr<base::State>> &path)
{
    std::vector<std::shared_ptr<planning::trajectory::Spline>> subsplines {};

    spline_current = std::make_shared<planning::trajectory::Spline5>
    (
        ss->robot, 
        path.front()->getCoord(), 
        Eigen::VectorXf::Zero(ss->num_dimensions), 
        Eigen::VectorXf::Zero(ss->num_dimensions)
    );
    spline_current->compute(path[1]->getCoord());
    
    float t {}, t_max {};
    bool spline_computed { false };
    size_t num {};
    const size_t max_num_iter { 5 };
    
    for (size_t i = 2; i < path.size(); i++)
    {
        spline_computed = false;
        num = 0;
        t = 0;
        t_max = spline_current->getTimeFinal();
        // std::cout << "i: " << i << " ---------------------------\n";
        // std::cout << "t_max: " << t_max << " [s] \n";

        while (!spline_computed && num++ < max_num_iter)
        {
            t = (num < max_num_iter) ?
                (t + t_max) / 2 :
                t_max;              // Solution surely exists, and 'spline_computed' will become true.
            // std::cout << "t: " << t << " [s] \n";

            spline_next = std::make_shared<planning::trajectory::Spline5>
            (
                ss->robot, 
                spline_current->getPosition(t), 
                spline_current->getVelocity(t), 
                spline_current->getAcceleration(t)
            );
            spline_computed = spline_next->compute(path[i]->getCoord());

            if (spline_computed && num < max_num_iter)
            {
                q_current = ss->getNewState(spline_current->getPosition(t));
                ss->computeDistance(q_current);     // Required by 'checkCollision' function
                if (q_current->getDistance() <= 0 || checkCollision(q_current, 0))
                    spline_computed = false;
            }
        }

        spline_current->setTimeFinal(t);
        subsplines.emplace_back(spline_current);
        spline_current = spline_next;
    }

    subsplines.emplace_back(spline_current);
    composite_spline = std::make_shared<planning::trajectory::CompositeSpline>(subsplines);
}

/// @brief A method (v2) to convert a path 'path' to a corresponding trajectory.
/// Converting this path to trajectory (i.e., assigning time instances to these points) will be automatically done by this function.
/// This is done by creating a sequence of quintic splines in a way that all constraints on robot's maximal velocity, 
/// acceleration and jerk are surely always satisfied.
/// @param path Path containing all points that robot should visit.
/// @note Be careful since the distance between each two adjacent points from 'path' should not be too long! 
/// The robot motion between them is generally not a straight line in C-space. 
/// Consider using 'preprocessPath' function from 'base::RealVectorSpace' class before using this function.
void planning::trajectory::Trajectory::path2traj_v2(const std::vector<std::shared_ptr<base::State>> &path)
{
    std::vector<std::shared_ptr<planning::trajectory::Spline>> subsplines {};

    spline_current = std::make_shared<planning::trajectory::Spline5>
    (
        ss->robot, 
        path.front()->getCoord(), 
        Eigen::VectorXf::Zero(ss->num_dimensions), 
        Eigen::VectorXf::Zero(ss->num_dimensions)
    );
    spline_current->compute(path[1]->getCoord());
    
    float t {}, t_max {};
    bool spline_computed { false };
    size_t num {};
    const size_t max_num_iter { 5 };
    bool non_zero_final_vel { false };
    
    for (size_t i = 2; i < path.size(); i++)
    {
        spline_computed = false;
        num = 0;
        t = 0;
        t_max = spline_current->getTimeFinal();
        non_zero_final_vel = (i < path.size()-1) ? ss->checkLinearDependency(path[i-1], path[i], path[i+1]) : false;
        q_target = path[i];
        // std::cout << "i: " << i << " ---------------------------\n";
        // std::cout << "t_max: " << t_max << " [s] \n";

        while (!spline_computed && num++ < max_num_iter)
        {
            t = (num < max_num_iter) ?
                (t + t_max) / 2 :
                t_max;                  // Solution surely exists, and 'spline_computed' will become true.
            // std::cout << "t: " << t << " [s] \n";

            q_current = path[i-1];      // Required for the estimation of final vector velocity
            spline_computed = computeRegular(
                spline_current->getPosition(t),
                spline_current->getVelocity(t),
                spline_current->getAcceleration(t),
                max_remaining_iter_time,
                SplinesConfig::MAX_TIME_COMPUTE_REGULAR,
                non_zero_final_vel
            );
            
            if (spline_computed && num < max_num_iter)
            {
                q_current = ss->getNewState(spline_current->getPosition(t));
                ss->computeDistance(q_current);     // Required by 'checkCollision' function
                if (q_current->getDistance() <= 0 || checkCollision(q_current, 0))
                    spline_computed = false;
            }
        }

        spline_current->setTimeFinal(t);
        subsplines.emplace_back(spline_current);
        spline_current = spline_next;
    }

    subsplines.emplace_back(spline_current);
    composite_spline = std::make_shared<planning::trajectory::CompositeSpline>(subsplines);
}

/// @brief A method (v3) to convert a path 'path' to a corresponding trajectory.
/// Converting this path to trajectory (i.e., assigning time instances to these points) will be automatically done by this function.
/// This is done by creating a sequence of quintic splines in a way that all constraints on robot's maximal velocity, 
/// acceleration and jerk are surely always satisfied.
/// @param path Path containing all points that robot should visit.
/// @param must_visit Whether path points must be visited.
/// @note Be careful since the distance between each two adjacent points from 'path' should not be too long! 
/// The robot motion between them is generally not a straight line in C-space. 
/// Consider using 'preprocessPath' function from 'base::RealVectorSpace' class before using this function.
void planning::trajectory::Trajectory::path2traj_v3(const std::vector<std::shared_ptr<base::State>> &path, bool must_visit)
{
    std::vector<std::shared_ptr<planning::trajectory::Spline>> subsplines(path.size(), nullptr);
    bool spline_computed { false };
    size_t num_iter {};
    size_t max_num_iter { 5 };
    float delta_t_max {};
    Eigen::VectorXf q_final_dot_max {};
    Eigen::VectorXf q_final_dot_min {};
    Eigen::VectorXf q_final_dot {};
    Eigen::VectorXf q_final {};
    std::vector<float> vel_coeff(path.size(), 1.0);
    const float vel_coeff_const { 0.9 };
    auto time_start = std::chrono::steady_clock::now();
    float max_time { 1.0 };
    bool monotonic { true };

    subsplines.front() = std::make_shared<planning::trajectory::Spline5>
    (
        ss->robot, 
        path.front()->getCoord(), 
        Eigen::VectorXf::Zero(ss->num_dimensions), 
        Eigen::VectorXf::Zero(ss->num_dimensions)
    );
    
    for (size_t i = 1; i < path.size(); i++)
    {
        subsplines[i] = std::make_shared<planning::trajectory::Spline5>
        (
            ss->robot, 
            subsplines[i-1]->getPosition(subsplines[i-1]->getTimeFinal()), 
            subsplines[i-1]->getVelocity(subsplines[i-1]->getTimeFinal()), 
            subsplines[i-1]->getAcceleration(subsplines[i-1]->getTimeFinal())
        );

        if (i == path.size() - 1)   // Final configuration will be reached, thus final velocity and acceleration must be zero!
        {
            spline_computed = subsplines[i]->compute(path.back()->getCoord()) && subsplines[i]->checkPositionMonotonicity() != 0;
            if (!spline_computed) 
            {
                // std::cout << "Spline not computed! \n";
                vel_coeff[--i] *= vel_coeff_const;
                --i;
            }
            else
                break;
        }

        if (i > 1)
            monotonic = (!ss->checkLinearDependency(path[i-2], path[i-1], path[i])) ? false : true;
        
        if (!must_visit)
            q_final = (path[i-1]->getCoord() + path[i]->getCoord()) / 2;
        else
            q_final = path[i]->getCoord();
        
        spline_computed = false;
        num_iter = 0;
        delta_t_max = ((q_final - path[i-1]->getCoord()).cwiseQuotient(ss->robot->getMaxVel())).cwiseAbs().maxCoeff();
        q_final_dot_max = (q_final - path[i-1]->getCoord()) / delta_t_max;
        q_final_dot_min = Eigen::VectorXf::Zero(ss->num_dimensions);

        do
        {
            q_final_dot = vel_coeff[i] * (q_final_dot_max + q_final_dot_min) / 2;
            std::shared_ptr<planning::trajectory::Spline> spline_new {
                std::make_shared<planning::trajectory::Spline5>
                (
                    ss->robot, 
                    subsplines[i-1]->getPosition(subsplines[i-1]->getTimeFinal()), 
                    subsplines[i-1]->getVelocity(subsplines[i-1]->getTimeFinal()), 
                    subsplines[i-1]->getAcceleration(subsplines[i-1]->getTimeFinal())
                )
            };
            
            if (spline_new->compute(q_final, q_final_dot) && 
                ((monotonic && spline_new->checkPositionMonotonicity() != 0) || !monotonic))
            {
                *subsplines[i] = *spline_new;
                q_final_dot_min = q_final_dot;
                spline_computed = true;
            }
            else
                q_final_dot_max = q_final_dot;
        }
        while (++num_iter < max_num_iter);

        if (!spline_computed)
        {
            spline_computed = subsplines[i]->compute(q_final) && 
                              ((monotonic && subsplines[i]->checkPositionMonotonicity() != 0) || !monotonic);
            if (!spline_computed)
            {
                // std::cout << "Spline not computed! \n";
                vel_coeff[--i] *= vel_coeff_const;
                --i;
            }
            // else std::cout << "Spline computed with zero final velocity! \n";
        }

        if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_start).count() * 1e-3 > max_time)
        {
            spline_computed = false;
            break;
        }
    }

    subsplines.erase(subsplines.begin());
    if (spline_computed)
        composite_spline = std::make_shared<planning::trajectory::CompositeSpline>(subsplines);
    else
        std::cout << "Could not convert path to trajectory! \n";
        // path2traj(path);      // Add path using another method.
}

// This function is just for debugging. It operates in real-time by logging 'spline_current' and 'spline_next'. 
// You can set a desired output path for the file to be saved.
void planning::trajectory::Trajectory::recordTrajectory(bool spline_computed)
{
    std::ofstream output_file {};
    output_file.open("/home/spear/xarm6-etf-lab/src/etf_modules/RPMPLv2/data/planar_2dof/scenario_random_obstacles/temp/visualize_trajectory.log", 
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
