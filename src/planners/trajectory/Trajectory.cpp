#include "Trajectory.h"

planning::trajectory::Trajectory::Trajectory(const std::shared_ptr<base::StateSpace> &ss_) : 
    AbstractTrajectory(ss_)
{
    spline = nullptr;
}

planning::trajectory::Trajectory::Trajectory
    (const std::shared_ptr<base::StateSpace> &ss_, planning::trajectory::State current, float max_iter_time_) : 
        AbstractTrajectory(ss_, max_iter_time_)
{
    spline = std::make_shared<planning::trajectory::Spline5>(ss->robot, current.pos, current.vel, current.acc);
}

planning::trajectory::Trajectory::~Trajectory() {}

/// @brief Compute a regular spline that is not surely safe for environment, meaning that,
/// if collision eventually occurs, it may be at robot's non-zero velocity.
/// @param current Current robot's state
/// @param target Target robot's state
/// @param t_iter_remain Remaining time in [s] in the current iteration
/// @param t_max Maximal available time in [s] for a spline computing
/// @param non_zero_final_vel Whether final spline velocity can be non-zero
/// @return The success of a spline computation
bool planning::trajectory::Trajectory::computeRegular(planning::trajectory::State current, 
    planning::trajectory::State target, float t_iter_remain, float t_max, bool non_zero_final_vel)
{
    std::chrono::steady_clock::time_point time_start_ { std::chrono::steady_clock::now() };    
    float t_remain { t_max };
    bool spline_computed { false };
    std::shared_ptr<planning::trajectory::Spline> spline_new 
        { std::make_shared<planning::trajectory::Spline5>(ss->robot, current.pos, current.vel, current.acc) };

    if (non_zero_final_vel)
    {
        is_zero_final_vel = false;
        size_t num_iter { 0 };
        float delta_t_max { ((target.pos - current.pos).cwiseQuotient(ss->robot->getMaxVel())).cwiseAbs().maxCoeff() };
        Eigen::VectorXf target_vel_max { (target.pos - current.pos) / delta_t_max };
        Eigen::VectorXf target_vel_min { Eigen::VectorXf::Zero(ss->num_dimensions) };
        Eigen::VectorXf target_vel { target_vel_max };
        
        while (!spline_computed && num_iter++ < max_num_iter_trajectory && t_remain > 0)
        {
            if (spline_new->compute(target.pos, target_vel)) 
            {
                spline_computed = true;
                setSpline(spline_new);
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
    
    // If spline was not computed or robot is getting away from 'new_current_pos'
    if ((!spline_computed || (new_current_pos - target.pos).norm() > (current.pos - target.pos).norm()) && t_remain > 0)
    {
        is_zero_final_vel = true;
        spline_computed = spline_new->compute(target.pos);
        if (spline_computed)
            setSpline(spline_new);
    }
    
    // std::cout << "\t Spline " << (!spline_computed ? "NOT " : "") << "computed with " 
    //           << (!is_zero_final_vel ? "NON-" : "") <<  "ZERO final velocity. \n";
    // std::cout << "Spline: \n" << spline << "\n";
    return spline_computed;
}

/// @brief Compute a safe spline that will render a robot motion surely safe for environment. 
/// If collision eventually occurs, it will be at robot's zero velocity, meaning that an obstacle hit the robot, and not vice versa. 
/// @param current Current robot's state
/// @param target Target robot's state
/// @param t_iter_remain Remaining time in [s] in the current iteration
/// @param t_max Maximal available time in [s] for a spline computing
/// @param q_current Current robot's configuration
/// @return The success of a spline computation
bool planning::trajectory::Trajectory::computeSafe(planning::trajectory::State current, 
    planning::trajectory::State target, float t_iter_remain, float t_max, const std::shared_ptr<base::State> q_current)
{
    std::chrono::steady_clock::time_point time_start_ { std::chrono::steady_clock::now() };
    std::shared_ptr<planning::trajectory::Spline> spline_new
        { std::make_shared<planning::trajectory::Spline5>(ss->robot, current.pos, current.vel, current.acc) };
    std::shared_ptr<planning::trajectory::Spline> spline_temp 
        { std::make_shared<planning::trajectory::Spline5>(ss->robot, current.pos, current.vel, current.acc) };
    std::shared_ptr<planning::trajectory::Spline> spline_emg_new 
        { std::make_shared<planning::trajectory::Spline4>(ss->robot, current.pos, current.vel, current.acc) };
    std::shared_ptr<planning::trajectory::Spline> spline_emg_temp { nullptr };
    
    float rho_robot {};
    float rho_obs {};
    bool is_safe {};
    bool spline_computed { false };
    bool spline_emg_computed { false };
    float t_iter { max_iter_time - t_iter_remain };
    float t_spline_max { t_iter_remain + (max_iter_time - max_remaining_iter_time) };
    int num_iter { 0 };
    int max_num_iter = std::ceil(std::log2(RealVectorSpaceConfig::NUM_INTERPOLATION_VALIDITY_CHECKS * 
                                           (current.pos - target.pos).norm() / RRTConnectConfig::EPS_STEP));
    if (max_num_iter <= 0) max_num_iter = 1;
    Eigen::VectorXf target_pos_min { current.pos };
    Eigen::VectorXf target_pos_max { target.pos };
    Eigen::VectorXf target_pos { target.pos };

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
        // std::cout << "target_pos: " << target_pos.transpose() << "\n";

        if (spline_temp->compute(target_pos))
        {
            // std::cout << "Spline is computed! \n";
            if (spline_temp->getTimeFinal() < t_spline_max)
            {
                rho_obs = max_obs_vel * (t_iter + spline_temp->getTimeFinal());
                if (rho_obs < q_current->getDistance())
                {
                    rho_robot = computeRho(target_pos);
                    if (rho_obs + rho_robot < q_current->getDistance())
                    {
                        spline_emg_computed = false;
                        is_safe = true;
                    }
                }
            }
            else
            {
                spline_emg_temp = std::make_shared<planning::trajectory::Spline4>
                (
                    ss->robot,
                    spline_temp->getPosition(t_spline_max),
                    spline_temp->getVelocity(t_spline_max),
                    spline_temp->getAcceleration(t_spline_max)
                );

                if (spline_emg_temp->compute())
                {
                    // std::cout << "Emergency spline is computed! \n";
                    rho_obs = max_obs_vel * (t_iter + t_spline_max + spline_emg_temp->getTimeFinal());
                    if (rho_obs < q_current->getDistance())
                    {
                        rho_robot = computeRho(spline_emg_temp->getPosition(INFINITY));
                        if (rho_obs + rho_robot < q_current->getDistance())
                        {
                            *spline_emg_new = *spline_emg_temp;
                            spline_emg_computed = true;
                            is_safe = true;
                            spline_temp->setTimeFinal(t_spline_max);
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
            *spline_new = *spline_temp;
            spline_computed = true;
            target_pos_min = target_pos;
            if (num_iter == 1) 
                break;
        }
        else
        {
            // std::cout << "\tRobot is NOT safe! \n";
            target_pos_max = target_pos;
        }
        target_pos = (target_pos_min + target_pos_max) / 2;
    }

    if (spline_computed)    // Check whether computed splines are collision-free
    {
        std::shared_ptr<planning::trajectory::Spline> spline_safe
        {
            spline_emg_computed ?
                std::make_shared<planning::trajectory::CompositeSpline>
                    (std::vector<std::shared_ptr<planning::trajectory::Spline>>({ spline_new, spline_emg_new })) :
                spline_new
        };

        // std::cout << "Spline: \n" << spline << "\n";
        spline_computed = isSafe(spline_safe, q_current, t_iter);

        if (spline_computed)
            setSpline(spline_safe);
    }

    return spline_computed;
}

/// @brief Check whether the computed spline is safe (i.e., collision-free during the time interval [0, 'spline_safe->getTimeFinal()'])
/// @param spline_safe Safe spline which needs to be validated.
/// @param q_current Current configuration from where bur spines are generated.
/// @param nearest_points Nearest points between the robot and obstacles.
/// @param t_iter Elapsed time from the beginning of iteration to a time instance when the spline is starting.
/// @return True if safe. False if not.
/// @note 'q_current' must have a distance-to-obstacles or its underestimation!
bool planning::trajectory::Trajectory::isSafe(const std::shared_ptr<planning::trajectory::Spline> spline_safe,
                                              const std::shared_ptr<base::State> q_current, float t_iter)
{
    // std::chrono::steady_clock::time_point time_start_ { std::chrono::steady_clock::now() };    
    float delta_t { TrajectoryConfig::TIME_STEP };
    size_t num_iter = std::ceil(spline_safe->getTimeFinal() / delta_t);
    delta_t = spline_safe->getTimeFinal() / num_iter;
    float rho_robot {};
    float rho_obs {};
    float t_init { 0 };
    std::shared_ptr<base::State> q_temp { q_current };
    std::shared_ptr<base::State> q_final { nullptr };
	Eigen::VectorXf delta_q {};

    for (float t = delta_t; t <= spline_safe->getTimeFinal() + RealVectorSpaceConfig::EQUALITY_THRESHOLD; t += delta_t)
    {
        // std::cout << "Considering t: " << t << "\n";
        q_final = ss->getNewState(spline_safe->getPosition(t));
        if (ss->robot->checkSelfCollision(q_final))
            return false;
        
        t_iter += delta_t;
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
    // std::cout << "Elapsed time for the spline collision checking: " << t_elapsed << " [us] \n";
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

void planning::trajectory::Trajectory::setSpline(const std::shared_ptr<planning::trajectory::Spline> spline_)
{
    spline = spline_;
    time_current = 0;
    time_final = spline->getTimeFinal();
}

/// @brief A method (v1) to convert a path 'path' to a corresponding trajectory.
/// Converting this path to trajectory (i.e., assigning time instances to these points) will be automatically done by this function.
/// This is done by creating a sequence of quintic splines in a way that all constraints on robot's maximal velocity, 
/// acceleration and jerk are surely always satisfied.
/// @param path Path containing all points that robot should visit.
/// @return Success of converting a path to a corresponding trajectory.
/// @note Be careful since the distance between each two adjacent points from 'path' should not be too long! 
/// The robot motion between them is generally not a straight line in C-space. 
/// Consider using 'preprocessPath' function from 'base::RealVectorSpace' class before using this function.
bool planning::trajectory::Trajectory::convertPathToTraj_v1(const std::vector<std::shared_ptr<base::State>> &path)
{
    std::vector<std::shared_ptr<planning::trajectory::Spline>> subsplines {};
    std::shared_ptr<planning::trajectory::Spline> spline_current
    {
        std::make_shared<planning::trajectory::Spline5>(
            ss->robot, 
            path.front()->getCoord(), 
            Eigen::VectorXf::Zero(ss->num_dimensions), 
            Eigen::VectorXf::Zero(ss->num_dimensions)
        )
    };
    std::shared_ptr<planning::trajectory::Spline> spline_next { nullptr };
    std::shared_ptr<base::State> q_current { nullptr };

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
                ss->computeDistance(q_current);     // Required by 'isSafe' function
                if (q_current->getDistance() <= 0 || !isSafe(spline_next, q_current, 0))
                    spline_computed = false;
            }
        }

        spline_current->setTimeFinal(t);
        subsplines.emplace_back(spline_current);
        spline_current = spline_next;
    }

    subsplines.emplace_back(spline_current);
    setSpline(std::make_shared<planning::trajectory::CompositeSpline>(subsplines));

    return true;
}

/// @brief A method (v2) to convert a path 'path' to a corresponding trajectory.
/// Converting this path to trajectory (i.e., assigning time instances to these points) will be automatically done by this function.
/// This is done by creating a sequence of quintic splines in a way that all constraints on robot's maximal velocity, 
/// acceleration and jerk are surely always satisfied.
/// @param path Path containing all points that robot should visit.
/// @return Success of converting a path to a corresponding trajectory.
/// @note Be careful since the distance between each two adjacent points from 'path' should not be too long! 
/// The robot motion between them is generally not a straight line in C-space. 
/// Consider using 'preprocessPath' function from 'base::RealVectorSpace' class before using this function.
bool planning::trajectory::Trajectory::convertPathToTraj_v2(const std::vector<std::shared_ptr<base::State>> &path)
{
    std::vector<std::shared_ptr<planning::trajectory::Spline>> subsplines {};
    std::shared_ptr<planning::trajectory::Spline> spline_current
    {
        std::make_shared<planning::trajectory::Spline5>(
            ss->robot, 
            path.front()->getCoord(), 
            Eigen::VectorXf::Zero(ss->num_dimensions), 
            Eigen::VectorXf::Zero(ss->num_dimensions)
        )
    };
    std::shared_ptr<base::State> q_current { nullptr };
    
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
        // std::cout << "i: " << i << " ---------------------------\n";
        // std::cout << "t_max: " << t_max << " [s] \n";

        while (!spline_computed && num++ < max_num_iter)
        {
            t = (num < max_num_iter) ?
                (t + t_max) / 2 :
                t_max;                  // Solution surely exists, and 'spline_computed' will become true.
            // std::cout << "t: " << t << " [s] \n";

            spline_computed = computeRegular
            (
                planning::trajectory::State
                (
                    spline_current->getPosition(t),
                    spline_current->getVelocity(t),
                    spline_current->getAcceleration(t)
                ),
                planning::trajectory::State
                (
                    path[i]->getCoord()
                ),
                max_remaining_iter_time,
                TrajectoryConfig::MAX_TIME_COMPUTE_REGULAR,
                non_zero_final_vel
            );
            
            if (spline_computed && num < max_num_iter)
            {
                q_current = ss->getNewState(spline_current->getPosition(t));
                ss->computeDistance(q_current);     // Required by 'isSafe' function
                if (q_current->getDistance() <= 0 || !isSafe(spline, q_current, 0))
                    spline_computed = false;
            }
        }

        spline_current->setTimeFinal(t);
        subsplines.emplace_back(spline_current);
        spline_current = spline;
    }

    subsplines.emplace_back(spline_current);
    setSpline(std::make_shared<planning::trajectory::CompositeSpline>(subsplines));

    return true;
}

/// @brief A method (v3) to convert a path 'path' to a corresponding trajectory.
/// Converting this path to trajectory (i.e., assigning time instances to these points) will be automatically done by this function.
/// This is done by creating a sequence of quintic splines in a way that all constraints on robot's maximal velocity, 
/// acceleration and jerk are surely always satisfied.
/// @param path Path containing all points that robot should visit.
/// @param must_visit Whether path points must be visited.
/// @return Success of converting a path to a corresponding trajectory.
/// @note Be careful since the distance between each two adjacent points from 'path' should not be too long! 
/// The robot motion between them is generally not a straight line in C-space. 
/// Consider using 'preprocessPath' function from 'base::RealVectorSpace' class before using this function.
bool planning::trajectory::Trajectory::convertPathToTraj_v3(const std::vector<std::shared_ptr<base::State>> &path, bool must_visit)
{
    std::vector<std::shared_ptr<planning::trajectory::Spline>> subsplines(path.size(), nullptr);
    bool spline_computed { false };
    size_t num_iter {};
    size_t max_num_iter { 5 };
    float delta_t_max {};
    Eigen::VectorXf target_vel_max {};
    Eigen::VectorXf target_vel_min {};
    Eigen::VectorXf target_vel {};
    Eigen::VectorXf target_pos {};
    std::vector<float> vel_coeff(path.size(), 1.0);
    const float vel_coeff_const { 0.9 };
    auto time_start_ { std::chrono::steady_clock::now() };
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
            target_pos = (path[i-1]->getCoord() + path[i]->getCoord()) / 2;
        else
            target_pos = path[i]->getCoord();
        
        spline_computed = false;
        num_iter = 0;
        delta_t_max = ((target_pos - path[i-1]->getCoord()).cwiseQuotient(ss->robot->getMaxVel())).cwiseAbs().maxCoeff();
        target_vel_max = (target_pos - path[i-1]->getCoord()) / delta_t_max;
        target_vel_min = Eigen::VectorXf::Zero(ss->num_dimensions);

        do
        {
            target_vel = vel_coeff[i] * (target_vel_max + target_vel_min) / 2;
            std::shared_ptr<planning::trajectory::Spline> spline_new 
            {
                std::make_shared<planning::trajectory::Spline5>(
                    ss->robot, 
                    subsplines[i-1]->getPosition(subsplines[i-1]->getTimeFinal()), 
                    subsplines[i-1]->getVelocity(subsplines[i-1]->getTimeFinal()), 
                    subsplines[i-1]->getAcceleration(subsplines[i-1]->getTimeFinal())
                )
            };
            
            if (spline_new->compute(target_pos, target_vel) && 
                ((monotonic && spline_new->checkPositionMonotonicity() != 0) || !monotonic))
            {
                *subsplines[i] = *spline_new;
                target_vel_min = target_vel;
                spline_computed = true;
            }
            else
                target_vel_max = target_vel;
        }
        while (++num_iter < max_num_iter);

        if (!spline_computed)
        {
            spline_computed = subsplines[i]->compute(target_pos) && 
                              ((monotonic && subsplines[i]->checkPositionMonotonicity() != 0) || !monotonic);
            if (!spline_computed)
            {
                // std::cout << "Spline not computed! \n";
                vel_coeff[--i] *= vel_coeff_const;
                --i;
            }
            // else std::cout << "Spline computed with zero final velocity! \n";
        }

        if (std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::steady_clock::now() - time_start_).count() * 1e-3 > max_time)
        {
            spline_computed = false;
            break;
        }
    }

    subsplines.erase(subsplines.begin());
    
    if (spline_computed)
    {
        setSpline(std::make_shared<planning::trajectory::CompositeSpline>(subsplines));
        return true;
    }
    else
    {
        std::cout << "Could not convert path to a trajectory! \n";
        return false;
    }
}

bool planning::trajectory::Trajectory::convertPathToTraj(const std::vector<std::shared_ptr<base::State>> &path)
{
    return convertPathToTraj_v1(path);      // The best results are obtained with this one.
    // return convertPathToTraj_v2(path);
    // return convertPathToTraj_v3(path);
}
