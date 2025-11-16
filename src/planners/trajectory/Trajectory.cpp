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

bool planning::trajectory::Trajectory::computeRegularTraj(const planning::trajectory::State &current, 
    const planning::trajectory::State &target)
{
    // std::cout << "Trying to compute a regular spline...\n";
    std::shared_ptr<planning::trajectory::Spline> spline_new 
        { std::make_shared<planning::trajectory::Spline5>(ss->robot, current.pos, current.vel, current.acc) };

    if (spline_new->compute(target.pos, target.vel)) 
    {
        setSpline(spline_new);
        return true;
    }

    return false;
}

bool planning::trajectory::Trajectory::computeSafeTraj(const planning::trajectory::State &current, 
    const planning::trajectory::State &target, float t_iter, float t_spline_max, const std::shared_ptr<base::State> q_current)
{
    // std::cout << "Trying to compute a safe spline...\n";
    std::shared_ptr<planning::trajectory::Spline> spline_new 
        { std::make_shared<planning::trajectory::Spline5>(ss->robot, current.pos, current.vel, current.acc) };
        
    if (!spline_new->compute(target.pos, target.vel))
    {
        // std::cout << "Could not compute spline!\n";
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

    std::shared_ptr<planning::trajectory::Spline> spline_emg_new { nullptr };
    bool is_safe { false };
    bool spline_emg_computed { false };
    float rho_robot {};
    float rho_obs {};

    if (spline_new->getTimeFinal() < t_spline_max && spline_new->getIsZeroFinalVel())
    {
        rho_obs = max_obs_vel * (t_iter + spline_new->getTimeFinal());
        if (rho_obs < q_current->getDistance())
        {
            rho_robot = computeRho(target.pos);
            if (rho_obs + rho_robot < q_current->getDistance())
            {
                spline_emg_computed = false;
                is_safe = true;
            }
        }
    }
    else
    {
        spline_emg_new = std::make_shared<planning::trajectory::Spline4>
        (
            ss->robot,
            spline_new->getPosition(t_spline_max),
            spline_new->getVelocity(t_spline_max),
            spline_new->getAcceleration(t_spline_max)
        );

        if (spline_emg_new->compute())
        {
            // std::cout << "Emergency spline is computed! \n";
            rho_obs = max_obs_vel * (t_iter + t_spline_max + spline_emg_new->getTimeFinal());
            if (rho_obs < q_current->getDistance())
            {
                rho_robot = computeRho(spline_emg_new->getPosition(spline_emg_new->getTimeFinal()));
                if (rho_obs + rho_robot < q_current->getDistance())
                {
                    spline_emg_computed = true;
                    is_safe = true;
                    spline_new->setTimeFinal(t_spline_max);
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
        std::shared_ptr<planning::trajectory::Spline> spline_safe
        {
            spline_emg_computed ?
                std::make_shared<planning::trajectory::CompositeSpline>
                    (std::vector<std::shared_ptr<planning::trajectory::Spline>>({ spline_new, spline_emg_new })) :
                spline_new
        };

        is_safe = isSafeSpline(spline_safe, q_current, t_iter);
        if (is_safe)
            setSpline(spline_safe);
    }

    // std::cout << "Spline is " << (is_safe ? "" : "NOT ") << "safe!\n";
    return is_safe;
}

/// @brief Check whether the computed spline is safe (i.e., collision-free during the time interval [0, 'spline_safe->getTimeFinal()'])
/// @param spline_safe Safe spline which needs to be validated.
/// @param q_current Current configuration from where bur spines are generated.
/// @param t_iter Elapsed time from the beginning of iteration to a time instance when the spline is starting.
/// @return True if safe. False if not.
/// @note 'q_current' must have a distance-to-obstacles or its underestimation!
bool planning::trajectory::Trajectory::isSafeSpline(const std::shared_ptr<planning::trajectory::Spline> spline_safe,
                                                    const std::shared_ptr<base::State> q_current, float t_iter)
{
    std::vector<Eigen::VectorXf> pos_points {};
    for (float t = 0; t <= spline_safe->getTimeFinal(); t += TrajectoryConfig::TIME_STEP)
        pos_points.emplace_back(spline_safe->getPosition(t));
    
    return isSafe(pos_points, q_current, t_iter);
}

void planning::trajectory::Trajectory::setSpline(const std::shared_ptr<planning::trajectory::Spline> spline_)
{
    spline = spline_;
    time_current = 0;
    time_final = spline->getTimeFinal();
    // std::cout << "Spline: \n" << spline << "\n";
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
                ss->computeDistance(q_current);     // Required by 'isSafeSpline' function
                if (q_current->getDistance() <= 0 || !isSafeSpline(spline_next, q_current, 0))
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
                ss->computeDistance(q_current);     // Required by 'isSafeSpline' function
                if (q_current->getDistance() <= 0 || !isSafeSpline(spline, q_current, 0))
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
    planning::trajectory::State target {};
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
            target.pos = (path[i-1]->getCoord() + path[i]->getCoord()) / 2;
        else
            target.pos = path[i]->getCoord();
        
        spline_computed = false;
        num_iter = 0;
        delta_t_max = ((target.pos - path[i-1]->getCoord()).cwiseQuotient(ss->robot->getMaxVel())).cwiseAbs().maxCoeff();
        target_vel_max = (target.pos - path[i-1]->getCoord()) / delta_t_max;
        target_vel_min = Eigen::VectorXf::Zero(ss->num_dimensions);

        do
        {
            target.pos = vel_coeff[i] * (target_vel_max + target_vel_min) / 2;
            std::shared_ptr<planning::trajectory::Spline> spline_new 
            {
                std::make_shared<planning::trajectory::Spline5>(
                    ss->robot, 
                    subsplines[i-1]->getPosition(subsplines[i-1]->getTimeFinal()), 
                    subsplines[i-1]->getVelocity(subsplines[i-1]->getTimeFinal()), 
                    subsplines[i-1]->getAcceleration(subsplines[i-1]->getTimeFinal())
                )
            };
            
            if (spline_new->compute(target.pos, target.pos) && 
                ((monotonic && spline_new->checkPositionMonotonicity() != 0) || !monotonic))
            {
                *subsplines[i] = *spline_new;
                target_vel_min = target.pos;
                spline_computed = true;
            }
            else
                target_vel_max = target.pos;
        }
        while (++num_iter < max_num_iter);

        if (!spline_computed)
        {
            spline_computed = subsplines[i]->compute(target.pos) && 
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
