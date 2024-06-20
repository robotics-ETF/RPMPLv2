#include "Spline4.h"
#include "Spline5Config.h"

planning::trajectory::Spline4::Spline4(const std::shared_ptr<robots::AbstractRobot> robot_, const Eigen::VectorXf &q_current) :
    Spline(4, robot_, q_current) 
{
    a = b = c = d = Eigen::VectorXf::Zero(num_dimensions);
    e = q_current;
}

planning::trajectory::Spline4::Spline4(const std::shared_ptr<robots::AbstractRobot> robot_, const Eigen::VectorXf &q_current, 
    const Eigen::VectorXf &q_current_dot, const Eigen::VectorXf &q_current_ddot) :
    Spline(5, robot_, q_current)
{
    a = b = Eigen::VectorXf::Zero(num_dimensions);
    c = q_current_ddot / 2;
    d = q_current_dot;
    e = q_current;
}

/// @brief Compute a quartic spline from 'q_current' to unknown 'q_final' such that robot stops at 'q_final', where all constraints on
/// robot's maximal velocity, acceleration and jerk are satisfied.
/// @return Success of computing the spline.
/// @note After reaching a final configuration, velocity and acceleration will remain zero!
/// This function is useful for emergency stopping of the robot.
bool planning::trajectory::Spline4::compute()
{
    return compute(Eigen::VectorXf::Zero(num_dimensions), Eigen::VectorXf::Zero(num_dimensions));
}

/// @brief Compute a quartic spline from 'q_current' to unknown 'q_final' such that robot stops at 'q_final', where all constraints on
/// robot's maximal velocity, acceleration and jerk are satisfied.
/// @param q_final_dot Desired velocity in a final configuration.
/// @return Success of computing the spline.
/// @note After reaching a final configuration, acceleration will remain zero, while velocity will remain constant!
bool planning::trajectory::Spline4::compute(const Eigen::VectorXf &q_final_dot)
{
    return compute(q_final_dot, Eigen::VectorXf::Zero(num_dimensions));
}

/// @brief Compute a quartic spline from 'q_current' to unknown 'q_final' such that robot stops at 'q_final', where all constraints on
/// robot's maximal velocity, acceleration and jerk are satisfied.
/// @param q_final_dot Desired velocity in a final configuration.
/// @param q_final_ddot Desired acceleration in a final configuration.
/// @return Success of computing the spline.
/// @note After reaching a final configuration, acceleration will remain constant, while velocity will increase linearly!
bool planning::trajectory::Spline4::compute(const Eigen::VectorXf &q_final_dot, const Eigen::VectorXf &q_final_ddot)
{
    std::chrono::steady_clock::time_point time_start_ { std::chrono::steady_clock::now() };
    if (q_final_dot.norm() < RealVectorSpaceConfig::EQUALITY_THRESHOLD)
        is_zero_final_vel = true;
    else
        is_zero_final_vel = false;

    if (q_final_ddot.norm() < RealVectorSpaceConfig::EQUALITY_THRESHOLD)
        is_zero_final_acc = true;
    else
        is_zero_final_acc = false;

    int idx_corr { -1 };
    float t_f_opt { -1 };
    float t_f { 0 }, t_f_left { 0 }, t_f_right { 0 };
    float b_left {}, b_right {};
    float a_left {}, a_right {};
    const size_t max_num_iter = std::ceil(std::log2(2 * robot->getMaxJerk(0) / Spline5Config::FINAL_JERK_STEP));

    for (size_t idx = 0; idx < num_dimensions; idx++)
    {
        std::cout << "Joint: " << idx << " ---------------------------------------------------\n";
        std::cout << "Init. pos: " << e(idx) << "\t Final pos: unknown \n";
        std::cout << "Init. vel: " << d(idx) << "\t Final vel: " << q_final_dot(idx) << "\n";
        std::cout << "Init. acc: " << 2*c(idx) << "\t Final acc: " << q_final_ddot(idx) << "\n";
        std::cout << "t_f_opt:   " << t_f_opt << "\n";

        if (d(idx) == 0 && c(idx) == 0)    // Special case
        {
            std::cout << "Joint position does not change. Just continue! \n";
            continue;
        }

        if (t_f_opt > 0)
        {
            t_f = t_f_opt;
            b(idx) = compute_b(idx, t_f, q_final_dot(idx), q_final_ddot(idx));
            a(idx) = compute_a(idx, t_f, q_final_ddot(idx));

            if (checkConstraints(idx, t_f))
            {
                std::cout << "All constraints are satisfied for t_f: " << t_f << " [s]. Just continue! \n";
                continue;
            }
            else
            {
                idx_corr = idx;
                std::cout << "idx_corr: " << idx_corr << "\n";
            }
        }

        if (4*c(idx) + q_final_ddot(idx) >= 0 && d(idx) - q_final_dot(idx) >= 0)
        {
            b_left = -robot->getMaxJerk(idx) / 6;
            b_right = -RealVectorSpaceConfig::EQUALITY_THRESHOLD;
        }
        else if (4*c(idx) + q_final_ddot(idx) >= 0 && d(idx) - q_final_dot(idx) < 0)
        {
            b_left = (4*c(idx) + q_final_ddot(idx)) * (4*c(idx) + q_final_ddot(idx)) / (36*(d(idx) - q_final_dot(idx)));
            b_right = robot->getMaxJerk(idx) / 6;
            if (b_left < -robot->getMaxJerk(idx) / 6)
                b_left = -robot->getMaxJerk(idx) / 6;
        }
        else if (4*c(idx) + q_final_ddot(idx) <= 0 && d(idx) - q_final_dot(idx) > 0)
        {
            b_left = -robot->getMaxJerk(idx) / 6;
            b_right = (4*c(idx) + q_final_ddot(idx)) * (4*c(idx) + q_final_ddot(idx)) / (36*(d(idx) - q_final_dot(idx)));
            if (b_right > robot->getMaxJerk(idx) / 6)
                b_right = robot->getMaxJerk(idx) / 6;
        }
        else if (4*c(idx) + q_final_ddot(idx) <= 0 && d(idx) - q_final_dot(idx) <= 0)
        {
            b_left = RealVectorSpaceConfig::EQUALITY_THRESHOLD;
            b_right = robot->getMaxJerk(idx) / 6;
        }
        std::cout << "Finding b between " << b_left << " and " << b_right << "\n";

        b(idx) = b_left;
        t_f_left = computeFinalTime(idx, q_final_dot(idx), q_final_ddot(idx));
        a_left = a(idx);

        b(idx) = b_right;
        t_f_right = computeFinalTime(idx, q_final_dot(idx), q_final_ddot(idx));
        a_right = a(idx);

        std::cout << "t_f_left: " << t_f_left << "\t t_f_right: " << t_f_right << "\n";
        if (t_f_left == 0 && t_f_right == 0)
        {
            std::cout << "No solution! \n";
            return false;
        }
        else if (t_f_left > 0 && t_f_right > 0)
        {
            std::cout << "Solution is found! Just continue! \n";
            if (t_f_left < t_f_right)
            {
                t_f_opt = t_f_left;
                a(idx) = a_left;
                b(idx) = b_left;
            }
            else
            {
                t_f_opt = t_f_right;
                a(idx) = a_right;
                b(idx) = b_right;
            }
            continue;
        }

        // Using bisection method to find t_f, when t_f_left < inf and t_f_right = inf, or vice versa
        bool found { false };
        for (size_t num = 0; num < max_num_iter; num++)
        {
            std::cout << "Num. iter: " << num << " --------------------------\n";
            std::cout << "b_left: " << b_left << "\t b_right: " << b_right << "\n";
            b(idx) = (b_left + b_right) / 2;
            t_f = computeFinalTime(idx, q_final_dot(idx), q_final_ddot(idx));

            if (t_f_left == 0)
            {
                if (t_f == 0)
                {
                    a_left = a(idx);
                    b_left = b(idx);
                }
                else
                {
                    a_right = a(idx);
                    b_right = b(idx);
                }
            }
            else
            {
                if (t_f == 0)
                {
                    a_right = a(idx);
                    b_right = b(idx);
                }
                else
                {
                    a_left = a(idx);
                    b_left = b(idx);
                }
            }
            
            if (t_f > 0)
            {
                t_f_opt = t_f;
                found = true;
            }
        }

        if (!found)
            return false;
        else if (t_f_left == 0)
        {
            a(idx) = a_right;
            b(idx) = b_right;
        }
        else if (t_f_right == 0)
        {
            a(idx) = a_left;
            b(idx) = b_left;
        }
    }

    // Corrections
    t_f = t_f_opt;
    for (int idx = 0; idx < idx_corr; idx++)
    {
        // std::cout << "Correcting joint: " << idx << " ---------------------------------------------------\n";
        b(idx) = compute_b(idx, t_f, q_final_dot(idx), q_final_ddot(idx));
        a(idx) = compute_a(idx, t_f, q_final_ddot(idx));
    }

    // Solution is found. Set the parameters for a new spline
    time_final = t_f_opt;
    for (size_t idx = 0; idx < num_dimensions; idx++)
        coeff.row(idx) << e(idx), d(idx), c(idx), b(idx), a(idx);
    
    auto t_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start_).count();
    std::cout << "Elapsed time: " << t_elapsed << " [us] \n";

    return true;
}

/// @brief Compute a final time, i.e., a time to reach from 'q_current(idx)' to 'q_final(idx)'.
/// @param idx Index of robot's joint
/// @param q_f_dot Desired 'idx'-th velocity in a final configuration
/// @param q_f_ddot Desired 'idx'-th acceleration in a final configuration
/// @return Final time. If final time is zero, it means that constraints are not satisfied.
/// If final time is infinite, it means there is no solution.
float planning::trajectory::Spline4::computeFinalTime(size_t idx, float q_f_dot, float q_f_ddot)
{
    float t_f { 0 };
    float D = std::sqrt((4*c(idx) + q_f_ddot)*(4*c(idx) + q_f_ddot) - 36*b(idx)*(d(idx) - q_f_dot));
    std::vector<float> t_sol { (-4*c(idx) - q_f_ddot + D) / (6*b(idx)),
                               (-4*c(idx) - q_f_ddot - D) / (6*b(idx)) };
    for (size_t i = 0; i < t_sol.size(); i++)
    {
        if (t_sol[i] > 0)
            t_f = std::min(t_f, t_sol[i]);
    }
    std::cout << "For b: " << b(idx) << ", it follows t_f: " << t_f << "\n";

    a(idx) = compute_a(idx, t_f, q_f_ddot);
    std::cout << "a: " << a(idx) << ",\t b: " << b(idx) << ",\t c: " << c(idx) << ",\t " 
              << "d: " << d(idx) << ",\t e: " << e(idx) << "\n";
    
    if (!checkConstraints(idx, t_f))
        return 0;

    return t_f;
}

float planning::trajectory::Spline4::compute_a(size_t idx, float t_f, float q_f_ddot)
{
    return (q_f_ddot - 2*c(idx) - 6*b(idx)*t_f) / (12*t_f*t_f);
}

float planning::trajectory::Spline4::compute_b(size_t idx, float t_f, float q_f_dot, float q_f_ddot)
{
    return (q_f_dot - d(idx) - (4*c(idx) + q_f_ddot)*t_f/3) / (t_f*t_f);
}

bool planning::trajectory::Spline4::checkConstraints(size_t idx, float t_f)
{
    // Maximal jerk constraint
    std::cout << "\t Max. jerk.\t t_f: " << 0 << "\t value: " << 6*std::abs(c(idx)) << "\n";
    std::cout << "\t Max. jerk.\t t_f: " << t_f << "\t value: " << std::abs(getJerk(t_f, idx, t_f)) << "\n";
    if (6*std::abs(b(idx)) > robot->getMaxJerk(idx) + RealVectorSpaceConfig::EQUALITY_THRESHOLD ||
        std::abs(24*a(idx)*t_f + 6*b(idx)) > robot->getMaxJerk(idx) + RealVectorSpaceConfig::EQUALITY_THRESHOLD)
    {
        std::cout << "\t Maximal jerk constraint not satisfied! \n";
        return false;
    }

    // Maximal acceleration constraint
    // Note: Initial and final acceleration are surely satisfied!
    float t_max_ { getMaxAccelerationTimes(idx)[0] };
    std::cout << "\t Max. acceleration.\t t_max: " << t_max_ << "\t value: " << std::abs(getAcceleration(t_max_, idx, t_f)) << "\n";
    if (std::abs(2*c(idx) - 3*b(idx)*b(idx) / (4*a(idx))) > robot->getMaxAcc(idx))
    {
        std::cout << "\t Maximal acceleration constraint not satisfied! \n";
        return false;
    }

    // Maximal velocity constraint
    // Note: Initial and final velocity are surely satisfied!
    std::vector<float> t_max { getMaxVelocityTimes(idx) };
    for (size_t i = 0; i < t_max.size(); i++)
    {
        std::cout << "\t Max. velocity.\t t_max: " << t_max[i] << "\t value: " << std::abs(getVelocity(t_max[i], idx, t_f)) << "\n";
        if (std::abs(getVelocity(t_max[i], idx, t_f)) > robot->getMaxVel(idx))
        {
            std::cout << "\t Maximal velocity constraint not satisfied! \n";
            return false;
        }
    }

    std::cout << "\t All constraints are satisfied! \n";
    return true;
}

std::vector<float> planning::trajectory::Spline4::getMaxVelocityTimes(size_t idx)
{
    std::vector<float> t_max {};

    if (a(idx) != 0)
    {
        float D { 36*b(idx)*b(idx) - 96*a(idx)*c(idx) };
        if (D >= 0)
        {
            for (int sign : {-1, 1})
                t_max.emplace_back((-6*b(idx) + sign*std::sqrt(D)) / (24*a(idx)));
        }
    }

    return t_max;
}

std::vector<float> planning::trajectory::Spline4::getMaxAccelerationTimes(size_t idx)
{
    std::vector<float> t_max {};

    if (a(idx) != 0)
        t_max.emplace_back(-b(idx) / (4*a(idx)));

    return t_max;
}

float planning::trajectory::Spline4::getPosition(float t, size_t idx, float t_f)
{
    float delta_t { 0 };
    float vel_final { 0 };

    if (t > t_f)
    {
        if (!is_zero_final_vel)
        {
            delta_t = t - t_f;
            vel_final = getVelocity(t, idx, t_f);
        }
        t = t_f;
    }
    else if (t < 0)
        t = 0;
    
    return e(idx) + d(idx)*t + c(idx)*t*t + b(idx)*t*t*t + a(idx)*t*t*t*t 
           + delta_t * vel_final;
}

float planning::trajectory::Spline4::getVelocity(float t, size_t idx, float t_f)
{
    float delta_t { 0 };
    float acc_final { 0 };

    if (t > t_f)
    {
        if (!is_zero_final_acc)
        {
            delta_t = t - t_f;
            acc_final = getAcceleration(t, idx, t_f);
        }
        t = t_f;
    }
    else if (t < 0)
        t = 0;

    return d(idx) + 2*c(idx)*t + 3*b(idx)*t*t + 4*a(idx)*t*t*t 
           + delta_t * acc_final;
}

float planning::trajectory::Spline4::getAcceleration(float t, size_t idx, float t_f)
{
    if (t > t_f)
        t = t_f;
    else if (t < 0)
        t = 0;

    return 2*c(idx) + 6*b(idx)*t + 12*a(idx)*t*t;
}

float planning::trajectory::Spline4::getJerk(float t, size_t idx, float t_f)
{
    if (t > t_f)
        t = t_f;
    else if (t < 0)
        t = 0;

    return 6*b(idx) + 24*a(idx)*t;
}
