#include "Spline5.h"
#include "RealVectorSpaceConfig.h"

// #include <unsupported/Eigen/Polynomials>

planning::trajectory::Spline5::Spline5(const std::shared_ptr<robots::AbstractRobot> robot_, const Eigen::VectorXf &q_current) :
    Spline(5, robot_, q_current) 
{
    a = b = c = d = e = Eigen::VectorXf::Zero(num_dimensions);
    f = q_current;
}

planning::trajectory::Spline5::Spline5(const std::shared_ptr<robots::AbstractRobot> robot_, const Eigen::VectorXf &q_current, 
    const Eigen::VectorXf &q_current_dot, const Eigen::VectorXf &q_current_ddot) :
    Spline(5, robot_, q_current)
{
    a = b = c = Eigen::VectorXf::Zero(num_dimensions);
    d = q_current_ddot / 2;
    e = q_current_dot;
    f = q_current;
}

/// @brief Compute a quintic spline from 'q_current' to 'q_final' such that robot stops at 'q_final', where all constraints on
/// robot's maximal velocity, acceleration and jerk are satisfied.
/// @param q_final Desired final configuration in which the spline is ending.
/// @return Success of computing the spline.
/// @note Velocity and acceleration in a final configuration will be zero!
bool planning::trajectory::Spline5::compute(const Eigen::VectorXf &q_final)
{
    return compute(q_final, Eigen::VectorXf::Zero(num_dimensions), Eigen::VectorXf::Zero(num_dimensions));
}

/// @brief Compute a quintic spline from 'q_current' to 'q_final' such that robot stops at 'q_final', where all constraints on
/// robot's maximal velocity, acceleration and jerk are satisfied.
/// @param q_final Desired final configuration in which the spline is ending.
/// @param q_final_dot Desired velocity in a final configuration.
/// @return Success of computing the spline.
/// @note Acceleration in a final configuration will be zero!
/// @note Velocity in a final configuration will be constant!
bool planning::trajectory::Spline5::compute(const Eigen::VectorXf &q_final, const Eigen::VectorXf &q_final_dot)
{
    return compute(q_final, q_final_dot, Eigen::VectorXf::Zero(num_dimensions));
}

/// @brief Compute a quintic spline from 'q_current' to 'q_final' such that robot stops at 'q_final', where all constraints on
/// robot's maximal velocity, acceleration and jerk are satisfied.
/// @param q_final Desired final configuration in which the spline is ending.
/// @param q_final_dot Desired velocity in a final configuration.
/// @param q_final_ddot Desired acceleration in a final configuration.
/// @return Success of computing the spline.
/// @note Velocity and acceleration in a final configuration will be constant!
bool planning::trajectory::Spline5::compute(const Eigen::VectorXf &q_final, const Eigen::VectorXf &q_final_dot, const Eigen::VectorXf &q_final_ddot)
{
    // std::chrono::steady_clock::time_point time_start_ = std::chrono::steady_clock::now();
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
    Eigen::Vector3f abc_left {}, abc_right {};       // a, b and c coefficients, respectively
    const size_t max_num_iter = std::ceil(std::log2(2 * robot->getMaxJerk(0) / Spline5Config::FINAL_JERK_STEP));

    for (size_t idx = 0; idx < num_dimensions; idx++)
    {
        // std::cout << "Joint: " << idx << " ---------------------------------------------------\n";
        // std::cout << "Init. pos: " << f(idx) << "\t Final pos: " << q_final(idx) <<  "\n";
        // std::cout << "Init. vel: " << e(idx) << "\t Final vel: 0 \n";
        // std::cout << "Init. acc: " << 2*d(idx) << "\t Final acc: 0 \n";
        // std::cout << "t_f_opt:   " << t_f_opt << "\n";

        if (f(idx) == q_final(idx) && e(idx) == 0 && d(idx) == 0)    // Special case
        {
            // std::cout << "Joint position does not change. Just continue! \n";
            continue;
        }

        if (t_f_opt > 0)
        {
            t_f = t_f_opt;
            c(idx) = compute_c(idx, t_f, q_final(idx), q_final_dot(idx), q_final_ddot(idx));
            b(idx) = compute_b(idx, t_f, q_final_dot(idx), q_final_ddot(idx));
            a(idx) = compute_a(idx, t_f, q_final_ddot(idx));

            if (checkConstraints(idx, t_f))
            {
                // std::cout << "All constraints are satisfied for t_f: " << t_f << " [s]. Just continue! \n";
                continue;
            }
            else
            {
                idx_corr = idx;
                // std::cout << "idx_corr: " << idx_corr << "\n";
            }
        }

        c(idx) = -robot->getMaxJerk(idx) / 6;
        t_f_left = computeFinalTime(idx, q_final(idx), q_final_dot(idx), q_final_ddot(idx));
        abc_left << a(idx), b(idx), c(idx);

        c(idx) = robot->getMaxJerk(idx) / 6;
        t_f_right = computeFinalTime(idx, q_final(idx), q_final_dot(idx), q_final_ddot(idx));
        abc_right << a(idx), b(idx), c(idx);

        // std::cout << "t_f_left: " << t_f_left << "\t t_f_right: " << t_f_right << "\n";
        if ((t_f_left == INFINITY) && (t_f_right == INFINITY || t_f_left == 0) && (t_f_right == 0))
        {
            // std::cout << "No solution! 'q_final' must be changed! \n";
            return false;
        }
        else if (t_f_left > 0 && t_f_left < INFINITY && t_f_right > 0 && t_f_right < INFINITY)
        {
            // std::cout << "Solution is found! Just continue! \n";
            if (t_f_left < t_f_right)
            {
                t_f_opt = t_f_left;
                a(idx) = abc_left(0); b(idx) = abc_left(1); c(idx) = abc_left(2);
            }
            else
            {
                t_f_opt = t_f_right;
                a(idx) = abc_right(0); b(idx) = abc_right(1); c(idx) = abc_right(2);
            }
            continue;
        }
        else if (t_f_left > 0 && t_f_left < INFINITY)
        {
            // std::cout << "Solution is found! Just continue! \n";
            t_f_opt = t_f_left;
            a(idx) = abc_left(0); b(idx) = abc_left(1); c(idx) = abc_left(2);
            continue;
        }
        else if (t_f_right > 0 && t_f_right < INFINITY)
        {
            // std::cout << "Solution is found! Just continue! \n";
            t_f_opt = t_f_right;
            a(idx) = abc_right(0); b(idx) = abc_right(1); c(idx) = abc_right(2);
            continue;
        }

        // Using bisection method to find t_f, when t_f_left = 0 and t_f_right = inf, or vice versa
        bool found { false };
        for (size_t num = 0; num < max_num_iter; num++)
        {
            // std::cout << "Num. iter: " << num << " --------------------------\n";
            // std::cout << "c_left: " << abc_left(2) << "\t c_right: " << abc_right(2) << "\n";
            c(idx) = (abc_left(2) + abc_right(2)) / 2;
            t_f = computeFinalTime(idx, q_final(idx), q_final_dot(idx), q_final_ddot(idx));

            if (t_f_left == 0)
            {
                if (t_f == 0)
                    abc_left << a(idx), b(idx), c(idx);
                else
                    abc_right << a(idx), b(idx), c(idx);
            }
            else
            {
                if (t_f == 0)
                    abc_right << a(idx), b(idx), c(idx);
                else
                    abc_left << a(idx), b(idx), c(idx);
            }
            
            if (t_f > 0 && t_f < INFINITY)
            {
                t_f_opt = t_f;
                found = true;
            }
        }

        if (!found)
            return false;
        else if (t_f_left == 0)
        {
            a(idx) = abc_right(0); b(idx) = abc_right(1); c(idx) = abc_right(2);
        }
        else if (t_f_right == 0)
        {
            a(idx) = abc_left(0); b(idx) = abc_left(1); c(idx) = abc_left(2);
        }
    }

    // Corrections
    t_f = t_f_opt;
    for (int idx = 0; idx < idx_corr; idx++)
    {
        // std::cout << "Correcting joint: " << idx << " ---------------------------------------------------\n";
        c(idx) = compute_c(idx, t_f, q_final(idx), q_final_dot(idx), q_final_ddot(idx));
        b(idx) = compute_b(idx, t_f, q_final_dot(idx), q_final_ddot(idx));
        a(idx) = compute_a(idx, t_f, q_final_ddot(idx));
    }

    // Solution is found. Set the parameters for a new spline
    time_final = t_f_opt;
    for (size_t idx = 0; idx < num_dimensions; idx++)
        coeff.row(idx) << f(idx), e(idx), d(idx), c(idx), b(idx), a(idx);
    
    // auto t_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start_).count();
    // std::cout << "Elapsed time: " << t_elapsed << " [us] \n";

    return true;
}

/// @brief Compute a final time, i.e., a time to reach from 'q_current(idx)' to 'q_final(idx)'.
/// @param idx Index of robot's joint
/// @param q_f Desired final 'idx'-th configuration
/// @param q_f_dot Desired 'idx'-th velocity in a final configuration
/// @param q_f_ddot Desired 'idx'-th acceleration in a final configuration
/// @return Final time. If final time is zero, it means that constraints are not satisfied.
/// If final time is infinite, it means there is no solution.
float planning::trajectory::Spline5::computeFinalTime(size_t idx, float q_f, float q_f_dot, float q_f_ddot)
{
    float t_f { INFINITY };
    std::vector<float> t_sol { solveQubicEquation(c(idx), 3*d(idx) - 0.5*q_f_ddot, 6*e(idx) + 4*q_f_dot, 10*(f(idx) - q_f)) };

    // std::cout << "For c: " << c(idx) << ", it follows t_f: ";
    for (size_t i = 0; i < t_sol.size(); i++)
    {
        // std::cout << t_sol[i] << "\t";
        if (t_sol[i] > 0)
            t_f = std::min(t_f, t_sol[i]);
    }
    // std::cout << "\n";

    if (t_f == INFINITY)
        return INFINITY;

    b(idx) = compute_b(idx, t_f, q_f_dot, q_f_ddot);
    a(idx) = compute_a(idx, t_f, q_f_ddot);

    // std::cout << "a: " << a(idx) << ",\t b: " << b(idx) << ",\t c: " << c(idx) << ",\t " 
    //           << "d: " << d(idx) << ",\t e: " << e(idx) << ",\t f: " << f(idx) << "\n";
    
    if (!checkConstraints(idx, t_f))
        return 0;

    return t_f;
}

float planning::trajectory::Spline5::compute_a(size_t idx, float t_f, float q_f_ddot)
{
    return -(6*b(idx) * t_f*t_f 
            + 3*c(idx) * t_f 
            + d(idx) - 0.5*q_f_ddot) 
            / (10*t_f*t_f*t_f);
}

float planning::trajectory::Spline5::compute_b(size_t idx, float t_f, float q_f_dot, float q_f_ddot)
{
    return -(3*c(idx) * t_f*t_f 
            + (3*d(idx) + 0.5*q_f_ddot) * t_f 
            + 2*(e(idx) - q_f_dot)) 
            / (2*t_f*t_f*t_f);
}

float planning::trajectory::Spline5::compute_c(size_t idx, float t_f, float q_f, float q_f_dot, float q_f_ddot)
{
    return -((3*d(idx) - 0.5*q_f_ddot) * t_f*t_f 
            + (6*e(idx) + 4*q_f_dot) * t_f 
            + 10*(f(idx) - q_f)) 
            / (t_f*t_f*t_f);
}

bool planning::trajectory::Spline5::checkConstraints(size_t idx, float t_f)
{
    // Maximal jerk constraint
    // std::cout << "\t Max. jerk.\t t_f: " << 0 << "\t value: " << 6 * std::abs(c(idx)) << "\n";
    // std::cout << "\t Max. jerk.\t t_f: " << t_f << "\t value: " << std::abs(getJerk(t_f, idx, t_f)) << "\n";
    std::vector<float> t_max { getMaxJerkTimes(idx) };
    for (size_t i = 0; i < t_max.size(); i++)
    {
        // std::cout << "\t Max. jerk.\t t_max: " << t_max[i] << "\t value: " << std::abs(getJerk(t_max[i], idx, t_f)) << "\n";
        if (6 * std::abs(c(idx)) > robot->getMaxJerk(idx) + RealVectorSpaceConfig::EQUALITY_THRESHOLD ||
            std::abs(getJerk(t_f, idx, t_f)) > robot->getMaxJerk(idx) + RealVectorSpaceConfig::EQUALITY_THRESHOLD || 
            (std::abs(getJerk(t_max[i], idx, t_f)) > robot->getMaxJerk(idx)))
        {
            // std::cout << "\t Maximal jerk constraint not satisfied! \n";
            return false;
        }
    }

    // Maximal acceleration constraint
    // Note: Initial and final acceleration are zero!
    t_max = getMaxAccelerationTimes(idx);
    for (size_t i = 0; i < t_max.size(); i++)
    {
        // std::cout << "\t Max. acceleration.\t t_max: " << t_max[i] << "\t value: " << std::abs(getAcceleration(t_max[i], idx, t_f)) << "\n";
        if (std::abs(getAcceleration(t_max[i], idx, t_f)) > robot->getMaxAcc(idx))
        {
            // std::cout << "\t Maximal acceleration constraint not satisfied! \n";
            return false;
        }
    }

    // Maximal velocity constraint
    // Note: Initial and final velocity are zero!
    t_max = getMaxVelocityTimes(idx);
    for (size_t i = 0; i < t_max.size(); i++)
    {
        // std::cout << "\t Max. velocity.\t t_max: " << t_max[i] << "\t value: " << std::abs(getVelocity(t_max[i], idx, t_f)) << "\n";
        if (std::abs(getVelocity(t_max[i], idx, t_f)) > robot->getMaxVel(idx))
        {
            // std::cout << "\t Maximal velocity constraint not satisfied! \n";
            return false;
        }
    }

    // std::cout << "\t All constraints are satisfied! \n";
    return true;
}

std::vector<float> planning::trajectory::Spline5::getMaxVelocityTimes(size_t idx)
{
    return solveQubicEquation(20*a(idx), 12*b(idx), 6*c(idx), 2*d(idx));
}

std::vector<float> planning::trajectory::Spline5::getMaxAccelerationTimes(size_t idx)
{
    std::vector<float> t_max {};

    if (a(idx) != 0)
    {
        float D = 576*b(idx)*b(idx) - 1440*a(idx)*c(idx);
        if (D >= 0)
        {
            for (int sign : {-1, 1})
                t_max.emplace_back((-24*b(idx) + sign*std::sqrt(D)) / (120*a(idx)));
        }
    }

    return t_max;
}

std::vector<float> planning::trajectory::Spline5::getMaxJerkTimes(size_t idx)
{
    std::vector<float> t_max {};

    if (a(idx) != 0)
        t_max.emplace_back(-b(idx) / (5*a(idx)));

    return t_max;
}

float planning::trajectory::Spline5::getPosition(float t, size_t idx, float t_f)
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
    
    return f(idx) + e(idx)*t + d(idx)*t*t + c(idx)*t*t*t + b(idx)*t*t*t*t + a(idx)*t*t*t*t*t 
           + delta_t * vel_final;
}

float planning::trajectory::Spline5::getVelocity(float t, size_t idx, float t_f)
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

    return e(idx) + 2*d(idx)*t + 3*c(idx)*t*t + 4*b(idx)*t*t*t + 5*a(idx)*t*t*t*t 
           + delta_t * acc_final;
}

float planning::trajectory::Spline5::getAcceleration(float t, size_t idx, float t_f)
{
    if (t > t_f)
        t = t_f;
    else if (t < 0)
        t = 0;

    return 2*d(idx) + 6*c(idx)*t + 12*b(idx)*t*t + 20*a(idx)*t*t*t;
}

float planning::trajectory::Spline5::getJerk(float t, size_t idx, float t_f)
{
    if (t > t_f)
        t = t_f;
    else if (t < 0)
        t = 0;

    return 6*c(idx) + 24*b(idx)*t + 60*a(idx)*t*t;
}

/// @brief Solve a cubic equation a*t³ + b*t² + c*t + d = 0
/// @param a real coefficient (different from zero) next to t³
/// @param b real coefficient next to t²
/// @param c real coefficient next to t¹
/// @param d real coefficient next to t⁰
/// @return Only real solutions
/// @note Code copied from https://cplusplus.com/forum/beginner/234717/
const std::vector<float> planning::trajectory::Spline5::solveQubicEquation(float a, float b, float c, float d)
{
    // 1. option: Using Eigen
    // std::chrono::steady_clock::time_point time_start_ { std::chrono::steady_clock::now() };
    // Eigen::VectorXd poly(4);
    // poly << d, c, b, a;
    // Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
    // solver.compute(poly);
    // const Eigen::PolynomialSolver<double, Eigen::Dynamic>::RootsType &r = solver.roots();
    // std::cout << "Elapsed time for solving cubic equation (with Eigen): " << 
    //     std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - time_start_).count() << " [ns] \n";
    // std::cout << "Eigen solutions: " << r.transpose() << "\n";


    // 2. option: Without using Eigen
    // std::chrono::steady_clock::time_point time_start_ { std::chrono::steady_clock::now() };
    std::vector<float> roots {};

    // Reduced equation: X^3 - 3pX - 2q = 0, where X = x-b/(3a)
    float p = (b * b - 3.0 * a * c) / (9.0 * a * a);
    float q = (9.0 * a * b * c - 27.0 * a * a * d - 2.0 * b * b * b) / (54.0 * a * a * a);
    float offset = b / (3.0 * a);

    // Discriminant
    float discriminant = p * p * p - q * q;
    // std::cout << "discriminant: " << discriminant << "\n";

    if (discriminant > 0)           // All real roots
    {
        float theta = acos(q / (p * sqrt(p)));
        float r = 2.0 * sqrt(p);
        for (size_t n = 0; n < 3; n++)
        {
            roots.emplace_back(r * cos((theta + 2.0 * n * M_PI) / 3.0) - offset);
            // std::cout << roots[n] << "\n";
        }
    }
    else 
    {
        float gamma1 = cbrt(q + sqrt(-discriminant));
        float gamma2 = cbrt(q - sqrt(-discriminant));

        roots.emplace_back(gamma1 + gamma2 - offset);
        // std::cout << roots[0] << "\n";

        float re = -0.5 * (gamma1 + gamma2) - offset;
        // float im = (gamma1 - gamma2) * static_cast<float>(sqrt(3.0) / 2.0);
        if (abs(discriminant) < 1e-16)                // Equal roots
        {
            roots.emplace_back(re);
            // std::cout << re << "\n";
            // std::cout << re << "\n";
        }
        // else     // Complex roots
        // {
        //     std::cout << re << " + " << im << " i \n";
        //     std::cout << re << " - " << im << " i \n";
        // }
    }
    
    // std::cout << "Elapsed time for solving cubic equation: " << 
    //     std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - time_start_).count() << " [ns] \n";
    return roots;
}
