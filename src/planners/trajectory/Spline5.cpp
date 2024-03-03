#include "Spline5.h"

// #include <unsupported/Eigen/Polynomials>

planning::trajectory::Spline5::Spline5(const std::shared_ptr<base::StateSpace> ss_, const std::shared_ptr<base::State> q_current) :
    Spline(5, ss_, q_current) 
{
    a = b = c = d = e = Eigen::VectorXf::Zero(ss->num_dimensions);
    f = q_current->getCoord();
}

planning::trajectory::Spline5::Spline5(const std::shared_ptr<base::StateSpace> ss_, const std::shared_ptr<base::State> q_current, 
    const std::shared_ptr<base::State> q_current_dot, const std::shared_ptr<base::State> q_current_ddot) :
    Spline(5, ss_, q_current)
{
    a = b = c =  Eigen::VectorXf::Zero(ss->num_dimensions);
    d = q_current_ddot->getCoord() / 2;
    e = q_current_dot->getCoord();
    f = q_current->getCoord();
}

/// @brief Compute a quintic spline from 'q_current' to 'q_final' such that robot stops at 'q_final', where all constraints on
/// robot's maximal velocity, acceleration and jerk are satisfied.
/// @param q_final Final configuration in which the spline is ending.
/// @return Success of computing the spline.
bool planning::trajectory::Spline5::compute(const std::shared_ptr<base::State> q_final)
{
    // std::chrono::steady_clock::time_point time_start_ = std::chrono::steady_clock::now();
    int idx_corr = -1;
    float t_f_opt = -1;
    float t_f, t_f_left, t_f_right;
    Eigen::Vector3f abc_left, abc_right;    // a, b and c coefficients, respectively
    const int max_num_iter = 5;

    for (int idx = 0; idx < ss->num_dimensions; idx++)
    {
        std::cout << "Joint: " << idx << " ---------------------------------------------------\n";
        std::cout << "Init. pos: " << f(idx) << "\t Final pos: " << q_final->getCoord(idx) <<  "\n";
        std::cout << "Init. vel: " << e(idx) << "\t Final vel: 0 \n";
        std::cout << "Init. acc: " << 2*d(idx) << "\t Final acc: 0 \n";
        std::cout << "t_f_opt:   " << t_f_opt << "\n";

        if (f(idx) == q_final->getCoord(idx) && e(idx) == 0 && d(idx) == 0)    // Special case
        {
            std::cout << "Joint position does not change. Just continue! \n";
            continue;
        }

        if (t_f_opt > 0)
        {
            t_f = t_f_opt;
            c(idx) = (10*(q_final->getCoord(idx) - f(idx)) - 6*e(idx)*t_f - 3*d(idx)*t_f*t_f) / (t_f*t_f*t_f);
            b(idx) = -(3*c(idx)*t_f*t_f + 3*d(idx)*t_f + 2*e(idx)) / (2*t_f*t_f*t_f);
            a(idx) = -(6*b(idx)*t_f*t_f + 3*c(idx)*t_f + d(idx)) / (10*t_f*t_f*t_f);
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

        c(idx) = -ss->robot->getMaxJerk(idx) / 6;
        t_f_left = computeFinalTime(idx, q_final->getCoord(idx));
        abc_left << a(idx), b(idx), c(idx);

        c(idx) = ss->robot->getMaxJerk(idx) / 6;
        t_f_right = computeFinalTime(idx, q_final->getCoord(idx));
        abc_right << a(idx), b(idx), c(idx);

        // std::cout << "t_f_left: " << t_f_left << "\t t_f_right: " << t_f_right << "\n";
        if (t_f_left == INFINITY && t_f_right == INFINITY || t_f_left == 0 && t_f_right == 0)
        {
            // std::cout << "No solution! 'q_final' must be changed! \n";
            return false;
        }
        else if (t_f_left > 0 && t_f_left < INFINITY && t_f_right > 0 && t_f_right < INFINITY)
        {
            std::cout << "Solution is found! Just continue! \n";
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
            std::cout << "Solution is found! Just continue! \n";
            t_f_opt = t_f_left;
            a(idx) = abc_left(0); b(idx) = abc_left(1); c(idx) = abc_left(2);
            continue;
        }
        else if (t_f_right > 0 && t_f_right < INFINITY)
        {
            std::cout << "Solution is found! Just continue! \n";
            t_f_opt = t_f_right;
            a(idx) = abc_right(0); b(idx) = abc_right(1); c(idx) = abc_right(2);
            continue;
        }

        // Using bisection method to find t_f, when t_f_left = 0 and t_f_right = inf, or vice versa
        bool found = false;
        for (int num = 0; num < max_num_iter; num++)
        {
            // std::cout << "Num. iter: " << num << " --------------------------\n";
            // std::cout << "c_left: " << abc_left(2) << "\t c_right: " << abc_right(2) << "\n";
            c(idx) = (abc_left(2) + abc_right(2)) / 2;
            t_f = computeFinalTime(idx, q_final->getCoord(idx));

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
        std::cout << "Correcting joint: " << idx << " ---------------------------------------------------\n";
        c(idx) = (10*(q_final->getCoord(idx) - f(idx)) - 6*e(idx)*t_f - 3*d(idx)*t_f*t_f) / (t_f*t_f*t_f);
        b(idx) = -(3*c(idx)*t_f*t_f + 3*d(idx)*t_f + 2*e(idx)) / (2*t_f*t_f*t_f);
        a(idx) = -(6*b(idx)*t_f*t_f + 3*c(idx)*t_f + d(idx)) / (10*t_f*t_f*t_f);
    }

    // Solution is found. Set the parameters for a new spline
    time_final = t_f_opt;
    for (int idx = 0; idx < ss->num_dimensions; idx++)
        coeff.row(idx) << f(idx), e(idx), d(idx), c(idx), b(idx), a(idx);
    
    // int t_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start_).count();
    // std::cout << "Elapsed time: " << t_elapsed << " [us] \n";

    return true;
}

/// @brief Compute a final time, i.e., a time to reach from 'q_current(idx)' to 'q_final(idx)'.
/// @param idx Index of robot's joint
/// @param q_f_i Final 'idx'-th desired configuration
/// @return Final time. If final time is zero, it means that constraints are not satisfied.
/// If final time is infinite, it means there is no solution.
float planning::trajectory::Spline5::computeFinalTime(int idx, float q_f_i)
{
    float t_f = INFINITY;
    std::vector<float> t_sol = solveQubicEquation(c(idx), 3*d(idx), 6*e(idx), 10*(f(idx) - q_f_i));

    std::cout << "For c: " << c(idx) << ", it follows t_f: ";
    for (int i = 0; i < t_sol.size(); i++)
    {
        std::cout << t_sol[i] << "\t";
        if (t_sol[i] > 0)
            t_f = std::min(t_f, t_sol[i]);
    }
    std::cout << "\n";

    if (t_f == INFINITY)
        return INFINITY;

    b(idx) = -(3*c(idx)*t_f*t_f + 3*d(idx)*t_f + 2*e(idx)) / (2*t_f*t_f*t_f);
    a(idx) = -(6*b(idx)*t_f*t_f + 3*c(idx)*t_f + d(idx)) / (10*t_f*t_f*t_f);

    // std::cout << "a: " << a(idx) << ",\t b: " << b(idx) << ",\t c: " << c(idx) << ",\t " 
    //           << "d: " << d(idx) << ",\t e: " << e(idx) << ",\t f: " << f(idx) << "\n";
    
    if (!checkConstraints(idx, t_f))
        return 0;

    return t_f;
}

bool planning::trajectory::Spline5::checkConstraints(int idx, float t_f)
{
    // Maximal jerk constraint
    float t_max = -b(idx) / (5*a(idx));
    // std::cout << "\t Max. jerk.\t t_f: " << 0 << "\t value: " << 6 * std::abs(c(idx)) << "\n";
    // std::cout << "\t Max. jerk.\t t_f: " << t_f << "\t value: " << std::abs(getJerk(t_f, idx, t_f)) << "\n";
    // std::cout << "\t Max. jerk.\t t_max: " << t_max << "\t value: " << std::abs(getJerk(t_max, idx, t_f)) << "\n";
    if (6 * std::abs(c(idx)) > ss->robot->getMaxJerk(idx) + 1e-3 ||
        std::abs(getJerk(t_f, idx, t_f)) > ss->robot->getMaxJerk(idx) + 1e-3 || 
        a(idx) != 0 && t_max > 0 && t_max < t_f && std::abs(getJerk(t_max, idx, t_f)) > ss->robot->getMaxJerk(idx))
    {
        std::cout << "\t Maximal jerk constraint not satisfied! \n";
        return false;
    }

    // Maximal acceleration constraint
    // Note: Initial and final acceleration is zero!
    float D = 576*b(idx)*b(idx) - 1440*a(idx)*c(idx);
    if (a(idx) != 0 && D >= 0)
    {
        for (int sign : {-1, 1})
        {
            t_max = (-24*b(idx) + sign*std::sqrt(D)) / (120*a(idx));
            // std::cout << "\t Max. acceleration.\t t_max: " << t_max << "\t value: " << std::abs(getAcceleration(t_max, idx, t_f)) << "\n";
            if (t_max > 0 && t_max < t_f && std::abs(getAcceleration(t_max, idx, t_f)) > ss->robot->getMaxAcc(idx))
            {
                std::cout << "\t Maximal acceleration constraint not satisfied! \n";
                return false;
            }
        }
    }

    // Maximal velocity constraint
    // Note: Initial and final velocity is zero!
    std::vector<float> t_max_ = solveQubicEquation(20*a(idx), 12*b(idx), 6*c(idx), 2*d(idx));
    for (int i = 0; i < t_max_.size(); i++)
    {
        // std::cout << "\t Max. velocity.\t t_max: " << t_max_[i] << "\t value: " << std::abs(getVelocity(t_max_[i], idx, t_f)) << "\n";
        if (t_max_[i] > 0 && t_max_[i] < t_f && std::abs(getVelocity(t_max_[i], idx, t_f)) > ss->robot->getMaxVel(idx))
        {
            std::cout << "\t Maximal velocity constraint not satisfied! \n";
            return false;
        }
    }

    std::cout << "\t All constraints are satisfied! \n";
    return true;
}

float planning::trajectory::Spline5::getPosition(float t, int idx, float t_f)
{
    if (t < 0)
        t = 0;
    else if (t > t_f)
        t = t_f;
    
    return f(idx) + e(idx)*t + d(idx)*t*t + c(idx)*t*t*t + b(idx)*t*t*t*t + a(idx)*t*t*t*t*t;
}

float planning::trajectory::Spline5::getVelocity(float t, int idx, float t_f)
{
    if (t >= 0 && t <= t_f)
        return e(idx) + 2*d(idx)*t + 3*c(idx)*t*t + 4*b(idx)*t*t*t + 5*a(idx)*t*t*t*t;

    return 0;
}

float planning::trajectory::Spline5::getAcceleration(float t, int idx, float t_f)
{
    if (t >= 0 && t <= t_f)
        return 2*d(idx) + 6*c(idx)*t + 12*b(idx)*t*t + 20*a(idx)*t*t*t;

    return 0;
}

float planning::trajectory::Spline5::getJerk(float t, int idx, float t_f)
{
    if (t >= 0 && t <= t_f)
        return 6*c(idx) + 24*b(idx)*t + 60*a(idx)*t*t;

    return 0;
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
    // std::chrono::steady_clock::time_point time_start_ = std::chrono::steady_clock::now();
    // Eigen::VectorXd poly(4);
    // poly << d, c, b, a;
    // Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
    // solver.compute(poly);
    // const Eigen::PolynomialSolver<double, Eigen::Dynamic>::RootsType &r = solver.roots();
    // std::cout << "Elapsed time for solving cubic equation (with Eigen): " << 
    //     std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - time_start_).count() << " [ns] \n";
    // std::cout << "Eigen solutions: " << r.transpose() << "\n";


    // 2. option: Without using Eigen
    // std::chrono::steady_clock::time_point time_start_ = std::chrono::steady_clock::now();
    std::vector<float> roots;

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
        for (int n = 0; n < 3; n++)
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
        float im = (gamma1 - gamma2) * sqrt(3.0) / 2.0;
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
