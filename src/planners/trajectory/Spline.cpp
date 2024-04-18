#include "Spline.h"

planning::trajectory::Spline::Spline(size_t order_, const std::shared_ptr<robots::AbstractRobot> robot_, const Eigen::VectorXf &q_current)
{
    order = order_;
    robot = robot_;
    num_dimensions = robot->getNumDOFs();
    coeff = Eigen::MatrixXf::Zero(num_dimensions, order + 1);
    coeff.col(0) = q_current;   // All initial conditions are zero, except position
    time_start = std::chrono::steady_clock::now();
    time_final = 0;
    time_current = 0;
    time_begin = 0;
    time_end = 0;
}

planning::trajectory::Spline::~Spline() {}

Eigen::VectorXf planning::trajectory::Spline::getPosition(float t)
{
    Eigen::VectorXf q { Eigen::VectorXf::Zero(num_dimensions) };
    for (size_t i = 0; i < num_dimensions; i++)
        q(i) = getPosition(t, i);

    // std::cout << "Robot position at time " << t << " [s] is " << q.transpose() << "\n";
    return q;
}

float planning::trajectory::Spline::getPosition(float t, size_t idx)
{
    if (t > time_final)
        t = time_final;
    else if (t < 0)
        t = 0;
    
    float q { 0 };
    for (size_t i = 0; i <= order; i++)
        q += coeff(idx, i) * std::pow(t, i);

    return q;
}

Eigen::VectorXf planning::trajectory::Spline::getVelocity(float t)
{
    Eigen::VectorXf q { Eigen::VectorXf::Zero(num_dimensions) };
    for (size_t i = 0; i < num_dimensions; i++)
        q(i) = getVelocity(t, i);

    // std::cout << "Robot velocity at time " << t << " [s] is " << q.transpose() << "\n";
    return q;
}

float planning::trajectory::Spline::getVelocity(float t, size_t idx)
{
    if (t > time_final)
        t = time_final;
    else if (t < 0)
        t = 0;

    float q { 0 };
    for (size_t i = 1; i <= order; i++)
        q += coeff(idx, i) * i * std::pow(t, i-1);

    return q;
}

Eigen::VectorXf planning::trajectory::Spline::getAcceleration(float t)
{
    Eigen::VectorXf q { Eigen::VectorXf::Zero(num_dimensions) };
    for (size_t i = 0; i < num_dimensions; i++)
        q(i) = getAcceleration(t, i);

    // std::cout << "Robot acceleration at time " << t << " [s] is " << q.transpose() << "\n";
    return q;
}

float planning::trajectory::Spline::getAcceleration(float t, size_t idx)
{
    if (t > time_final)
        t = time_final;
    else if (t < 0)
        t = 0;

    float q { 0 };
    for (size_t i = 2; i <= order; i++)
        q += coeff(idx, i) * i * (i-1) * std::pow(t, i-2);

    return q;
}

Eigen::VectorXf planning::trajectory::Spline::getJerk(float t)
{
    Eigen::VectorXf q { Eigen::VectorXf::Zero(num_dimensions) };
    for (size_t i = 0; i < num_dimensions; i++)
        q(i) = getJerk(t, i);

    // std::cout << "Robot jerk at time " << t << " [s] is " << q.transpose() << "\n";
    return q;
}

float planning::trajectory::Spline::getJerk(float t, size_t idx)
{
    if (t > time_final)
        t = time_final;
    else if (t < 0)
        t = 0;

    float q { 0 };
    for (size_t i = 3; i <= order; i++)
        q += coeff(idx, i) * i * (i-1) * (i-2) * std::pow(t, i-3);

    return q;
}

/// @brief Get current time of a spline.
/// @param measure_time If true, current time will be automatically computed/measured (default: false).
/// @note 'measure_time' should always be false when simulation pacing is used, since then a time measuring will not be correct! 
/// In such case, it is assumed that user was previously set 'measure_time' to a correct value.
float planning::trajectory::Spline::getTimeCurrent(bool measure_time)
{
    if (!measure_time)
        return time_current;
    
    time_current = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - time_start).count() * 1e-9;
    return time_current;
}

void planning::trajectory::Spline::setTimeStart()
{
    time_start = std::chrono::steady_clock::now();
}

namespace planning::trajectory 
{
    std::ostream &operator<<(std::ostream &os, const std::shared_ptr<planning::trajectory::Spline> spline)
    {
        for (size_t i = 0; i < spline->num_dimensions; i++)
        {
            os << "q_" << i << "(t) = ";
            for (size_t j = 0; j <= spline->order; j++)
                os << spline->getCoeff(i, j) << " t^" << j << (j == spline->order ? "" : " + ");
            
            os << "\t for t in [0, " << spline->time_final << "] [s] \n";
        }

        return os;
    }
}