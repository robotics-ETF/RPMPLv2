#include "Spline.h"

planning::trajectory::Spline::Spline(int order_, const std::shared_ptr<robots::AbstractRobot> robot_, const Eigen::VectorXf &q_current)
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
    Eigen::VectorXf q = Eigen::VectorXf::Zero(num_dimensions);
    for (int i = 0; i < num_dimensions; i++)
        q(i) = getPosition(t, i);

    // std::cout << "Robot position at time " << t << " [s] is " << q.transpose() << "\n";
    return q;
}

float planning::trajectory::Spline::getPosition(float t, int idx)
{
    if (t < 0)
        t = 0;
    else if (t > time_final)
        t = time_final;
    
    float q = 0;
    for (int i = 0; i <= order; i++)
        q += coeff(idx, i) * std::pow(t, i);

    return q;
}

Eigen::VectorXf planning::trajectory::Spline::getVelocity(float t)
{
    Eigen::VectorXf q = Eigen::VectorXf::Zero(num_dimensions);
    for (int i = 0; i < num_dimensions; i++)
        q(i) = getVelocity(t, i);

    // std::cout << "Robot velocity at time " << t << " [s] is " << q.transpose() << "\n";
    return q;
}

float planning::trajectory::Spline::getVelocity(float t, int idx)
{
    float q = 0;
    if (t >= 0 && t <= time_final)
    {
        for (int i = 1; i <= order; i++)
            q += coeff(idx, i) * i * std::pow(t, i-1);
    }

    return q;
}

Eigen::VectorXf planning::trajectory::Spline::getAcceleration(float t)
{
    Eigen::VectorXf q = Eigen::VectorXf::Zero(num_dimensions);
    for (int i = 0; i < num_dimensions; i++)
        q(i) = getAcceleration(t, i);

    // std::cout << "Robot acceleration at time " << t << " [s] is " << q.transpose() << "\n";
    return q;
}

float planning::trajectory::Spline::getAcceleration(float t, int idx)
{
    float q = 0;
    if (t >= 0 && t <= time_final)
    {
        for (int i = 2; i <= order; i++)
            q += coeff(idx, i) * i * (i-1) * std::pow(t, i-2);
    }

    return q;
}

Eigen::VectorXf planning::trajectory::Spline::getJerk(float t)
{
    Eigen::VectorXf q = Eigen::VectorXf::Zero(num_dimensions);
    for (int i = 0; i < num_dimensions; i++)
        q(i) = getJerk(t, i);

    // std::cout << "Robot jerk at time " << t << " [s] is " << q.transpose() << "\n";
    return q;
}

float planning::trajectory::Spline::getJerk(float t, int idx)
{
    float q = 0;
    if (t >= 0 && t <= time_final)
    {
        for (int i = 3; i <= order; i++)
            q += coeff(idx, i) * i * (i-1) * (i-2) * std::pow(t, i-3);
    }

    return q;
}

void planning::trajectory::Spline::setTimeStart()
{
    time_start = std::chrono::steady_clock::now();
}

/// @brief Set current time.
/// @param time_current_ Time in [s]. If not passed, it will be automatically computed.
/// @note 'time_current_' should always be passed when simulation pacing is used, since then a time measuring will not be correct! 
void planning::trajectory::Spline::setTimeCurrent(float time_current_)
{
    if (time_current_ > 0)
        time_current = time_current_;
    else
    {
        std::chrono::steady_clock::time_point time_start_ = std::chrono::steady_clock::now();
        time_current = (std::chrono::duration_cast<std::chrono::nanoseconds>(time_start_ - time_start).count()) * 1e-9;
    }
}

namespace planning::trajectory {
std::ostream& operator<<(std::ostream &os, const std::shared_ptr<planning::trajectory::Spline> spline)
{
    for (int i = 0; i < spline->num_dimensions; i++)
    {
        os << "q_" << i << "(t) = ";
        for (int j = 0; j <= spline->order; j++)
            os << spline->getCoeff(i, j) << " t^" << j << (j == spline->order ? "" : " + ");
        
        os << "\t for t in [0, " << spline->time_final << "] [s] \n";
    }

    return os;
}
}