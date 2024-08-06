#include "Spline.h"

planning::trajectory::Spline::Spline(size_t order_, const std::shared_ptr<robots::AbstractRobot> robot_, const Eigen::VectorXf &q_current)
{
    order = order_;
    robot = robot_;
    coeff = Eigen::MatrixXf::Zero(robot->getNumDOFs(), order + 1);
    coeff.col(0) = q_current;   // All initial conditions are zero, except position
    time_start = std::chrono::steady_clock::now();
    time_start_offset = 0;
    times_final = std::vector<float>(robot->getNumDOFs(), 0);
    time_final = 0;
    time_current = 0;
    time_begin = 0;
    time_end = 0;
    is_zero_final_vel = true;
    is_zero_final_acc = true;
    subsplines = {};
}

planning::trajectory::Spline::~Spline() {}

/// @brief Check whether 'q' is a final configuration of the spline.
/// @param q Configuration to be checked.
/// @return True if yes, false if not.
bool planning::trajectory::Spline::isFinalConf(const Eigen::VectorXf &q)
{
    return ((q - getPosition(time_final)).norm() < RealVectorSpaceConfig::EQUALITY_THRESHOLD) ? true : false;
}

/// @brief Check whether position function is monotonic.
/// @param idx Index of robot's joint.
/// @return Return 1 if the function is monotonically increasing.
/// @return Return -1 if the function is monotonically decreasing.
/// @return Return 0 if the function is non monotonic.
int planning::trajectory::Spline::checkPositionMonotonicity(size_t idx)
{
    float value { getVelocity(SplinesConfig::TIME_STEP, idx) };
    int monotonic { value > 0 ? 1 : -1 };

    for (float t = 2*SplinesConfig::TIME_STEP; t <= times_final[idx]; t += SplinesConfig::TIME_STEP)
    {
        value = getVelocity(t, idx);
        if ((value > 0 && monotonic == -1) || (value < 0 && monotonic == 1))
            return 0;
    }

    return monotonic;
}

/// @brief Check whether position function is monotonic considering all spline functions.
/// @return Return 1 if the function is monotonically increasing.
/// @return Return -1 if the function is monotonically decreasing.
/// @return Return 0 if the function is non monotonic.
int planning::trajectory::Spline::checkPositionMonotonicity()
{
    for (size_t i = 0; i < robot->getNumDOFs(); i++)
    {
        if (checkPositionMonotonicity(i) == 0)
            return false;
    }

    return true;
}

Eigen::VectorXf planning::trajectory::Spline::getPosition(float t)
{
    Eigen::VectorXf q { Eigen::VectorXf::Zero(robot->getNumDOFs()) };
    for (size_t i = 0; i < robot->getNumDOFs(); i++)
        q(i) = getPosition(t, i);

    // std::cout << "Robot position at time " << t << " [s] is " << q.transpose() << "\n";
    return q;
}

float planning::trajectory::Spline::getPosition(float t, size_t idx)
{
    float q { 0 };
    float delta_t { 0 };
    float vel_final { 0 };
    float acc_final { 0 };

    if (t > times_final[idx])
    {
        if (!is_zero_final_vel && is_zero_final_acc)
        {
            delta_t = t - times_final[idx];
            vel_final = getVelocity(t, idx);
        }
        else if (!is_zero_final_acc)
        {
            delta_t = t - times_final[idx];
            acc_final = getAcceleration(t, idx);
        }
        t = times_final[idx];
    }
    else if (t < 0)
        t = 0;
    
    for (size_t i = 0; i <= order; i++)
        q += coeff(idx, i) * std::pow(t, i);

    return q + vel_final * delta_t + acc_final * delta_t*delta_t * 0.5;
}

Eigen::VectorXf planning::trajectory::Spline::getVelocity(float t)
{
    Eigen::VectorXf q { Eigen::VectorXf::Zero(robot->getNumDOFs()) };
    for (size_t i = 0; i < robot->getNumDOFs(); i++)
        q(i) = getVelocity(t, i);

    // std::cout << "Robot velocity at time " << t << " [s] is " << q.transpose() << "\n";
    return q;
}

float planning::trajectory::Spline::getVelocity(float t, size_t idx)
{
    float q { 0 };
    float delta_t { 0 };
    float acc_final { 0 };

    if (t > times_final[idx])
    {
        if (!is_zero_final_acc)
        {
            delta_t = t - times_final[idx];
            acc_final = getAcceleration(t, idx);
        }
        t = times_final[idx];
    }
    else if (t < 0)
        t = 0;

    for (size_t i = 1; i <= order; i++)
        q += coeff(idx, i) * i * std::pow(t, i-1);

    return q + acc_final * delta_t;
}

Eigen::VectorXf planning::trajectory::Spline::getAcceleration(float t)
{
    Eigen::VectorXf q { Eigen::VectorXf::Zero(robot->getNumDOFs()) };
    for (size_t i = 0; i < robot->getNumDOFs(); i++)
        q(i) = getAcceleration(t, i);

    // std::cout << "Robot acceleration at time " << t << " [s] is " << q.transpose() << "\n";
    return q;
}

float planning::trajectory::Spline::getAcceleration(float t, size_t idx)
{
    if (t > times_final[idx])
        t = times_final[idx];
    else if (t < 0)
        t = 0;

    float q { 0 };
    for (size_t i = 2; i <= order; i++)
        q += coeff(idx, i) * i * (i-1) * std::pow(t, i-2);

    return q;
}

Eigen::VectorXf planning::trajectory::Spline::getJerk(float t)
{
    Eigen::VectorXf q { Eigen::VectorXf::Zero(robot->getNumDOFs()) };
    for (size_t i = 0; i < robot->getNumDOFs(); i++)
        q(i) = getJerk(t, i);

    // std::cout << "Robot jerk at time " << t << " [s] is " << q.transpose() << "\n";
    return q;
}

float planning::trajectory::Spline::getJerk(float t, size_t idx)
{
    if (t > times_final[idx])
        t = times_final[idx];
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
    
    time_current = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - time_start).count() * 1e-9 
                   - time_start_offset;
    return time_current;
}

void planning::trajectory::Spline::setTimeStart(float time_start_offset_)
{
    time_start = std::chrono::steady_clock::now();
    time_start_offset = time_start_offset_;
}

namespace planning::trajectory 
{
    std::ostream &operator<<(std::ostream &os, const std::shared_ptr<planning::trajectory::Spline> spline)
    {
        auto print = [&os](std::shared_ptr<planning::trajectory::Spline> s) -> void
        {
            for (size_t i = 0; i < s->robot->getNumDOFs(); i++)
            {
                os << "q_" << i << "(t) = ";
                for (size_t j = 0; j <= s->order; j++)
                    os << s->getCoeff(i, j) << " t^" << j << (j == s->order ? "" : " + ");
                
                os << "\t for t in [0, " << std::min(s->time_final, s->times_final[i]) << "] [s] \n";
            }
        };

        if (spline->subsplines.empty())
            print(spline);
        else
        {
            for (size_t k = 0; k < spline->subsplines.size(); k++)
            {
                os << "Subspline " << k << ":\n";
                print(spline->subsplines[k]);
            }
        }

        return os;
    }
}
