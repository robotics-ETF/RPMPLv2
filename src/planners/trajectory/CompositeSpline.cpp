#include "CompositeSpline.h"

planning::trajectory::CompositeSpline::CompositeSpline(const std::vector<std::shared_ptr<planning::trajectory::Spline>> &subsplines_)
{
    subsplines = subsplines_;
    time_start = std::chrono::steady_clock::now();
    time_start_offset = 0;
    time_current = 0;
    time_begin = 0;
    time_end = 0;
    is_zero_final_vel = subsplines.back()->getIsZeroFinalVel();
    is_zero_final_acc = subsplines.back()->getIsZeroFinalAcc();

    float time_final_sum { 0 };
    for (size_t i = 0; i < subsplines.size(); i++)
        time_final_sum += subsplines[i]->getTimeFinal();
    time_final = time_final_sum;
}

planning::trajectory::CompositeSpline::~CompositeSpline() {}

size_t planning::trajectory::CompositeSpline::findSubsplineIdx(float t)
{
    float time_final_sum { 0 };
    for (size_t i = 0; i < subsplines.size(); i++)
    {
        if (t < subsplines[i]->getTimeFinal() + time_final_sum)
            return i;
        else
            time_final_sum += subsplines[i]->getTimeFinal();
    }

    return subsplines.size()-1;
}

Eigen::VectorXf planning::trajectory::CompositeSpline::getPosition(float t)
{
    size_t idx { findSubsplineIdx(t) };
    float t_offset { idx > 0 ? subsplines[idx-1]->getTimeFinal() : 0 };
    return subsplines[idx]->getPosition(t - t_offset);
}

Eigen::VectorXf planning::trajectory::CompositeSpline::getVelocity(float t)
{
    size_t idx { findSubsplineIdx(t) };
    float t_offset { idx > 0 ? subsplines[idx-1]->getTimeFinal() : 0 };
    return subsplines[idx]->getVelocity(t - t_offset);
}

Eigen::VectorXf planning::trajectory::CompositeSpline::getAcceleration(float t)
{
    size_t idx { findSubsplineIdx(t) };
    float t_offset { idx > 0 ? subsplines[idx-1]->getTimeFinal() : 0 };
    return subsplines[idx]->getAcceleration(t - t_offset);
}

Eigen::VectorXf planning::trajectory::CompositeSpline::getJerk(float t)
{
    size_t idx { findSubsplineIdx(t) };
    float t_offset { idx > 0 ? subsplines[idx-1]->getTimeFinal() : 0 };
    return subsplines[idx]->getJerk(t - t_offset);
}
