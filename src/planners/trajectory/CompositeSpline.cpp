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

std::vector<float> planning::trajectory::CompositeSpline::getPositionExtremumTimes(size_t idx)
{
    std::vector<float> t_extrema {};
    for (size_t i = 0; i < subsplines.size(); i++)
    {
        for (float t : subsplines[i]->getPositionExtremumTimes(idx))
        {
            // This check is only because of possible overlapping of solutions from the previous and current subspline
            if (!t_extrema.empty() && std::abs(t - t_extrema.back()) > RealVectorSpaceConfig::EQUALITY_THRESHOLD)
                t_extrema.emplace_back(t);
        }
    }

    return t_extrema;
}

std::vector<float> planning::trajectory::CompositeSpline::getVelocityExtremumTimes(size_t idx)
{
    std::vector<float> t_extrema {};
    for (size_t i = 0; i < subsplines.size(); i++)
    {
        for (float t : subsplines[i]->getVelocityExtremumTimes(idx))
        {
            // This check is only because of possible overlapping of solutions from the previous and current subspline
            if (!t_extrema.empty() && std::abs(t - t_extrema.back()) > RealVectorSpaceConfig::EQUALITY_THRESHOLD)
                t_extrema.emplace_back(t);
        }
    }

    return t_extrema;
}

std::vector<float> planning::trajectory::CompositeSpline::getAccelerationExtremumTimes(size_t idx)
{
    std::vector<float> t_extrema {};
    for (size_t i = 0; i < subsplines.size(); i++)
    {
        for (float t : subsplines[i]->getAccelerationExtremumTimes(idx))
        {
            // This check is only because of possible overlapping of solutions from the previous and current subspline
            if (!t_extrema.empty() && std::abs(t - t_extrema.back()) > RealVectorSpaceConfig::EQUALITY_THRESHOLD)
                t_extrema.emplace_back(t);
        }
    }

    return t_extrema;
}

std::vector<float> planning::trajectory::CompositeSpline::getJerkExtremumTimes(size_t idx)
{
    std::vector<float> t_extrema {};
    for (size_t i = 0; i < subsplines.size(); i++)
    {
        for (float t : subsplines[i]->getJerkExtremumTimes(idx))
        {
            // This check is only because of possible overlapping of solutions from the previous and current subspline
            if (!t_extrema.empty() && std::abs(t - t_extrema.back()) > RealVectorSpaceConfig::EQUALITY_THRESHOLD)
                t_extrema.emplace_back(t);
        }
    }

    return t_extrema;
}

int planning::trajectory::CompositeSpline::checkPositionMonotonicity(size_t idx)
{
    int monotonic { subsplines.front()->checkPositionMonotonicity(idx) };
    for (size_t i = 1; i < subsplines.size(); i++)
    {
        if (subsplines[i]->checkPositionMonotonicity(idx) != monotonic)
            return 0;
    }

    return monotonic;
}
