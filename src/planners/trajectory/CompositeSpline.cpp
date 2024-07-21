#include "CompositeSpline.h"

planning::trajectory::CompositeSpline::CompositeSpline(const std::vector<std::shared_ptr<planning::trajectory::Spline>> &splines_)
{
    splines = splines_;
    time_start = std::chrono::steady_clock::now();
    time_start_offset = 0;
    time_current = 0;
    time_begin = 0;
    time_end = 0;
    is_zero_final_vel = splines.back()->getIsZeroFinalVel();
    is_zero_final_acc = splines.back()->getIsZeroFinalAcc();

    float time_final_sum { 0 };
    for (size_t i = 0; i < splines.size(); i++)
        time_final_sum += splines[i]->getTimeFinal();
    time_final = time_final_sum;
}

planning::trajectory::CompositeSpline::~CompositeSpline() {}

size_t planning::trajectory::CompositeSpline::findSplineIdx(float t)
{
    float time_final_sum { 0 };
    for (size_t i = 0; i < splines.size(); i++)
    {
        if (t < splines[i]->getTimeFinal() + time_final_sum)
            return i;
        else
            time_final_sum += splines[i]->getTimeFinal();
    }

    return splines.size()-1;
}

Eigen::VectorXf planning::trajectory::CompositeSpline::getPosition(float t)
{
    size_t idx { findSplineIdx(t) };
    float t_offset { idx > 0 ? splines[idx-1]->getTimeFinal() : 0 };
    std::cout << "Finding idx in composite spline... \t";
    std::cout << "idx: " << idx << "\t";
    std::cout << "t_offset: " << t_offset << "\t";
    std::cout << "time: " << t - t_offset << "\t";
    std::cout << "pos: " << splines[idx]->getPosition(t - t_offset).transpose() << "\n";
    return splines[idx]->getPosition(t - t_offset);
}

Eigen::VectorXf planning::trajectory::CompositeSpline::getVelocity(float t)
{
    size_t idx { findSplineIdx(t) };
    float t_offset { idx > 0 ? splines[idx-1]->getTimeFinal() : 0 };
    return splines[idx]->getVelocity(t - t_offset);
}

Eigen::VectorXf planning::trajectory::CompositeSpline::getAcceleration(float t)
{
    size_t idx { findSplineIdx(t) };
    float t_offset { idx > 0 ? splines[idx-1]->getTimeFinal() : 0 };
    return splines[idx]->getAcceleration(t - t_offset);
}

Eigen::VectorXf planning::trajectory::CompositeSpline::getJerk(float t)
{
    size_t idx { findSplineIdx(t) };
    float t_offset { idx > 0 ? splines[idx-1]->getTimeFinal() : 0 };
    return splines[idx]->getJerk(t - t_offset);
}
