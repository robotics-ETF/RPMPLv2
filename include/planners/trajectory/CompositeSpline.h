//
// Created by nermin on 20.07.24.
//

#ifndef RPMPL_COMPOSITESPLINE_H
#define RPMPL_COMPOSITESPLINE_H

#include "Spline.h"

namespace planning::trajectory
{
    class CompositeSpline : public Spline
    {
    public:
        CompositeSpline(const std::vector<std::shared_ptr<planning::trajectory::Spline>> &subsplines_);
        ~CompositeSpline();

        float getCoeff(size_t i, size_t j, size_t idx) const { return subsplines[idx]->getCoeff(i, j); }
        size_t findSubsplineIdx(float t);

        Eigen::VectorXf getPosition(float t) override;
        float getPosition(float t, size_t idx) override;

        Eigen::VectorXf getVelocity(float t) override;
        float getVelocity(float t, size_t idx) override;

        Eigen::VectorXf getAcceleration(float t) override;
        float getAcceleration(float t, size_t idx) override;
        
        Eigen::VectorXf getJerk(float t) override;
        float getJerk(float t, size_t idx) override;
        
        std::vector<float> getPositionExtremumTimes(size_t idx) override;
        std::vector<float> getVelocityExtremumTimes(size_t idx) override;
        std::vector<float> getAccelerationExtremumTimes(size_t idx) override;
        std::vector<float> getJerkExtremumTimes(size_t idx) override;
        int checkPositionMonotonicity(size_t idx) override;

        // Not used functions here:
        bool compute() override { return false; }
        bool compute([[maybe_unused]] const Eigen::VectorXf &q_final) override { return false; }
        bool compute([[maybe_unused]] const Eigen::VectorXf &q_final, [[maybe_unused]] const Eigen::VectorXf &q_final_dot) override { return false; }
        bool compute([[maybe_unused]] const Eigen::VectorXf &q_final, [[maybe_unused]] const Eigen::VectorXf &q_final_dot, 
                     [[maybe_unused]] const Eigen::VectorXf &q_final_ddot) override { return false; }
        bool checkConstraints([[maybe_unused]] size_t idx, [[maybe_unused]] float t_f) override { return false; }

    private:
        std::vector<float> times_connecting;
    };
}

#endif //RPMPL_COMPOSITESPLINE_H