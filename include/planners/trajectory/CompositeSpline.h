//
// Created by nermin on 20.07.24.
//
#ifndef RPMPL_COMPOSITESPLINE_H
#define RPMPL_COMPOSITESPLINE_H

#include "Spline.h"

namespace planning
{
    namespace trajectory
    {
        class CompositeSpline : public Spline
        {
        public:
            CompositeSpline(const std::vector<std::shared_ptr<planning::trajectory::Spline>> &subsplines_);
            ~CompositeSpline();

            size_t findSubsplineIdx(float t);
            Eigen::VectorXf getPosition(float t) override;
            Eigen::VectorXf getVelocity(float t) override;
            Eigen::VectorXf getAcceleration(float t) override;
            Eigen::VectorXf getJerk(float t) override;
            float getCoeff(size_t i, size_t j, size_t idx) const { return subsplines[idx]->getCoeff(i, j); }

            // Not used functions here:
            bool compute() override { return false; }
            bool compute([[maybe_unused]] const Eigen::VectorXf &q_final) override { return false; }
            bool compute([[maybe_unused]] const Eigen::VectorXf &q_final, [[maybe_unused]] const Eigen::VectorXf &q_final_dot) override { return false; }
            bool compute([[maybe_unused]] const Eigen::VectorXf &q_final, [[maybe_unused]] const Eigen::VectorXf &q_final_dot, 
                         [[maybe_unused]] const Eigen::VectorXf &q_final_ddot) override { return false; }
            bool checkConstraints([[maybe_unused]] size_t idx, [[maybe_unused]] float t_f) override { return false; }
            
            std::vector<float> getMaxVelocityTimes([[maybe_unused]] size_t idx) override { return std::vector<float>(); }
            std::vector<float> getMaxAccelerationTimes([[maybe_unused]] size_t idx) override { return std::vector<float>(); }
            std::vector<float> getMaxJerkTimes([[maybe_unused]] size_t idx) override { return std::vector<float>(); }

        };
    }
}

#endif //RPMPL_COMPOSITESPLINE_H