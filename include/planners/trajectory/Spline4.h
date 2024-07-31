//
// Created by nermin on 05.06.24.
//

#ifndef RPMPL_SPLINE4_H
#define RPMPL_SPLINE4_H

#include "Spline.h"

namespace planning::trajectory
{
    class Spline4 : public Spline
    {
    public:
        Spline4(const std::shared_ptr<robots::AbstractRobot> robot_, const Eigen::VectorXf &q_current);
        Spline4(const std::shared_ptr<robots::AbstractRobot> robot_, const Eigen::VectorXf &q_current, 
                const Eigen::VectorXf &q_current_dot, const Eigen::VectorXf &q_current_ddot);
        ~Spline4();

        bool compute() override;
        bool compute(const Eigen::VectorXf &q_final_dot) override;
        bool compute(const Eigen::VectorXf &q_final_dot, const Eigen::VectorXf &q_final_ddot) override;
        bool compute([[maybe_unused]] const Eigen::VectorXf &q_final_dot, [[maybe_unused]] const Eigen::VectorXf &q_final_ddot, 
                     [[maybe_unused]] const Eigen::VectorXf &q_temp) override { return false; }
        bool checkConstraints(size_t idx, float t_f) override;

        std::vector<float> getPositionExtremumTimes(size_t idx) override;
        std::vector<float> getVelocityExtremumTimes(size_t idx) override;
        std::vector<float> getAccelerationExtremumTimes(size_t idx) override;
        std::vector<float> getJerkExtremumTimes([[maybe_unused]] size_t idx) override { return std::vector<float>(); }

    private:
        float computeFinalTime(size_t idx, float q_f_dot, float q_f_ddot, bool check_all_sol = false);
        float compute_a(size_t idx, float t_f, float q_f_ddot);
        float compute_b(size_t idx, float t_f, float q_f_dot, float q_f_ddot);
        float getPosition(float t, size_t idx, float t_f);
        float getVelocity(float t, size_t idx, float t_f);
        float getAcceleration(float t, size_t idx, float t_f);
        float getJerk(float t, size_t idx, float t_f);

        Eigen::VectorXf a, b, c, d, e;   // Coefficients of a spline a*t⁴ + b*t³ + c*t² + d*t + e
    };
}

#endif //RPMPL_SPLINE4_H