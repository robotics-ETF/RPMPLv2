//
// Created by nermin on 13.02.24.
//
#ifndef RPMPL_SPLINE5_H
#define RPMPL_SPLINE5_H

#include "Spline.h"

namespace planning
{
    namespace trajectory
    {
        class Spline5 : public Spline
        {
        public:
            Spline5(const std::shared_ptr<robots::AbstractRobot> robot_, const Eigen::VectorXf &q_current);
            Spline5(const std::shared_ptr<robots::AbstractRobot> robot_, const Eigen::VectorXf &q_current, 
                    const Eigen::VectorXf &q_current_dot, const Eigen::VectorXf &q_current_ddot);
		    ~Spline5() {}

            bool compute() override { return false; }
            bool compute(const Eigen::VectorXf &q_final) override;
            bool compute(const Eigen::VectorXf &q_final, const Eigen::VectorXf &q_final_dot) override;
            bool compute(const Eigen::VectorXf &q_final, const Eigen::VectorXf &q_final_dot, const Eigen::VectorXf &q_final_ddot) override;
            bool checkConstraints(size_t idx, float t_f) override;

            std::vector<float> getMaxVelocityTimes(size_t idx) override;
            std::vector<float> getMaxAccelerationTimes(size_t idx) override;
            std::vector<float> getMaxJerkTimes(size_t idx) override;

            float getPosition(float t, size_t idx, float t_f) override;
            float getVelocity(float t, size_t idx, float t_f) override;
            float getAcceleration(float t, size_t idx, float t_f) override;
            float getJerk(float t, size_t idx, float t_f) override;

        private:
            float computeFinalTime(size_t idx, float q_f, float q_f_dot, float q_f_ddot);
            float compute_a(size_t idx, float t_f, float q_f_ddot);
            float compute_b(size_t idx, float t_f, float q_f_dot, float q_f_ddot);
            float compute_c(size_t idx, float t_f, float q_f, float q_f_dot, float q_f_ddot);
            const std::vector<float> solveQubicEquation(float a, float b, float c, float d);
            
            Eigen::VectorXf a, b, c, d, e, f;   // Coefficients of a spline a*t⁵ + b*t⁴ + c*t³ + d*t² + e*t + f
        };
        
    }
}

#endif //RPMPL_SPLINE5_H