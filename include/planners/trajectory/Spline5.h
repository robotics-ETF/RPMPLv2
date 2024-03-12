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

            bool compute(const Eigen::VectorXf &q_final) override;
            bool checkConstraints(int idx, float t_f) override;

            float getPosition(float t, int idx, float t_f) override;
            float getVelocity(float t, int idx, float t_f) override;
            float getAcceleration(float t, int idx, float t_f) override;
            float getJerk(float t, int idx, float t_f) override;

        private:
            float computeFinalTime(int idx, float q_f_i);
            const std::vector<float> solveQubicEquation(float a, float b, float c, float d);
            
            Eigen::VectorXf a, b, c, d, e, f;   // Coefficients of a spline a*t⁵ + b*t⁴ + c*t³ + d*t² + e*t + f
        };
        
    }
}

#endif //RPMPL_SPLINE5_H