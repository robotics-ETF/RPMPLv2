//
// Created by nermin on 13.02.24.
//
#ifndef RPMPL_SPLINE_H
#define RPMPL_SPLINE_H

#include <chrono>

#include "AbstractRobot.h"
#include "RealVectorSpaceConfig.h"

namespace planning
{
    namespace trajectory
    {
        class Spline
        {
        public:
            Spline(size_t order_, const std::shared_ptr<robots::AbstractRobot> robot_, const Eigen::VectorXf &q_current);
		    virtual ~Spline() = 0;

            virtual bool compute(const Eigen::VectorXf &q_final) = 0;
            virtual bool compute(const Eigen::VectorXf &q_final, const Eigen::VectorXf &q_final_dot) = 0;
            virtual bool compute(const Eigen::VectorXf &q_final, const Eigen::VectorXf &q_final_dot, const Eigen::VectorXf &q_final_ddot) = 0;
            virtual bool checkConstraints(size_t idx, float t_f) = 0;
            bool isFinalConf(const Eigen::VectorXf &q);
            
            virtual std::vector<float> getMaxVelocityTimes(size_t idx) = 0;
            virtual std::vector<float> getMaxAccelerationTimes(size_t idx) = 0;
            virtual std::vector<float> getMaxJerkTimes(size_t idx) = 0;

            Eigen::VectorXf getPosition(float t);
            float getPosition(float t, size_t idx);
            virtual float getPosition(float t, size_t idx, float t_f) = 0;

            Eigen::VectorXf getVelocity(float t);
            float getVelocity(float t, size_t idx);
            virtual float getVelocity(float t, size_t idx, float t_f) = 0;

            Eigen::VectorXf getAcceleration(float t);
            float getAcceleration(float t, size_t idx);
            virtual float getAcceleration(float t, size_t idx, float t_f) = 0;

            Eigen::VectorXf getJerk(float t);
            float getJerk(float t, size_t idx);
            virtual float getJerk(float t, size_t idx, float t_f) = 0;

            float getCoeff(size_t i, size_t j) const { return coeff(i, j); }
            float getTimeFinal() const { return time_final; }
            float getTimeCurrent(bool measure_time = false);
            float getTimeBegin() const { return time_begin; }
            float getTimeEnd() const { return time_end; }

            void setTimeStart();
            void setTimeCurrent(float time_current_) { time_current = time_current_; }
            void setTimeBegin(float time_begin_) { time_begin = time_begin_; }
            void setTimeEnd(float time_end_) { time_end = time_end_; }

            friend std::ostream &operator<<(std::ostream &os, const std::shared_ptr<planning::trajectory::Spline> spline);

        protected:            
            size_t order;
            size_t num_dimensions;
            std::shared_ptr<robots::AbstractRobot> robot;
            Eigen::MatrixXf coeff;                              // Num. of rows = 'num_DOFs', and num. of columns = 'order+1'. Form: sum{j=0, num_DOFs-1} coeff(i,j) * t^j
            std::chrono::steady_clock::time_point time_start;   // Start time point when a spline is created
            float time_final;                                   // Final time in [s] for a spline. After this time, velocity, acceleration and jerk are zero, while position remains constant.
            float time_current;                                 // Elapsed time in [s] from a time instant when a spline is created. It is used to determine a current robot's position, velocity and acceleration. 
            float time_begin;                                   // Time instance in [s] when a spline begins in the current iteration
            float time_end;                                     // Time instance in [s] when a spline ends in the current iteration
            bool is_zero_final_vel;                             // Whether final velocity is zero
            bool is_zero_final_acc;                             // Whether final acceleration is zero
        };
        
    }
}

#endif //RPMPL_SPLINE_H