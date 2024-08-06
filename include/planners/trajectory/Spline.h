//
// Created by nermin on 13.02.24.
//

#ifndef RPMPL_SPLINE_H
#define RPMPL_SPLINE_H

#include <chrono>

#include "AbstractRobot.h"
#include "RealVectorSpaceConfig.h"
#include "SplinesConfig.h"

namespace planning::trajectory
{
    class Spline
    {
    public:
        Spline() {}
        Spline(size_t order_, const std::shared_ptr<robots::AbstractRobot> robot_, const Eigen::VectorXf &q_current);
        virtual ~Spline() = 0;

        virtual bool compute() = 0;
        virtual bool compute(const Eigen::VectorXf &q_final) = 0;
        virtual bool compute(const Eigen::VectorXf &q_final, const Eigen::VectorXf &q_final_dot) = 0;
        virtual bool compute(const Eigen::VectorXf &q_final, const Eigen::VectorXf &q_final_dot, const Eigen::VectorXf &q_final_ddot) = 0;
        virtual bool checkConstraints(size_t idx, float t_f) = 0;
        bool isFinalConf(const Eigen::VectorXf &q);
        
        virtual std::vector<float> getPositionExtremumTimes(size_t idx) = 0;
        virtual std::vector<float> getVelocityExtremumTimes(size_t idx) = 0;
        virtual std::vector<float> getAccelerationExtremumTimes(size_t idx) = 0;
        virtual std::vector<float> getJerkExtremumTimes(size_t idx) = 0;
        virtual int checkPositionMonotonicity(size_t idx);
        int checkPositionMonotonicity();

        virtual Eigen::VectorXf getPosition(float t);
        float getPosition(float t, size_t idx);

        virtual Eigen::VectorXf getVelocity(float t);
        float getVelocity(float t, size_t idx);

        virtual Eigen::VectorXf getAcceleration(float t);
        float getAcceleration(float t, size_t idx);

        virtual Eigen::VectorXf getJerk(float t);
        float getJerk(float t, size_t idx);

        float getCoeff(size_t i, size_t j) const { return coeff(i, j); }
        float getTimeFinal() const { return time_final; }
        float getTimeFinal(size_t idx) const { return times_final[idx]; }
        float getTimeCurrent(bool measure_time = false);
        float getTimeBegin() const { return time_begin; }
        float getTimeEnd() const { return time_end; }
        bool getIsZeroFinalVel() const { return is_zero_final_vel; }
        bool getIsZeroFinalAcc() const { return is_zero_final_acc; }

        void setTimeStart(float time_start_offset_);
        void setTimeFinal(float time_final_) { time_final = time_final_; }
        void setTimeCurrent(float time_current_) { time_current = time_current_; }
        void setTimeBegin(float time_begin_) { time_begin = time_begin_; }
        void setTimeEnd(float time_end_) { time_end = time_end_; }

        friend std::ostream &operator<<(std::ostream &os, const std::shared_ptr<planning::trajectory::Spline> spline);

    protected:            
        size_t order;
        std::shared_ptr<robots::AbstractRobot> robot;
        Eigen::MatrixXf coeff;                              // Num. of rows = 'num_DOFs', and num. of columns = 'order+1'. Form: sum{j=0, num_DOFs-1} coeff(i,j) * t^j
        std::chrono::steady_clock::time_point time_start;   // Start time point when a spline is created
        float time_start_offset;                            // Time offset in [s] which determines how much earlier 'time_start' is created
        std::vector<float> times_final;                     // Final time in [s] for each spline. After this time, velocity, acceleration and jerk are zero (if 'is_zero_final_vel' is true and 'is_zero_final_acc' is true), while position remains constant.
        float time_final;                                   // Maximal time from 'times_final', or a time set by user when prunning spline
        float time_current;                                 // Elapsed time in [s] from a time instant when a spline is created. It is used to determine a current robot's position, velocity and acceleration. 
        float time_begin;                                   // Time instance in [s] when a spline begins in the current iteration
        float time_end;                                     // Time instance in [s] when a spline ends in the current iteration
        bool is_zero_final_vel;                             // Whether final velocity is zero. If not, robot will move at constant velocity (if 'is_zero_final_acc' is true) after 'times_final[idx]'.
        bool is_zero_final_acc;                             // Whether final acceleration is zero. If not, robot will move at constant acceleration after 'times_final[idx]'.
        std::vector<std::shared_ptr<planning::trajectory::Spline>> subsplines;     // Contains a sequence of splines. Relevant only to 'CompositeSpline'.
    };
}

#endif //RPMPL_SPLINE_H