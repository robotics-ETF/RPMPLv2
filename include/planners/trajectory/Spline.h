//
// Created by nermin on 13.02.24.
//
#ifndef RPMPL_SPLINE_H
#define RPMPL_SPLINE_H

#include <chrono>

#include "StateSpace.h"

namespace planning
{
    namespace trajectory
    {
        class Spline
        {
        public:
            Spline(int order_, const std::shared_ptr<base::StateSpace> ss_, const std::shared_ptr<base::State> q_current);
		    virtual ~Spline() = 0;

            virtual bool compute(const std::shared_ptr<base::State> q_final) = 0;
            virtual bool checkConstraints(int idx, float t_f) = 0;
            
            std::shared_ptr<base::State> getPosition(float t);
            float getPosition(float t, int idx);
            virtual float getPosition(float t, int idx, float t_f) = 0;

            std::shared_ptr<base::State> getVelocity(float t);
            float getVelocity(float t, int idx);
            virtual float getVelocity(float t, int idx, float t_f) = 0;

            std::shared_ptr<base::State> getAcceleration(float t);
            float getAcceleration(float t, int idx);
            virtual float getAcceleration(float t, int idx, float t_f) = 0;

            std::shared_ptr<base::State> getJerk(float t);
            float getJerk(float t, int idx);
            virtual float getJerk(float t, int idx, float t_f) = 0;

            float getCoeff(int i, int j) const { return coeff(i, j); }
            float getTimeCurrent() const { return time_current; }
            float getTimeBegin() const { return time_begin; }
            float getTimeEnd() const { return time_end; }

            void setTimeStart();
            void setTimeCurrent(float time_current_ = -1);
            void setTimeBegin(float time_begin_) { time_begin = time_begin_; }
            void setTimeEnd(float time_end_) { time_end = time_end_; }

            friend std::ostream &operator<<(std::ostream &os, const std::shared_ptr<planning::trajectory::Spline> spline);

        protected:            
            int order;
            std::shared_ptr<base::StateSpace> ss;
            Eigen::MatrixXf coeff;                              // Num. of rows = 'num_DOFs', and num. of columns = 'order+1'. Form: sum{j=0, num_DOFs-1} coeff(i,j) * t^j
            std::chrono::steady_clock::time_point time_start;   // Start time point when a spline is created
            float time_final;                                   // Final time in [s] for a spline. After this time, velocity, acceleration and jerk are zero, while position remains constant.
            float time_current;                                 // Elapsed time in [s] from a time instant when a spline is created. It is used to determine a current robot's position, velocity and acceleration. 
            float time_begin;                                   // Time instance in [s] when a spline begins in the current iteration
            float time_end;                                     // Time instance in [s] when a spline ends in the current iteration
        };
        
    }
}

#endif //RPMPL_SPLINE_H