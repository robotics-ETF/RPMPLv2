//
// Created by nermin on 20.07.24.
//

#ifndef RPMPL_TRAJECTORY_H
#define RPMPL_TRAJECTORY_H

#include "AbstractTrajectory.h"
#include "Spline4.h"
#include "Spline5.h"
#include "CompositeSpline.h"
#include "RRTConnectConfig.h"

namespace planning::trajectory
{
    class Trajectory : public AbstractTrajectory
    {
    public:
        Trajectory(const std::shared_ptr<base::StateSpace> &ss_);
        Trajectory(const std::shared_ptr<base::StateSpace> &ss_, planning::trajectory::State current, float max_iter_time_);
        ~Trajectory();

        bool computeRegular(planning::trajectory::State current, planning::trajectory::State target, 
                            float t_iter_remain, float t_max, bool non_zero_final_vel) override;
        bool computeSafe(planning::trajectory::State current, planning::trajectory::State target, 
                         float t_iter_remain, float t_max, const std::shared_ptr<base::State> q_current) override;
        
        inline Eigen::VectorXf getPosition(float t) override { return spline->getPosition(t); }
        inline Eigen::VectorXf getVelocity(float t) override { return spline->getVelocity(t); }
        inline Eigen::VectorXf getAcceleration(float t) override { return spline->getAcceleration(t); }

        bool convertPathToTraj(const std::vector<std::shared_ptr<base::State>> &path) override;

    private:
        bool isSafeSpline(const std::shared_ptr<planning::trajectory::Spline> spline_safe, 
                          const std::shared_ptr<base::State> q_current, float t_iter);
        void setSpline(const std::shared_ptr<planning::trajectory::Spline> spline_);

        bool convertPathToTraj_v1(const std::vector<std::shared_ptr<base::State>> &path);
        bool convertPathToTraj_v2(const std::vector<std::shared_ptr<base::State>> &path);
        bool convertPathToTraj_v3(const std::vector<std::shared_ptr<base::State>> &path, bool must_visit = false);

        std::shared_ptr<planning::trajectory::Spline> spline;
    };
}

#endif //RPMPL_TRAJECTORY_H