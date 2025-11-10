//
// Created by nermin on 17.10.25.
//

#ifndef RPMPL_ABSTRACTTRAJECTORY_H
#define RPMPL_ABSTRACTTRAJECTORY_H

#include "StateSpace.h"
#include "TrajectoryConfig.h"
#include "RealVectorSpaceConfig.h"

namespace planning::trajectory
{
    class State
    {
    public:
        State() {}
        State(size_t num_DOFs);
        State(const Eigen::VectorXf &pos_);
        State(const Eigen::VectorXf &pos_, const Eigen::VectorXf &vel_, const Eigen::VectorXf &acc_);

        Eigen::VectorXf pos;
        Eigen::VectorXf vel;
        Eigen::VectorXf acc;
    };

    class AbstractTrajectory
    {
    public:
        AbstractTrajectory(const std::shared_ptr<base::StateSpace> &ss_);
        AbstractTrajectory(const std::shared_ptr<base::StateSpace> &ss_, float max_iter_time_);
        virtual ~AbstractTrajectory() = 0;
        
        virtual bool computeRegular(planning::trajectory::State current, planning::trajectory::State target, 
                                    float t_iter_remain, float t_max, bool non_zero_final_vel) = 0;
        virtual bool computeSafe(planning::trajectory::State current, planning::trajectory::State target, 
                                 float t_iter_remain, float t_max, const std::shared_ptr<base::State> q_current) = 0;
        virtual bool convertPathToTraj(const std::vector<std::shared_ptr<base::State>> &path) = 0;

        virtual Eigen::VectorXf getPosition(float t) = 0;
        virtual Eigen::VectorXf getVelocity(float t) = 0;
        virtual Eigen::VectorXf getAcceleration(float t) = 0;

        inline float getTimeBegin() const { return time_begin; }
        inline float getTimeEnd() const { return time_end; }
        inline float getTimeCurrent() const { return time_current; }
        inline float getTimeFinal() const { return time_final; }
        inline bool getIsZeroFinalVel() const { return is_zero_final_vel; }
        inline const std::vector<Eigen::VectorXf> &getTrajPointCurrentIter() const { return traj_points_current_iter; }

        inline void setTimeBegin(float time_begin_) { time_begin = time_begin_; }
        inline void setTimeEnd(float time_end_) { time_end = time_end_; }
        inline void setTimeCurrent(float time_current_) { time_current = time_current_; }
        inline void setMaxRemainingIterTime(float max_remaining_iter_time_) { max_remaining_iter_time = max_remaining_iter_time_; }
        
        bool isFinalConf(const Eigen::VectorXf &pos);
        void addTrajPointCurrentIter(const Eigen::VectorXf &pos);
        void clearTrajPointCurrentIter();
        bool isSafe(const std::vector<Eigen::VectorXf> &pos_points, const std::shared_ptr<base::State> q_current, float t_iter, 
                    float time_step = TrajectoryConfig::TIME_STEP);
        float computeDistanceUnderestimation(const std::shared_ptr<base::State> q, 
                                             const std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points, float delta_t);
        void recordTrajectory(bool traj_computed, float t_offset);
        
    protected:
        void setParams();
        
        std::shared_ptr<base::StateSpace> ss;                       // State space of the robot
        float max_iter_time;                                        // Maximal iteration time
        float max_remaining_iter_time;                              // Maximal remaining iteration time in [s] till the end of the current iteration
        float time_current;                                         // Elapsed time in [s] from a time instant when the trajectory is created. It is used to determine a current robot's position, velocity and acceleration. 
        float time_final;                                           // Final time for the trajectory
        float time_begin;                                           // Time instance in [s] when the trajectory begins in the current iteration
        float time_end;                                             // Time instance in [s] when the trajectory ends in the current iteration
        bool is_zero_final_vel;                                     // Whether final velocity is zero. If not, robot will move at constant velocity after 'time_final'.
        bool all_robot_vel_same;                                    // Whether all joint velocities are the same
        float max_obs_vel;                                          // Maximal velocity of dynamic obstacles used to generate dynamic bubbles
        size_t max_num_iter_trajectory;                             // Maximal number of iterations when computing trajectory
        std::vector<Eigen::VectorXf> traj_points_current_iter;      // Trajectory points from the current iteration to be validated within 'MotionValidity'
        
    };
}

#endif //RPMPL_ABSTRACTTRAJECTORY_H