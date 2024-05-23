//
// Created by nermin on 23.05.24.
//
#ifndef RPMPL_SPLINE5CONFIG_H
#define RPMPL_SPLINE5CONFIG_H

class Spline5Config
{
public:
    static float MAX_TIME_COMPUTE;      // Maximal time in [s] for computing a spline
    static float MAX_TIME_PUBLISH;      // Maximal time in [s] for publishing a spline when using 'trajectory_msgs::msg::JointTrajectory' ROS2 topic 
    static float FINAL_JERK_STEP;       // Final jerk step when using bisection method for finding optimal value of coefficient 'c'
    static float FINAL_VELOCITY_STEP;   // Final velocity step when using bisection method for finding optimal value of final velocity
};

#endif //RPMPL_SPLINE5CONFIG_H