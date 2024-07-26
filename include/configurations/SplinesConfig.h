//
// Created by nermin on 23.05.24.
//
#ifndef RPMPL_SPLINESCONFIG_H
#define RPMPL_SPLINESCONFIG_H

class SplinesConfig
{
public:
    static float MAX_TIME_COMPUTE;              // Maximal time in [s] for computing a spline
    static float MAX_TIME_PUBLISH;              // Maximal time in [s] for publishing a spline when using 'trajectory_msgs::msg::JointTrajectory' ROS2 topic. Determined by time measurements.
    static float MAX_TIME_FINAL;                // Maximal final time in [s] for a spline to be considered as valid 
    static float TIME_STEP;                     // Time step used for e.g. checking whether a spline is collision-free
    static float FINAL_JERK_STEP;               // Final jerk step when using bisection method for finding optimal value of coefficient 'c'
    static float FINAL_VELOCITY_STEP;           // Final velocity step when using bisection method for finding optimal value of final velocity
    static bool IS_FINAL_VELOCITY_ZERO;         // Whether a final velocity is zero when computing splines. If true, robot motion is more safe, yet a little bit slower. Note that when 'GUARANTEED_SAFE_MOTION' is true, this parameter is always true.
};

#endif //RPMPL_SPLINESCONFIG_H