//
// Created by nermin on 23.05.24.
//

#ifndef RPMPL_SPLINESCONFIG_H
#define RPMPL_SPLINESCONFIG_H

class SplinesConfig
{
public:
    static float MAX_TIME_COMPUTE_REGULAR;      // Maximal time in [s] for computing a regular spline
    static float MAX_TIME_COMPUTE_SAFE;         // Maximal time in [s] for computing a safe spline
    static float MAX_TIME_PUBLISH;              // Maximal time in [s] for publishing a spline when using 'trajectory_msgs::msg::JointTrajectory' ROS2 topic. Determined by time measurements.
    static float MAX_TIME_FINAL;                // Maximal final time in [s] for a spline to be considered as valid 
    static float TIME_STEP;                     // Time step used for e.g. checking whether a spline is collision-free
    static float FINAL_JERK_STEP;               // Final jerk step when using bisection method for finding optimal value of coefficient 'c'
    static float FINAL_VELOCITY_STEP;           // Final velocity step when using bisection method for finding optimal value of final velocity
};

#endif //RPMPL_SPLINESCONFIG_H