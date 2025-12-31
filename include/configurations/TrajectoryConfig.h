//
// Created by nermin on 23.05.24.
//

#ifndef RPMPL_TRAJECTORYCONFIG_H
#define RPMPL_TRAJECTORYCONFIG_H

typedef unsigned long size_t;

class TrajectoryConfig
{
public:
    static float MAX_TIME_COMPUTE_REGULAR;      // Maximal time in [s] for computing a regular trajectory
    static float MAX_TIME_COMPUTE_SAFE;         // Maximal time in [s] for computing a safe trajectory
    static float MAX_TIME_FINAL;                // Maximal final time in [s] for a trajectory to be considered as valid 
    static float TIME_STEP;                     // Time step used for e.g. checking whether a trajectory is collision-free
    static float FINAL_JERK_STEP;               // Final jerk step when using bisection method for finding optimal value of coefficient 'c'
    static float FINAL_VELOCITY_STEP;           // Final velocity step when using bisection method for finding optimal value of final velocity
    static float MAX_RADIUS;                    // Maximal radius in [rad] used when updating state (it should be experimentally evaluated)
    static size_t NUM_VALIDITY_POINTS_CHECK;    // Number of validity points from trajectory to be checked within 'MotionValidity' class
    static bool SCALE_TARGET;                   // Whether target state is scaled depending on the robot's current velocity. Recommended true in simulation, but false on a real robot to reduce jerk changes (i.e., to increase the lifespan of actuators). 
};

#endif //RPMPL_TRAJECTORYCONFIG_H