//
// Created by dinko on 18.03.25.
//

#ifndef RPMPL_RRTXCONFIG_H
#define RPMPL_RRTXCONFIG_H

#include <string>
#include "PlanningTypes.h"

typedef unsigned long size_t;

class RRTxConfig 
{
public:
    static size_t MAX_NUM_ITER;                                         // Maximal number of iterations
    static float MAX_ITER_TIME;                                         // Maximal runtime of a single iteration
    static float MAX_PLANNING_TIME;                                     // Maximal planning time in seconds
    static float EPS_STEP;                                              // Step size for extending
    static float R_REWIRE;                                              // Radius for rewiring
    static float R_COLLISION;                                           // Radius to check for collisions
    static float R_NEAREST;                                             // Initial radius for nearest neighbors
    static size_t MAX_NEIGHBORS;                                        // Maximal number of neighbors to consider
    static size_t REPLANNING_THROTTLE;                                  // Process obstacles every N iterations
    static float REWIRE_FACTOR;                                         // Factor for rewire radius
    static float START_BIAS;                                            // Probability of sampling start directly
    static float RESOLUTION_COLL_CHECK;                                 // Perform collision check when obstacle moves at most 'RESOLUTION_COLL_CHECK' in [m]
    static planning::TrajectoryInterpolation TRAJECTORY_INTERPOLATION;  // Method for interpolation of trajectory: "None" or "Spline"
};

#endif // RPMPL_RRTXCONFIG_H