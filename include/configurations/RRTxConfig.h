//
// Created by dinko on 18.03.25.
//

#ifndef RPMPL_RRTXCONFIG_H
#define RPMPL_RRTXCONFIG_H

#include "PlanningTypes.h"

typedef unsigned long size_t;

class RRTxConfig {
  public:
    static constexpr size_t MAX_NUM_ITER = 10000;                 // Maximal number of iterations
    static constexpr float MAX_ITER_TIME = 0.050;                 // Maximal runtime of a single iteration
    static constexpr float MAX_PLANNING_TIME = 10.0f;             // Maximal planning time in seconds
    static constexpr size_t MAX_NUM_STATES = 10000;               // Maximal number of states
    static constexpr double EPS_STEP = 0.1;                       // Step size for extending
    static constexpr double R_REWIRE = 1.0;                       // Radius for rewiring
    static constexpr double R_COLLISION = 0.2;                    // Radius to check for collisions
    static constexpr double R_NEAREST = 0.5;                      // Radius for nearest neighbors
    static constexpr size_t MAX_NEIGHBORS = 30;                   // Maximal number of neighbors to consider
    static constexpr size_t REPLANNING_THROTTLE = 1;              // Process obstacles every N iterations
    static constexpr double REWIRE_FACTOR = 1.1;                  // Factor for rewire radius
    static constexpr double START_BIAS = 0.05;                    // Probability of sampling start directly
    static constexpr float RESOLUTION_COLL_CHECK = 0.01;          // Perform collision check when obstacle moves at most 'RESOLUTION_COLL_CHECK' in [m]
    static constexpr planning::TrajectoryInterpolation TRAJECTORY_INTERPOLATION = 
        planning::TrajectoryInterpolation::None;                // Method for interpolation of trajectory: "None" or "Spline"
    
};

#endif // RPMPL_RRTXCONFIG_H