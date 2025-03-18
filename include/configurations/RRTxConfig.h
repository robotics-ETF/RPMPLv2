//
// Created by dinko on 18.03.25.
//

#ifndef RPMPL_RRTXCONNECTCONFIG_H
#define RPMPL_RRTXCONNECTCONFIG_H

typedef unsigned long size_t;

class RRTxConfig {
  public:
    static constexpr float MAX_PLANNING_TIME = 60.0f; // Maximum planning time in seconds
    static constexpr size_t MAX_NUM_STATES = 10000;   // Maximum number of states
    static constexpr size_t MAX_NUM_ITER = 10000;     // Maximum number of iterations
    static constexpr double EPS_STEP = 0.1;           // Step size for extending
    static constexpr double R_REWIRE = 1.0;           // Radius for rewiring
    static constexpr double R_COLLISION = 0.2;        // Radius to check for collisions
    static constexpr double R_NEAREST = 0.5;          // Radius for nearest neighbors
    static constexpr size_t MAX_NEIGHBORS = 30;       // Maximum number of neighbors to consider
    static constexpr size_t REPLANNING_THROTTLE = 10; // Process obstacles every N iterations
    static constexpr double REWIRE_FACTOR = 1.1;      // Factor for rewire radius
    static constexpr double GOAL_BIAS = 0.05;         // Probability of sampling goal directly
};

#endif // RPMPL_RRTXCONNECTCONFIG_H