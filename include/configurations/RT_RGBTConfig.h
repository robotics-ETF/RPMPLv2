//
// Created by nermin on 25.09.25.
//

#ifndef RPMPL_RTRGBTCONFIG_H
#define RPMPL_RTRGBTCONFIG_H

#include "PlanningTypes.h"

typedef unsigned long size_t;

class RT_RGBTConfig
{
public:
    static size_t MAX_NUM_ITER;                                             // Maximal number of algorithm iterations
    static float MAX_ITER_TIME;                                             // Maximal runtime of a single iteration
    static float MAX_PLANNING_TIME;                                         // Maximal algorithm runtime
    static float RESOLUTION_COLL_CHECK;                                     // Perform collision check when obstacle moves at most 'RESOLUTION_COLL_CHECK' in [m]
    static float GOAL_PROBABILITY;                                          // Probability for choosing 'q_goal' as 'q_target'
    static planning::TrajectoryInterpolation TRAJECTORY_INTERPOLATION;      // Method for interpolation of trajectory: "None", "Spline" or "Ruckig"
};

#endif //RPMPL_RTRGBTCONFIG_H