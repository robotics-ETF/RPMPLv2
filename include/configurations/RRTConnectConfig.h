//
// Created by dinko on 17.02.22.
//

#ifndef RPMPL_RRTCONNECTCONFIG_H
#define RPMPL_RRTCONNECTCONFIG_H

typedef unsigned long size_t;

class RRTConnectConfig
{
public:
    static size_t MAX_NUM_ITER;                 // Maximal number of algorithm iterations
    static size_t MAX_NUM_STATES;               // Maximal number of considered states
    static float MAX_PLANNING_TIME;             // Maximal algorithm runtime in [s]
    static size_t MAX_EXTENSION_STEPS;          // Maximal number of extensions in connect procedure
    static float EPS_STEP;                      // Advancing step in C-space in [rad] used by RRT-based algorithms
};

#endif //RPMPL_RRTCONNECTCONFIG_H