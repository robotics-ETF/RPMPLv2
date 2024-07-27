//
// Created by nermin on 28.02.22.
//

#ifndef RPMPL_RBTCONNECTCONFIG_H
#define RPMPL_RBTCONNECTCONFIG_H

typedef unsigned long size_t;

class RBTConnectConfig
{
public:
    static size_t MAX_NUM_ITER;                 // Maximal number of algorithm iterations
    static size_t MAX_NUM_STATES;               // Maximal number of considered states
    static float MAX_PLANNING_TIME;             // Maximal algorithm runtime in [s]
    static float D_CRIT;                        // Critical distance in W-space in [m] when RBT uses RRT-mode
    static float DELTA;    	                    // Radius of hypersphere in [rad] from q to q_e
    static size_t NUM_SPINES;                   // Number of bur spines
    static size_t NUM_ITER_SPINE;               // Number of iterations when computing a single spine (1 iteration is minimum, when accordingly cummulative workspace distance function 'phi' is not computed)
    static bool USE_EXPANDED_BUBBLE;            // Whether to use expanded bubble when generating a spine. If yes, distance profile function for each robot's link is used
};

#endif //RPMPL_RBTCONNECTCONFIG_H