//
// Created by nermin on 28.02.22.
//

class RBTConnectConfig
{
public:
    static unsigned long MAX_NUM_ITER;          // Maximal number of algorithm iterations
    static unsigned long MAX_NUM_STATES;        // Maximal number of considered states
    static int MAX_PLANNING_TIME;               // Maximal algorithm runtime in [ms]
    static float D_CRIT;                        // Critical distance in W-space in [m] when RBT uses RRT-mode
    static float DELTA;    	                    // Radius of hypersphere in [rad] from q to q_e
    static int NUM_SPINES;                      // Number of bur spines
    static int NUM_ITER_SPINE;                  // Number of iterations when computing a single spine (1 iteration is minimum, when accordingly function 'fi' is not computed)
    static bool USE_EXPANDED_BUBBLE;            // Whether to use expanded bubble when generating a spine. If yes, distance profile function for each robot's link is used
};