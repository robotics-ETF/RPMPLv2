//
// Created by nermin on 14.04.22.
//

#include <string>

class DRGBTConfig
{
public:
    static unsigned long MAX_NUM_ITER;          // Maximal number of algorithm iterations
    static float MAX_ITER_TIME;                 // Maximal runtime of a single iteration in [ms] (also 1/f_s, where f_s is the number of frames per second)
    static float MAX_PLANNING_TIME;             // Maximal algorithm runtime in [ms]
    static int INIT_HORIZON_SIZE;               // Initial horizon size
    static float STEP;                          // Advancing step in C-space in [rad] when moving from current towards next state in a single iteration
    static float WEIGHT_MIN;                    // Treshold 1 for the replanning assessment. Range: between 0 and 1
    static float WEIGHT_MEAN_MIN;               // Treshold 2 for the replanning assessment. Range: between 0 and 1
    static float D_CRIT;                        // Critical distance in W-space in [m] when RBT uses RRT-mode
    static float TASK1_UTILITY;                 // Maximal utility which Task 1 can take from the processor
    static int MAX_NUM_VALIDITY_CHECKS;         // Maximal number of validity checks when robot moves from previous to current configuration, while the obstacles are moving simultaneously
    static std::string REAL_TIME_SCHEDULING;    // "DPS" - Dynamic Priority Scheduling; "FPS" - Fixed Priority Scheduling; "" - Without real-time scheduling
};