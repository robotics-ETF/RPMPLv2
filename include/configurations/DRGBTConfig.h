//
// Created by nermin on 14.04.22.
//

#include <string>

class DRGBTConfig
{
public:
    static unsigned long MAX_NUM_ITER;          // Maximal number of algorithm iterations
    static int MAX_ITER_TIME;                   // Maximal runtime of a single iteration in [ms]
    static int MAX_PLANNING_TIME;               // Maximal algorithm runtime in [ms]
    static int INIT_HORIZON_SIZE;               // Initial horizon size
    static float MAX_ANG_VEL;                   // Maximal angular velocity of each robot's joint in [rad/s], which determines an advancing step in C-space in [rad] when moving from current towards next state in a single iteration
    static float TRESHOLD_WEIGHT;               // Treshold for the replanning assessment. Range: between 0 and 1
    static float D_CRIT;                        // Critical distance in W-space to compute critical nodes
    static int MAX_NUM_VALIDITY_CHECKS;         // Maximal number of validity checks when robot moves from previous to current configuration, while the obstacles are moving simultaneously
    static int MAX_NUM_MODIFY_ATTEMPTS;         // Maximal number of attempts when modifying bad or critical states
    static std::string STATIC_PLANNER_NAME;     // Name of a static planner (for obtaining the predefined path). Default: "RGBTConnect" or "RGBMT*"
    static std::string REAL_TIME_SCHEDULING;    // "FPS" - Fixed Priority Scheduling; "DPS" - Dynamic Priority Scheduling; "" - Without real-time scheduling    
    static int MAX_TIME_TASK1;                  // Maximal time in [ms] which Task 1 can take from the processor
};