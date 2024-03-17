//
// Created by nermin on 14.04.22.
//

#include <string>

typedef unsigned long size_t;

class DRGBTConfig
{
public:
    static size_t MAX_NUM_ITER;                     // Maximal number of algorithm iterations
    static float MAX_ITER_TIME;                     // Maximal runtime of a single iteration
    static float MAX_PLANNING_TIME;                 // Maximal algorithm runtime
    static size_t INIT_HORIZON_SIZE;                // Initial horizon size
    static float TRESHOLD_WEIGHT;                   // Treshold for the replanning assessment. Range: between 0 and 1
    static float D_CRIT;                            // Critical distance in W-space to compute critical nodes
    static size_t MAX_NUM_VALIDITY_CHECKS;          // Maximal number of validity checks when robot moves from previous to current configuration, while the obstacles are moving simultaneously
    static size_t MAX_NUM_MODIFY_ATTEMPTS;          // Maximal number of attempts when modifying bad or critical states
    static std::string STATIC_PLANNER_NAME;         // Name of a static planner (for obtaining the predefined path). Default: "RGBMT*" or "RGBTConnect" 
    static std::string REAL_TIME_SCHEDULING;        // "FPS" - Fixed Priority Scheduling; "none" - Without real-time scheduling    
    static float MAX_TIME_TASK1;                    // Maximal time which Task 1 can take from the processor
    static float MAX_TIME_UPDATE_CURRENT_STATE;     // Maximal time for the routine 'updateCurrentState'
    static std::string TRAJECTORY_INTERPOLATION;    // Method for interpolation of trajectory: 'none' or 'spline'
};