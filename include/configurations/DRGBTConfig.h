//
// Created by nermin on 14.04.22.
//

#ifndef RPMPL_DRGBTCONFIG_H
#define RPMPL_DRGBTCONFIG_H

#include <string>
#include <PlanningTypes.h>

typedef unsigned long size_t;

class DRGBTConfig
{
public:
    static size_t MAX_NUM_ITER;                                             // Maximal number of algorithm iterations
    static float MAX_ITER_TIME;                                             // Maximal runtime of a single iteration
    static float MAX_PLANNING_TIME;                                         // Maximal algorithm runtime
    static size_t INIT_HORIZON_SIZE;                                        // Initial horizon size
    static float TRESHOLD_WEIGHT;                                           // Treshold for the replanning assessment. Range: between 0 and 1
    static float D_CRIT;                                                    // Critical distance in W-space to compute critical nodes
    static size_t MAX_NUM_VALIDITY_CHECKS;                                  // Maximal number of validity checks when robot moves from previous to current configuration, while the obstacles are moving simultaneously
    static size_t MAX_NUM_MODIFY_ATTEMPTS;                                  // Maximal number of attempts when modifying bad or critical states
    static planning::PlannerType STATIC_PLANNER_TYPE;                       // Name of a static planner (for obtaining the predefined path). Available planners: "RGBMT*", "RGBT-Connect", "RBT-Connect" and "RRT-Connect" 
    static planning::RealTimeScheduling REAL_TIME_SCHEDULING;               // "FPS" - Fixed Priority Scheduling; "None" - Without real-time scheduling    
    static float MAX_TIME_TASK1;                                            // Maximal time which Task 1 can take from the processor
    static planning::TrajectoryInterpolation TRAJECTORY_INTERPOLATION;      // Method for interpolation of trajectory: "None" or "Spline"
    static bool GUARANTEED_SAFE_MOTION;                                     // Whether robot motion is surely safe for environment
};

#endif //RPMPL_DRGBTCONFIG_H