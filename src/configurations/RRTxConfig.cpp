#include "RRTxConfig.h"

size_t MAX_NUM_ITER                                         = 1e9;
float MAX_ITER_TIME                                         = 0.050;
float MAX_PLANNING_TIME                                     = 10.0f;
float EPS_STEP                                              = 0.1;
float R_REWIRE                                              = 1.0;
float R_COLLISION                                           = 0.2;
float R_NEAREST                                             = 0.5;
size_t MAX_NEIGHBORS                                        = 30;
size_t REPLANNING_THROTTLE                                  = 1;
float REWIRE_FACTOR                                         = 1.1;
float START_BIAS                                            = 0.05;
float RESOLUTION_COLL_CHECK                                 = 0.01;
planning::TrajectoryInterpolation TRAJECTORY_INTERPOLATION  = planning::TrajectoryInterpolation::None;
    