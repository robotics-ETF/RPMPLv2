#include "RRTxConfig.h"

size_t RRTxConfig::MAX_NUM_ITER                                         = 1e9;
float RRTxConfig::MAX_ITER_TIME                                         = 0.050;
float RRTxConfig::MAX_PLANNING_TIME                                     = 10.0f;
float RRTxConfig::EPS_STEP                                              = 0.1;
float RRTxConfig::R_REWIRE                                              = 3.14159;
size_t RRTxConfig::MAX_NEIGHBORS                                        = 1000;
size_t RRTxConfig::REPLANNING_THROTTLE                                  = 1;
float RRTxConfig::START_BIAS                                            = 0.1;
float RRTxConfig::RESOLUTION_COLL_CHECK                                 = 0.01;
planning::TrajectoryInterpolation RRTxConfig::TRAJECTORY_INTERPOLATION  = planning::TrajectoryInterpolation::Spline;