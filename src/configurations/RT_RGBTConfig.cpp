#include "RT_RGBTConfig.h"

size_t RT_RGBTConfig::MAX_NUM_ITER                                        = 1e9;
float RT_RGBTConfig::MAX_ITER_TIME                                        = 0.050;
float RT_RGBTConfig::MAX_PLANNING_TIME                                    = 10;
float RT_RGBTConfig::RESOLUTION_COLL_CHECK                                = 0.01;
float RT_RGBTConfig::GOAL_PROBABILITY                                     = 0.5;
planning::TrajectoryInterpolation RT_RGBTConfig::TRAJECTORY_INTERPOLATION = planning::TrajectoryInterpolation::Spline;