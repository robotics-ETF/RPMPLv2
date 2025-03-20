#include "DRGBTConfig.h"

size_t DRGBTConfig::MAX_NUM_ITER                                        = 1e9;
float DRGBTConfig::MAX_ITER_TIME                                        = 0.050;
float DRGBTConfig::MAX_PLANNING_TIME                                    = 60;
size_t DRGBTConfig::INIT_HORIZON_SIZE                                   = 10;
float DRGBTConfig::TRESHOLD_WEIGHT                                      = 0.5;
float DRGBTConfig::D_CRIT                                               = 0.05;
float DRGBTConfig::RESOLUTION_COLL_CHECK                                = 0.01;
size_t DRGBTConfig::MAX_NUM_MODIFY_ATTEMPTS                             = 10;
planning::PlannerType DRGBTConfig::STATIC_PLANNER_TYPE                  = planning::PlannerType::RGBMTStar;
planning::RealTimeScheduling DRGBTConfig::REAL_TIME_SCHEDULING          = planning::RealTimeScheduling::FPS;
float DRGBTConfig::MAX_TIME_TASK1                                       = 0.020;
planning::TrajectoryInterpolation DRGBTConfig::TRAJECTORY_INTERPOLATION = planning::TrajectoryInterpolation::Spline;
bool DRGBTConfig::GUARANTEED_SAFE_MOTION                                = true;
