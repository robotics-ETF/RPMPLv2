#include "TrajectoryConfig.h"

float TrajectoryConfig::MAX_TIME_COMPUTE_REGULAR   = 0.001;
float TrajectoryConfig::MAX_TIME_COMPUTE_SAFE      = 0.003;
float TrajectoryConfig::MAX_TIME_PUBLISH           = 0.0005;
float TrajectoryConfig::MAX_TIME_FINAL             = 10.0;
float TrajectoryConfig::TIME_STEP                  = 0.01;
float TrajectoryConfig::FINAL_JERK_STEP            = 1.0;
float TrajectoryConfig::FINAL_VELOCITY_STEP        = 0.1;
float TrajectoryConfig::MAX_RADIUS                 = 5.0;
size_t TrajectoryConfig::NUM_VALIDITY_POINTS_CHECK = 10;