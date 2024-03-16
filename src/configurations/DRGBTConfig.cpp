//
// Created by nermin on 14.04.22.
//

#include "DRGBTConfig.h"

unsigned long DRGBTConfig::MAX_NUM_ITER                 = 1e9;
float DRGBTConfig::MAX_ITER_TIME                        = 0.050;
float DRGBTConfig::MAX_PLANNING_TIME                    = 60;
int DRGBTConfig::INIT_HORIZON_SIZE                      = 10;
float DRGBTConfig::TRESHOLD_WEIGHT                      = 0.5;
float DRGBTConfig::D_CRIT                               = 0.05;
int DRGBTConfig::MAX_NUM_VALIDITY_CHECKS                = 10;
int DRGBTConfig::MAX_NUM_MODIFY_ATTEMPTS                = 10;
planning::PlannerType DRGBTConfig::STATIC_PLANNER_TYPE  = planning::PlannerType::RGBMTStar;
std::string DRGBTConfig::REAL_TIME_SCHEDULING           = "none";
float DRGBTConfig::MAX_TIME_TASK1                       = 0.020;
float DRGBTConfig::MAX_TIME_UPDATE_CURRENT_STATE        = 0.002;
std::string DRGBTConfig::TRAJECTORY_INTERPOLATION       = "spline";
