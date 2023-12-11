//
// Created by nermin on 14.04.22.
//

#include "DRGBTConfig.h"

unsigned long DRGBTConfig::MAX_NUM_ITER         = 10e6;
float DRGBTConfig::MAX_ITER_TIME                = 100;
float DRGBTConfig::MAX_PLANNING_TIME            = 60e3;
int DRGBTConfig::INIT_HORIZON_SIZE              = 10;
float DRGBTConfig::STEP                         = 1.0;
float DRGBTConfig::TRESHOLD_WEIGHT              = 0.5;
float DRGBTConfig::D_CRIT                       = 0.05;
int DRGBTConfig::MAX_NUM_VALIDITY_CHECKS        = 10;
std::string DRGBTConfig::STATIC_PLANNER_NAME    = "RGBTConnect";
std::string DRGBTConfig::REAL_TIME_SCHEDULING   = "";
float DRGBTConfig::TASK1_UTILITY                = 0.2;