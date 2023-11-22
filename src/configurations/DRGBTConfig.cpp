//
// Created by nermin on 14.04.22.
//

#include "DRGBTConfig.h"

unsigned long DRGBTConfig::MAX_NUM_ITER         = 10e6;
float DRGBTConfig::MAX_ITER_TIME                = 1000;
float DRGBTConfig::MAX_PLANNING_TIME            = 60e3;
int DRGBTConfig::INIT_HORIZON_SIZE              = 10;
float DRGBTConfig::STEP                         = 0.1;
float DRGBTConfig::WEIGHT_MIN                   = 0.5;
float DRGBTConfig::WEIGHT_MEAN_MIN              = 0.5;
float DRGBTConfig::D_CRIT                       = 0.05;
float DRGBTConfig::TASK1_UTILITY                = 0.3;
int DRGBTConfig::MAX_NUM_VALIDITY_CHECKS        = 10;
std::string DRGBTConfig::REAL_TIME_SCHEDULING   = "";