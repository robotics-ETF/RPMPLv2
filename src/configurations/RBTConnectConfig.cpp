//
// Created by nermin on 28.02.22.
//

#include "RBTConnectConfig.h"

unsigned long RBTConnectConfig::MAX_NUM_ITER    = 1e9;
unsigned long RBTConnectConfig::MAX_NUM_STATES  = 10000;
int RBTConnectConfig::MAX_PLANNING_TIME         = 60e3;
float RBTConnectConfig::D_CRIT                  = 0.03;
float RBTConnectConfig::DELTA                   = 3.14159;
int RBTConnectConfig::NUM_SPINES                = 7;
int RBTConnectConfig::NUM_ITER_SPINE            = 5;
bool RBTConnectConfig::USE_EXPANDED_BUBBLE      = true;