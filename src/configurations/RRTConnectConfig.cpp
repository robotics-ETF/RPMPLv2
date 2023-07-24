//
// Created by dinko on 17.02.22.
//

#include "RRTConnectConfig.h"

unsigned long RRTConnectConfig::MAX_NUM_ITER        = 1e9;
unsigned long RRTConnectConfig::MAX_NUM_STATES      = 10000;
float RRTConnectConfig::MAX_PLANNING_TIME           = 60e3;
int RRTConnectConfig::MAX_EXTENSION_STEPS           = 50;
float RRTConnectConfig::EPS_STEP                    = 0.1;