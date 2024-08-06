#include "RBTConnectConfig.h"

size_t RBTConnectConfig::MAX_NUM_ITER       = 1e9;
size_t RBTConnectConfig::MAX_NUM_STATES     = 1e9;
float RBTConnectConfig::MAX_PLANNING_TIME   = 60;
float RBTConnectConfig::D_CRIT              = 0.03;
float RBTConnectConfig::DELTA               = 3.14159;
size_t RBTConnectConfig::NUM_SPINES         = 7;
size_t RBTConnectConfig::NUM_ITER_SPINE     = 5;
bool RBTConnectConfig::USE_EXPANDED_BUBBLE  = true;