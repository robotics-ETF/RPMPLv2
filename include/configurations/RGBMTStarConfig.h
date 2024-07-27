//
// Created by nermin on 28.02.22.
//

#ifndef RPMPL_RGBMTSTARCONFIG_H
#define RPMPL_RGBMTSTARCONFIG_H

typedef unsigned long size_t;

class RGBMTStarConfig
{
public:
    static size_t MAX_NUM_ITER;                 // Maximal number of algorithm iterations
    static size_t MAX_NUM_STATES;               // Maximal number of considered states
    static float MAX_PLANNING_TIME;             // Maximal algorithm runtime in [s]
    static bool TERMINATE_WHEN_PATH_IS_FOUND;   // Whether to terminate when path is found (default: false)
};

#endif //RPMPL_RGBMTSTARCONFIG_H