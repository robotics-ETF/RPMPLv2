//
// Created by nermin on 28.02.22.
//

#ifndef RPMPL_RGBTCONNECTCONFIG_H
#define RPMPL_RGBTCONNECTCONFIG_H

typedef unsigned long size_t;

class RGBTConnectConfig
{
public:
    static size_t MAX_NUM_ITER;                 // Maximal number of algorithm iterations
    static size_t MAX_NUM_STATES;               // Maximal number of considered states
    static float MAX_PLANNING_TIME;             // Maximal algorithm runtime in [s]
    static size_t NUM_LAYERS;                   // Number of layers (extensions) for generating a generalized bur
};

#endif //RPMPL_RGBTCONNECTCONFIG_H