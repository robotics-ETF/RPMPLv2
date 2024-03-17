//
// Created by dinko on 17.02.22.
//

typedef unsigned long size_t;

class RealVectorSpaceConfig
{
public:
    static float EQUALITY_THRESHOLD;                    // Threshold to determine whether two states are equal
    static size_t NUM_INTERPOLATION_VALIDITY_CHECKS;    // Number of discrete collision checks of the edge with the length of RRTConnectConfig::EPS_STEP
};