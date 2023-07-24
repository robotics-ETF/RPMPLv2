//
// Created by dinko on 17.02.22.
//

class RealVectorSpaceConfig
{
public:
    static float EQUALITY_THRESHOLD;                // Threshold to determine whether two states are equal
    static int NUM_INTERPOLATION_VALIDITY_CHECKS;   // Number of discrete collision checks of the edge with the length of RRTConnectConfig::EPS_STEP

};