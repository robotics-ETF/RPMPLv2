#ifndef RPMPL_SL_DRGBTCONFIG_H
#define RPMPL_SL_DRGBTCONFIG_H

#include <cstddef>
#include <string>

class SL_DRGBTConfig
{
public:
    static std::size_t NUM_LAYERS;              // Number of layers for generating the pattern tree
    static float PROBABILITY_THRESHOLD;         // Maximum probability gap for considering alternative first-layer nodes
    static std::string ONNX_MODEL_PATH;         // Path to the GNN model for the prediction of the next best node. The model should be in ONNX format.
};

#endif //RPMPL_SL_DRGBTCONFIG_H
