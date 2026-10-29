#ifndef RPMPL_SL_DRGBTCONFIG_H
#define RPMPL_SL_DRGBTCONFIG_H

#include <cstddef>
#include <string>

class SL_DRGBTConfig
{
public:
    static std::size_t NUM_LAYERS;              // Number of layers for generating the pattern tree
    static std::size_t NUM_NEW_STATES;          // Number of new states predicted by NN that are added to the horizon
    static std::size_t STATE_MAX_HISTORY;       // Maximum history a state may remain in the horizon, measured by state index difference
    static std::string ONNX_MODEL_PATH;         // Path to the GNN model for the prediction of the next best node. The model should be in ONNX format.
};

#endif //RPMPL_SL_DRGBTCONFIG_H
