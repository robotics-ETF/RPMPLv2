//
// Created by nermin on 08.10.26.
//

#ifndef RPMPL_SL_DRGBT_H
#define RPMPL_SL_DRGBT_H

#include "DRGBT.h"
#include "PatternTree.h"
#include "ONNX/Predictor.h"
#include "SL_DRGBTConfig.h"

// #include <glog/log_severity.h>
// #include <glog/logging.h>
// WARNING: You need to be very careful with using LOG(INFO) for console output, due to a possible "stack smashing detected" error.
// If you get this error, just use std::cout for console output.

namespace planning::drbt
{
    class SL_DRGBT : public planning::drbt::DRGBT
    {
    public:
        SL_DRGBT(const std::shared_ptr<base::StateSpace> ss_);
        SL_DRGBT(const std::shared_ptr<base::StateSpace> ss_, 
                 const std::shared_ptr<base::State> q_start_, const std::shared_ptr<base::State> q_goal_);
        ~SL_DRGBT();                         
        
        bool solve() override;
        void generateInput(const std::shared_ptr<base::State> &q_root, std::vector<float> &input);
        std::shared_ptr<base::State> computeNextState(const Ort::ShapeInferContext::Floats &input, 
            const Predictor::Probabilities &output, const size_t use_layers = 2);

    protected:
	    std::shared_ptr<planning::rbt::PatternTree> pattern_tree;
        Predictor onnx_model;
    };
}

#endif //RPMPL_SL_DRGBT_H
