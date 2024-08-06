//
// Created by nermin on 20.02.22.
//

#ifndef RPMPL_RGBMTSTAR_H
#define RPMPL_RGBMTSTAR_H

#include "RGBTConnect.h"
#include "RGBMTStarConfig.h"

// #include <glog/log_severity.h>
// #include <glog/logging.h>
// WARNING: You need to be very careful with using LOG(INFO) for console output, due to a possible "stack smashing detected" error.
// If you get this error, just use std::cout for console output.

namespace planning::rbt_star
{
    class RGBMTStar : public planning::rbt::RGBTConnect
    {
    public:
        RGBMTStar(const std::shared_ptr<base::StateSpace> ss_);
        RGBMTStar(const std::shared_ptr<base::StateSpace> ss_, 
                  const std::shared_ptr<base::State> q_start_, const std::shared_ptr<base::State> q_goal_);
        
        bool solve() override;
        
        bool checkTerminatingCondition(base::State::Status status) override;
        void outputPlannerData(const std::string &filename, bool output_states_and_paths = true, bool append_output = false) const override;

    protected:
        std::vector<size_t> num_states;             // Total number of states for each tree
        float cost_opt;                             // Cost of the final path 
        std::shared_ptr<base::State> q_con_opt;     // State (takes start or goal conf.) from which the optimal path is constructed

        std::tuple<base::State::Status, std::shared_ptr<base::State>> connectGenSpine
            (const std::shared_ptr<base::State> q, const std::shared_ptr<base::State> q_e);
        float computeCostToCome(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2);
        std::shared_ptr<base::State> optimize(const std::shared_ptr<base::State> q, const std::shared_ptr<base::Tree> tree, 
                                              std::shared_ptr<base::State> q_reached);
        void unifyTrees(const std::shared_ptr<base::Tree> tree0, const std::shared_ptr<base::State> q_con, 
                        const std::shared_ptr<base::State> q0_con);
        void deleteTrees(const std::vector<size_t> &trees_connected);
        std::shared_ptr<base::State> getRandomState();
        void computePath(std::shared_ptr<base::State> q_con);

    private:
        void considerChildren(const std::shared_ptr<base::State> q, const std::shared_ptr<base::Tree> tree0,
                              const std::shared_ptr<base::State> q0_con, const std::shared_ptr<base::State> q_considered);
    };
}

#endif //RPMPL_RGBMTSTAR_H