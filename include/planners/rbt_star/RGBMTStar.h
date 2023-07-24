//
// Created by nermin on 20.02.22.
//
#ifndef RPMPL_RGBMTSTAR_H
#define RPMPL_RGBMTSTAR_H

#include "RGBTConnect.h"

namespace planning
{
	namespace rbt_star
	{
		class RGBMTStar : public planning::rbt::RGBTConnect
		{
		public:
			RGBMTStar(std::shared_ptr<base::StateSpace> ss_);
			RGBMTStar(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_, std::shared_ptr<base::State> goal_);
			~RGBMTStar();
			bool solve() override;
            bool checkTerminatingCondition(std::shared_ptr<base::State> q_con);
			void outputPlannerData(std::string filename, bool output_states_and_paths = true, bool append_output = false) const override;

		protected:
            std::vector<size_t> num_states;              // Total number of states for each tree
            float cost_opt;                              // Cost of the final path 
            
			std::tuple<base::State::Status, std::shared_ptr<base::State>> connectGenSpine
                (std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e);
            float computeCostToCome(std::shared_ptr<base::State> q1, std::shared_ptr<base::State> q2);
            std::shared_ptr<base::State> optimize(std::shared_ptr<base::State> q, std::shared_ptr<base::Tree> tree, 
                                                  std::shared_ptr<base::State> q_reached);
            void unifyTrees(std::shared_ptr<base::Tree> tree, std::shared_ptr<base::Tree> tree0,
                            std::shared_ptr<base::State> q_con, std::shared_ptr<base::State> q0_con);
            void deleteTrees(std::vector<int> &trees_connected);
            std::shared_ptr<base::State> getRandomState();
            void computePath(std::shared_ptr<base::State> q_con);
    
        private:
            void considerChildren(std::shared_ptr<base::State> q, std::shared_ptr<base::Tree> tree0,
                                  std::shared_ptr<base::State> q0_con, std::shared_ptr<base::State> q_considered);
        };
	}
}
#endif //RPMPL_RGBMTSTAR_H