//
// Created by nermin on 18.02.22.
//

#ifndef RPMPL_PATTERNTREE_H
#define RPMPL_PATTERNTREE_H

#include "RGBTConnect.h"
#include "RealVectorSpaceConfig.h"

namespace planning::rbt
{
    class PatternTree : public planning::rbt::RGBTConnect
    {
    public:
        PatternTree(const std::shared_ptr<base::StateSpace> ss_, size_t num_layers_);
    
		const std::vector<std::shared_ptr<base::State>> generateGBur(const std::shared_ptr<base::State> q_root, float delta = RBTConnectConfig::DELTA);
		const std::shared_ptr<base::Tree> generateLocalTree(const std::shared_ptr<base::State> q_root);
		size_t getNumNodes(int num_layers_ = -1);
	
	private:
		size_t num_layers;
		std::shared_ptr<base::Tree> tree;
	};
}

#endif //RPMPL_PATTERNTREE_H