#include "PatternTree.h"

planning::rbt::PatternTree::PatternTree(const std::shared_ptr<base::StateSpace> ss_, size_t num_layers_) : RGBTConnect(ss_) 
{
	num_layers = num_layers_;
}

size_t planning::rbt::PatternTree::getNumNodes(int num_layers_)
{
	if (num_layers_ == -1)
		num_layers_ = num_layers;
	
	size_t num_nodes { 1 + 2*ss->num_dimensions };
	for (size_t i = 1; i < size_t(num_layers_); i++)
		num_nodes += 2*ss->num_dimensions * std::pow(2*ss->num_dimensions - 1, i);
	
	return num_nodes;
}

const std::vector<std::shared_ptr<base::State>> planning::rbt::PatternTree::generateGBur(const std::shared_ptr<base::State> q_root, float delta)
{
	std::shared_ptr<base::State> q_parent { q_root->getParent() };
	std::shared_ptr<base::State> q_e { nullptr };
	std::shared_ptr<base::State> q_new { q_root };
	base::State::Status status { base::State::Status::None };
	std::vector<std::shared_ptr<base::State>> gbur {};
	Eigen::VectorXf q_root_e {};
	Eigen::VectorXf q_root_parent {};
	float cos_phi {};

	// LOG(INFO) << "Generating gbur...";
	// LOG(INFO) << "q_root: " << q_root->getCoord().transpose();

	for (size_t i = 0; i < ss->num_dimensions; i++)
	{
		for (int sign : {-1, 1})
		{			
			q_e = ss->getNewState(q_root->getCoord());
			q_e->setCoord(q_root->getCoord(i) + sign * delta, i);
			// LOG(INFO) << "q_e: " << q_e->getCoord().transpose();

			if (q_parent != nullptr)
			{
				q_root_e = q_e->getCoord() - q_root->getCoord();
				q_root_parent = q_parent->getCoord() - q_root->getCoord();
				cos_phi = q_root_e.dot(q_root_parent) / (q_root_e.norm() * q_root_parent.norm());
				if (std::abs(cos_phi - 1) < RealVectorSpaceConfig::EQUALITY_THRESHOLD)	// Two vectors are paralell and same oriented.
					continue;
			}

			q_e = ss->pruneEdge(q_root, q_e);
			tie(status, q_new) = extendGenSpine(q_root, q_e);
			// tie(status, q_new) = extendSpine(q_root, q_e);
			gbur.emplace_back(q_new);
		}
	}

	// LOG(INFO) << "Generated gbur...";
	return gbur;
}

const std::shared_ptr<base::Tree> planning::rbt::PatternTree::generateLocalTree(const std::shared_ptr<base::State> q_root)
{
	// LOG(INFO) << "Generating local tree...";
	tree = std::make_shared<base::Tree>();
	tree->upgradeTree(q_root, nullptr);
	std::vector<std::shared_ptr<base::State>> Q_prev { generateGBur(q_root) };
	
	// LOG(INFO) << "Layer 0";
	for (std::shared_ptr<base::State> q : Q_prev)
		tree->upgradeTree(q, q_root);

	for (size_t num = 1; num < num_layers; num++)
	{
		// LOG(INFO) << "Layer " << num;
		std::vector<std::shared_ptr<base::State>> Q_temp {};
		std::vector<std::shared_ptr<base::State>> Q {};
		for (std::shared_ptr<base::State> q : Q_prev)
		{
			Q_temp = generateGBur(q);
			for (std::shared_ptr<base::State> q_new : Q_temp)
			{
				Q.emplace_back(q_new);
				tree->upgradeTree(q_new, q);
			}
		}
		Q_prev = Q;
	}

	return tree;
}
