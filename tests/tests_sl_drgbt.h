//
// Regression test: SL_DRGBT::generateInput must handle pattern-tree nodes in collision, which have no nearest points.
// They occur when a spine enters an obstacle that 'computeDistance' ignores beyond its 'min_dist_tol', e.g., the ground
// in xarm6/scenario1. As in apps/generate_dataset.cpp (training data), OR-vectors of such nodes are zero.
//

#include "SL_DRGBT.h"
#include "PatternTree.h"
#include "ConfigurationReader.h"
#include "Scenario.h"

#include <cmath>
#include <random>

TEST(SL_DRGBTTest, generateInputWithCollidingTreeNodes)
{
	ConfigurationReader::initConfiguration(RPMPL_PROJECT_PATH);
	scenario::Scenario scenario("/data/xarm6/scenario1/scenario1.yaml", RPMPL_PROJECT_PATH);
	std::shared_ptr<base::StateSpace> ss { scenario.getStateSpace() };
	ss->robot->setGripperLength(0);		// With the gripper, self-collision results are not reproducible (see TODO in xArm6.cpp)

	SL_DRGBTConfig::ONNX_MODEL_PATH = std::string(RPMPL_PROJECT_PATH) + "/data/xarm6/onnx_models/GATR.onnx";
	planning::drbt::SL_DRGBT planner(ss, scenario.getStart(), scenario.getGoal());
	planning::rbt::PatternTree pattern_tree(ss, SL_DRGBTConfig::NUM_LAYERS);
	const size_t num_nodes { pattern_tree.getNumNodes(SL_DRGBTConfig::NUM_LAYERS - 1) };	// Nodes with OR-vectors in the input

	// Find a valid root whose pattern tree contains a node without nearest points
	std::mt19937 generator(7);
	Eigen::VectorXf root_coord(ss->num_dimensions);
	size_t colliding_node { 0 };
	for (size_t attempt = 0; attempt < 200 && colliding_node == 0; attempt++)
	{
		for (size_t i = 0; i < ss->num_dimensions; i++)
			root_coord(i) = std::uniform_real_distribution<float>(ss->robot->getLimits()[i].first, ss->robot->getLimits()[i].second)(generator);
		if (!ss->isValid(ss->getNewState(root_coord)))
			continue;

		std::shared_ptr<base::Tree> tree { pattern_tree.generateLocalTree(ss->getNewState(root_coord)) };
		for (size_t i = 1; i < num_nodes && colliding_node == 0; i++)
			if (tree->getState(i)->getNearestPoints() == nullptr)
				colliding_node = i;
	}
	ASSERT_NE(colliding_node, 0) << "No root with a colliding pattern-tree node was found";

	std::vector<float> input {};
	planner.generateInput(ss->getNewState(root_coord), input);
	ASSERT_EQ(input.size(), Predictor(SL_DRGBTConfig::ONNX_MODEL_PATH).getInputCount());
	for (float value : input)
		ASSERT_TRUE(std::isfinite(value));

	const size_t or_size { 3 * ss->num_dimensions };	// OR-vectors of all links of one node
	const size_t or_first { input.size() - num_nodes * or_size + colliding_node * or_size };
	for (size_t k = 0; k < or_size; k++)
		EXPECT_EQ(input[or_first + k], 0) << "OR-vector component " << k << " of node " << colliding_node;
}
