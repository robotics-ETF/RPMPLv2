#include "RGBMTStar.h"
#include "ConfigurationReader.h"
#include "CommonFunctions.h"

namespace planning::rbt
{
    class PatternTree : public planning::rbt::RGBTConnect
    {
    public:
        PatternTree(const std::shared_ptr<base::StateSpace> ss_, size_t num_layers_);
    
		const std::vector<std::shared_ptr<base::State>> generateGBur(const std::shared_ptr<base::State> q_root, float delta = RBTConnectConfig::DELTA);
		const std::shared_ptr<base::Tree> generateLocalTree(const std::shared_ptr<base::State> q_root);
		size_t getNumNodes(size_t num_layers_ = 0);
	
	private:
		size_t num_layers;
		std::shared_ptr<base::Tree> tree;
	};
}

planning::rbt::PatternTree::PatternTree(const std::shared_ptr<base::StateSpace> ss_, size_t num_layers_) : RGBTConnect(ss_) 
{
	num_layers = num_layers_;
}

size_t planning::rbt::PatternTree::getNumNodes(size_t num_layers_)
{
	if (num_layers_ == 0)
		num_layers_ = num_layers;
	
	size_t num_nodes { 1 + 2*ss->num_dimensions };
	for (size_t i = 1; i < num_layers_; i++)
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


int main(int argc, char **argv)
{
	std::string scenario_file_path
	{
		"/data/planar_2dof/scenario_test/scenario_test.yaml"
		// "/data/planar_2dof/scenario1/scenario1.yaml"
		// "/data/planar_2dof/scenario2/scenario2.yaml"
		// "/data/planar_2dof/scenario3/scenario3.yaml"

		// "/data/planar_10dof/scenario_test/scenario_test.yaml"
		// "/data/planar_10dof/scenario1/scenario1.yaml"
		// "/data/planar_10dof/scenario2/scenario2.yaml"

		// "/data/xarm6/scenario_test/scenario_test.yaml"
		// "/data/xarm6/scenario1/scenario1.yaml"
		// "/data/xarm6/scenario2/scenario2.yaml"
		// "/data/xarm6/scenario3/scenario3.yaml"
	};

	initGoogleLogging(argv);
	int clp = commandLineParser(argc, argv, scenario_file_path);
	if (clp != 0) return clp;

	const std::string project_path { getProjectPath() };
	ConfigurationReader::initConfiguration(project_path);
    YAML::Node node { YAML::LoadFile(project_path + scenario_file_path) };

	const size_t max_num_tests { node["testing"]["max_num"].as<size_t>() };
	const size_t num_random_obstacles { node["random_obstacles"]["num"].as<size_t>() };
	Eigen::Vector3f obs_dim {};
	for (size_t i = 0; i < 3; i++)
		obs_dim(i) = node["random_obstacles"]["dim"][i].as<float>();
	float min_dist_start_goal { node["robot"]["min_dist_start_goal"].as<float>() };
	float max_edge_length { node["testing"]["max_edge_length"].as<float>() };
	size_t num_layers { node["testing"]["num_layers"].as<size_t>() };
	bool write_header { node["testing"]["write_header"].as<bool>() };

	scenario::Scenario scenario(scenario_file_path, project_path);
	std::shared_ptr<base::StateSpace> ss { scenario.getStateSpace() };
	std::shared_ptr<base::State> q_start { scenario.getStart() };
	std::shared_ptr<base::State> q_goal { scenario.getGoal() };
	std::shared_ptr<env::Environment> env { scenario.getEnvironment() };
	std::unique_ptr<planning::AbstractPlanner> planner { nullptr };
	std::shared_ptr<planning::rbt::PatternTree> patern_tree { std::make_shared<planning::rbt::PatternTree>(ss, num_layers) };
	size_t num_nodes { patern_tree->getNumNodes() };
	size_t num_nodes2 { patern_tree->getNumNodes(num_layers-1) };

	std::ofstream file_input {};
	std::ofstream file_output {};
	file_input.open(project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) + "_dataset_input.csv", std::ofstream::app);
	file_output.open(project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) + "_dataset_output.csv", std::ofstream::app);
	
	if (write_header)
	{
		for (size_t j = 0; j < ss->num_dimensions; j++)
			file_input << "root_node(" << j+1 << "),";

		for (size_t i = 1; i < num_nodes; i++)
			file_input << "spine_length_" << i << ",";

		for (size_t j = 0; j < ss->num_dimensions; j++)
			file_input << "goal_node(" << j+1 << "),";

		for (size_t i = 0; i < num_nodes2; i++)
		{
			file_input << "dc_" << i << ",";

			for (size_t j = 0; j < ss->num_dimensions; j++)
			{
				file_input << "dc_" << i << "(" << j+1 << ")";
				file_input << ((i == num_nodes2-1 && j == ss->num_dimensions-1) ? "\n" : ",");
			}
		}

		for (size_t j = 0; j < ss->num_dimensions; j++)
		{
			file_output << "next_vector(" << j+1 << ")";
			file_output << (j == ss->num_dimensions-1 ? "\n" : ",");
		}
	}

	size_t num_test { 0 };
	while (num_test++ < max_num_tests)
	{
		try
		{
			LOG(INFO) << "Test number " << num_test << " of " << max_num_tests;

			generateRandomStartAndGoal(scenario, min_dist_start_goal);
			q_start = scenario.getStart();
			scenario.setGoal(q_goal);

			bool result { false };
			size_t num_obs { env->getNumObjects() };
			while (!result && num_random_obstacles > 0)
			{
				env->removeObjects(num_obs);
				initRandomObstacles(num_random_obstacles, obs_dim, scenario);
				planner = std::make_unique<planning::rbt_star::RGBMTStar>(ss, q_start, q_goal);
				RGBMTStarConfig::TERMINATE_WHEN_PATH_IS_FOUND = true;
				result = planner->solve();
				LOG(INFO) << "A path to the goal can " << (result ? "" : "not ") << "be found!";
				RGBMTStarConfig::TERMINATE_WHEN_PATH_IS_FOUND = false;
			}

			LOG(INFO) << "Using scenario:    " << project_path + scenario_file_path;
			LOG(INFO) << "Environment parts: " << env->getNumObjects();
			LOG(INFO) << "Number of DOFs:    " << ss->num_dimensions;
			LOG(INFO) << "State space type:  " << ss->getStateSpaceType();
			LOG(INFO) << "Start:             " << scenario.getStart();
			LOG(INFO) << "Goal:              " << scenario.getGoal();

			planner = std::make_unique<planning::rbt_star::RGBMTStar>(ss, q_start, q_goal);					
			result = planner->solve();

			LOG(INFO) << planner->getPlannerType() << " planning finished with " << (result ? "SUCCESS!" : "FAILURE!");
			LOG(INFO) << "Number of states: " << planner->getPlannerInfo()->getNumStates();
			LOG(INFO) << "Planning time: " << planner->getPlannerInfo()->getPlanningTime() << " [s]";
			
			if (result)
			{
				LOG(INFO) << "The path is found with the cost: " << planner->getPlannerInfo()->getOptimalCost();

				std::shared_ptr<base::Tree> tree { nullptr };
				std::vector<std::shared_ptr<base::State>> new_path {};
				ss->preprocessPath(planner->getPath(), new_path, max_edge_length);
				
				for (size_t idx = 0; idx < new_path.size()-1; idx++)
				{
					tree = patern_tree->generateLocalTree(new_path[idx]);
					// LOG(INFO) << "Path node: " << new_path[idx]->getCoord().transpose();
					// LOG(INFO) << *tree;
					
					for (size_t j = 0; j < ss->num_dimensions; j++)
						file_input << tree->getState(0)->getCoord(j) << ",";

					for (size_t i = 1; i < num_nodes; i++)
						file_input << (tree->getState(i)->getCoord() - tree->getState(i)->getParent()->getCoord()).norm() << ",";

					for (size_t j = 0; j < ss->num_dimensions; j++)
						file_input << q_goal->getCoord(j) << ",";

					for (size_t i = 0; i < num_nodes2; i++)
					{
						file_input << tree->getState(i)->getDistance() << ",";

						std::vector<float> d_c_profile { tree->getState(i)->getDistanceProfile() };
						for (size_t j = 0; j < ss->num_dimensions; j++)
						{
							file_input << d_c_profile[j]; 
							file_input << (i == num_nodes2-1 && j == ss->num_dimensions-1 ? "\n" : ",");
						}
					}

					for (size_t j = 0; j < ss->num_dimensions; j++)
					{
						file_output << new_path[idx+1]->getCoord(j) - new_path[idx]->getCoord(j);
						file_output << (j == ss->num_dimensions-1 ? "\n" : ",");
					}
				}
				LOG(INFO) << "Data considering " << new_path.size()-1 << " pattern trees is successfully written! ";
			}
			LOG(INFO) << "\n--------------------------------------------------------------------\n\n";
		}
		catch (std::exception &e)
		{
			LOG(ERROR) << e.what();
		}
	}

	LOG(INFO) << "Dataset input file is saved at: " << project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) + "_dataset_input.csv";
	LOG(INFO) << "Dataset output file is saved at: " << project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) + "_dataset_output.csv";
	file_input.close();
	file_output.close();
	
	google::ShutDownCommandLineFlags();
	return 0;
}
