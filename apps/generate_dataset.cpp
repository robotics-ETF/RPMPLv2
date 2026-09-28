#include "PatternTree.h"
#include "RGBMTStar.h"
#include "ConfigurationReader.h"
#include "CommonFunctions.h"

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
	const std::string directory_path { project_path + scenario_file_path.substr(0, scenario_file_path.find_last_of("/\\")) + "/datasets" };
	std::filesystem::create_directory(directory_path);
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
	std::shared_ptr<planning::rbt::PatternTree> pattern_tree { std::make_shared<planning::rbt::PatternTree>(ss, num_layers) };
	std::vector<size_t> num_nodes(num_layers+1, 1);
	for (size_t i = 1; i <= num_layers; i++)
		num_nodes[i] = pattern_tree->getNumNodes(i);
	
	std::ofstream output_file {};
	output_file.open(directory_path + "/dataset.csv", std::ofstream::app);
	
	if (write_header)
	{
		// Input:
		for (size_t j = 0; j < ss->num_dimensions; j++)
			output_file << "start_node(" << j+1 << "),";

		for (size_t j = 0; j < ss->num_dimensions; j++)
			output_file << "goal_node(" << j+1 << "),";

		for (size_t i = 1; i < num_nodes[num_layers]; i++)
			output_file << "spine_length" << i << ",";

		for (size_t i = 0; i < num_nodes[num_layers-1]; i++)
		{
			// output_file << "dc" << i << ","; 	// If only distance to obstacles is used
			
			for (size_t j = 0; j < ss->num_dimensions; j++)
				output_file << "dc" << i << "_link" << j+1 << ",";
		}

		for (size_t i = 0; i < num_nodes[num_layers-1]; i++)
		{
			for (size_t j = 0; j < ss->num_dimensions; j++)
			{
				for (size_t k = 0; k < 3; k++)
					output_file << "OR_vector" << i << "_link" << j+1 << "(" << k+1 << "),";
			}
		}

		// Output:
		// for (size_t j = 0; j < ss->num_dimensions; j++)
		// 	output_file << "next_vector(" << j+1 << "),";

		for (size_t i = 1; i < num_nodes[num_layers-1]; i++)
			output_file << "prob" << i << ",";

		output_file << "\n";
	}

	size_t num_test { 0 };
	while (num_test++ < max_num_tests)
	{
		try
		{
			LOG(INFO) << "Test number " << num_test << " of " << max_num_tests;

			generateRandomStartAndGoal(scenario, min_dist_start_goal);
			q_start = scenario.getStart();
			q_goal = scenario.getGoal();

			if (num_random_obstacles > 0)
				initRandomObstacles(num_random_obstacles, obs_dim, scenario);

			LOG(INFO) << "Using scenario:    " << project_path + scenario_file_path;
			LOG(INFO) << "Environment parts: " << env->getNumObjects();
			LOG(INFO) << "Number of DOFs:    " << ss->num_dimensions;
			LOG(INFO) << "State space type:  " << ss->getStateSpaceType();
			LOG(INFO) << "Start:             " << scenario.getStart();
			LOG(INFO) << "Goal:              " << scenario.getGoal();

			std::shared_ptr<base::Tree> tree { pattern_tree->generateLocalTree(q_start) };
			// LOG(INFO) << *tree;
			
			// Generate input data:
			for (size_t j = 0; j < ss->num_dimensions; j++)
				output_file << q_start->getCoord(j) << ",";

			for (size_t j = 0; j < ss->num_dimensions; j++)
				output_file << q_goal->getCoord(j) << ",";

			for (size_t i = 1; i < num_nodes[num_layers]; i++)
				output_file << (tree->getState(i)->getCoord() - tree->getState(i)->getParent()->getCoord()).norm() << ",";

			for (size_t i = 0; i < num_nodes[num_layers-1]; i++)
			{
				// output_file << tree->getState(i)->getDistance() << ",";	// If only distance to obstacles is used

				std::vector<float> d_c_profile { tree->getState(i)->getDistanceProfile() };
				for (size_t j = 0; j < ss->num_dimensions; j++)
					output_file << d_c_profile[j] << ",";
			}

			for (size_t i = 0; i < num_nodes[num_layers-1]; i++)
			{
				std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points { tree->getState(i)->getNearestPoints() };
				Eigen::Vector3f R {}, R_min {}, O {}, O_min {}, OR {};
				for (size_t j = 0; j < ss->num_dimensions; j++)
				{
					float d_min { RealVectorSpaceConfig::MAX_DISTANCE };
					for (size_t k = 0; k < env->getNumObjects(); k++)
					{
						R = nearest_points->at(k).col(j).head(3);
						O = nearest_points->at(k).col(j).tail(3);
						R += (O - R).normalized() * ss->robot->getCapsuleRadius(j);
						if ((R - O).norm() < d_min)
						{
							R_min = R;
							O_min = O;
							d_min = (R_min - O_min).norm();
						}
					}

					OR = (R_min - O_min).normalized();
					output_file << OR.x() << "," << OR.y() << "," << OR.z() << ",";
				}
			}

			// Generate output data:
			// 1. option (next target vector)
			// for (size_t j = 0; j < ss->num_dimensions; j++)
			// 	output_file << new_path[idx+1]->getCoord(j) - new_path[idx]->getCoord(j) << ",";

			// 2. option (probabilities of each node from the pattern tree)
			std::vector<float> weights(num_nodes[num_layers-1]-1, 0.0);
			for (size_t i = 1; i < num_nodes[num_layers-1]; i++)
			{
				float cost = (tree->getState(i)->getCoord() - tree->getState(i)->getParent()->getCoord()).norm(); 	// Initial cost 
				if (cost > RealVectorSpaceConfig::EQUALITY_THRESHOLD)
				{
					LOG(INFO) << "Planning from node " << i << ": " << tree->getState(i)->getCoord().transpose();
					planner = std::make_unique<planning::rbt_star::RGBMTStar>(ss, ss->getNewState(tree->getState(i)->getCoord()), q_goal);
					bool result { planner->solve() };
					LOG(INFO) << planner->getPlannerType() << " planning finished with " << (result ? "SUCCESS!" : "FAILURE!");
					
					if (result)
					{
						std::shared_ptr<std::vector<std::shared_ptr<base::State>>> children { tree->getState(i)->getChildren() };
						float clearance_factor { 1.0f };
						for (const std::shared_ptr<base::State> &child : *children)
							clearance_factor *= (child->getCoord() - tree->getState(i)->getCoord()).norm();
						
						if (clearance_factor > 0)
							weights[i-1] = clearance_factor / cost;

						std::cout << "Clearance factor: " << clearance_factor << "\t";
						std::cout << "Final cost: " << cost / clearance_factor << "\n";
					}
				}
			}

			size_t start_i { 0 };
			size_t num { 2*ss->num_dimensions };
			while (start_i < num_nodes[num_layers-1]-1)
			{
				float weights_sum = std::accumulate(weights.begin() + start_i, weights.begin() + start_i + num, 0.0f);
				if (weights_sum == 0) 	// Just to avoid "0/0" case.
					weights_sum = 1;

				for (size_t i = start_i; i < start_i + num; i++)
				{
					LOG(INFO) << "Probability " << i+1 << ": " << weights[i] / weights_sum;
					output_file << weights[i] / weights_sum << ",";
				}
				start_i += num;
				num = 2*ss->num_dimensions - 1;
			}

			output_file << "\n";

			LOG(INFO) << "Data is successfully written! ";
			LOG(INFO) << "\n--------------------------------------------------------------------\n\n";
		}
		catch (std::exception &e)
		{
			LOG(ERROR) << e.what();
		}
	}

	LOG(INFO) << "Dataset file is saved at: " << directory_path + "/dataset.csv";
	output_file.close();
	
	google::ShutDownCommandLineFlags();
	return 0;
}
