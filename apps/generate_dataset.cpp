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
	std::shared_ptr<planning::rbt::PatternTree> patern_tree { std::make_shared<planning::rbt::PatternTree>(ss, num_layers) };
	size_t num_nodes { patern_tree->getNumNodes() };
	size_t num_nodes2 { patern_tree->getNumNodes(num_layers-1) };

	std::ofstream output_file {};
	output_file.open(directory_path + "/dataset.csv", std::ofstream::app);
	
	if (write_header)
	{
		for (size_t j = 0; j < ss->num_dimensions; j++)
			output_file << "root_node(" << j+1 << "),";

		for (size_t i = 1; i < num_nodes; i++)
			output_file << "spine_length" << i << ",";

		for (size_t j = 0; j < ss->num_dimensions; j++)
			output_file << "goal_node(" << j+1 << "),";

		for (size_t i = 0; i < num_nodes2; i++)
		{
			// file_input << "dc_" << i << ",";

			for (size_t j = 0; j < ss->num_dimensions; j++)
				output_file << "dc" << i << "_link" << j+1 << ",";
		}

		// for (size_t i = 0; i < num_nodes2; i++)
		// {
		// 	for (size_t j = 0; j < ss->num_dimensions; j++)
		// 	{
		// 		for (size_t k = 0; k < 2; k++)	// for planar robot
		// 			output_file << "OR_vector" << i << "_link" << j+1 << "(" << k+1 << "),";
		// 	}
		// }

		for (size_t j = 0; j < ss->num_dimensions; j++)
		{
			output_file << "next_vector(" << j+1 << ")";
			output_file << (j == ss->num_dimensions-1 ? "\n" : ",");
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
			q_goal = scenario.getGoal();
			// scenario.setGoal(q_goal);

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
						output_file << tree->getState(0)->getCoord(j) << ",";

					for (size_t i = 1; i < num_nodes; i++)
						output_file << (tree->getState(i)->getCoord() - tree->getState(i)->getParent()->getCoord()).norm() << ",";

					for (size_t j = 0; j < ss->num_dimensions; j++)
						output_file << q_goal->getCoord(j) << ",";

					for (size_t i = 0; i < num_nodes2; i++)
					{
						// output_file << tree->getState(i)->getDistance() << ",";

						std::vector<float> d_c_profile { tree->getState(i)->getDistanceProfile() };
						for (size_t j = 0; j < ss->num_dimensions; j++)
							output_file << d_c_profile[j] << ",";
					}

					// for (size_t i = 0; i < num_nodes2; i++)
					// {
					// 	std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points { tree->getState(i)->getNearestPoints() };
					// 	Eigen::Vector3f R {}, R_min {}, O {}, O_min {};
					// 	for (size_t j = 0; j < ss->num_dimensions; j++)
					// 	{
					// 		float d_min { INFINITY };
					// 		for (size_t k = 0; k < env->getNumObjects(); k++)
					// 		{
					// 			R = nearest_points->at(k).col(j).head(3);
					// 			O = nearest_points->at(k).col(j).tail(3);
					// 			R += (O - R).normalized() * ss->robot->getCapsuleRadius(j);
					// 			if ((R - O).norm() < d_min)
					// 			{
					// 				R_min = R;
					// 				O_min = O;
					// 				d_min = (R_min - O_min).norm();
					// 			}
					// 		}
					// 		output_file << (R - O).x() << "," << (R - O).y() << ","; // << (R - O).z() << ",";	// Uncomment for spacial robot
					// 	}
					// }

					for (size_t j = 0; j < ss->num_dimensions; j++)
					{
						output_file << new_path[idx+1]->getCoord(j) - new_path[idx]->getCoord(j);
						output_file << (j == ss->num_dimensions-1 ? "\n" : ",");
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

	LOG(INFO) << "Dataset file is saved at: " << directory_path + "/dataset.csv";
	output_file.close();
	
	google::ShutDownCommandLineFlags();
	return 0;
}
