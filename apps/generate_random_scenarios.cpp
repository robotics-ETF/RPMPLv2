#include "ConfigurationReader.h"
#include "CommonFunctions.h"

int main(int argc, char **argv)
{
	std::string scenario_file_path
	{
		// "/data/planar_2dof/scenario_random_obstacles/scenario_random_obstacles.yaml"

		// "/data/xarm6/scenario_random_obstacles/scenario_random_obstacles.yaml"

		// "/data/spatial_10dof/scenario_random_obstacles/scenario_random_obstacles.yaml"
		// "/data/spatial_14dof/scenario_random_obstacles/scenario_random_obstacles.yaml"
		// "/data/spatial_18dof/scenario_random_obstacles/scenario_random_obstacles.yaml"
		"/data/spatial_22dof/scenario_random_obstacles/scenario_random_obstacles.yaml"
	};
	const std::string output_file_name { "random_scenarios.yaml" };

	// -------------------------------------------------------------------------------------- //

	initGoogleLogging(argv);
	int clp = commandLineParser(argc, argv, scenario_file_path);
	if (clp != 0) return clp;

	const std::string project_path { getProjectPath() };
	const std::string directory_path { project_path + scenario_file_path.substr(0, scenario_file_path.find_last_of("/\\")) };
	std::filesystem::create_directory(directory_path);
	ConfigurationReader::initConfiguration(project_path);
    YAML::Node node { YAML::LoadFile(project_path + scenario_file_path) };

	size_t init_num_obs { node["random_obstacles"]["init_num"].as<size_t>() };
	const size_t max_num_obs { node["random_obstacles"]["max_num"].as<size_t>() };
	const float max_vel_obs { node["random_obstacles"]["max_vel"].as<float>() };
	const float max_acc_obs { node["random_obstacles"]["max_acc"].as<float>() };
	Eigen::Vector3f obs_dim {};
	for (size_t i = 0; i < 3; i++)
		obs_dim(i) = node["random_obstacles"]["dim"][i].as<float>();

	float min_dist_start_goal { node["robot"]["min_dist_start_goal"].as<float>() };
	size_t init_num_test { node["testing"]["init_num"].as<size_t>() };
	const size_t max_num_tests { node["testing"]["max_num"].as<size_t>() };

	std::ofstream output_file {};
	output_file.open(directory_path + "/" + output_file_name, std::ofstream::out);
	
	while (init_num_obs <= max_num_obs)
	{
		LOG(INFO) << "Number of obstacles " << init_num_obs << " of " << max_num_obs;
		try
		{
			output_file << "scenario_" << init_num_obs << ": \n";
			for (size_t i = init_num_test-1; i < max_num_tests; i++)
			{
				LOG(INFO) << "Test num. " << i+1 << " of " << max_num_tests << "\n";
				scenario::Scenario scenario(scenario_file_path, project_path);
				std::shared_ptr<base::StateSpace> ss { scenario.getStateSpace() };
				std::shared_ptr<env::Environment> env { scenario.getEnvironment() };
				
				env->setBaseRadius(std::max(ss->robot->getCapsuleRadius(0), ss->robot->getCapsuleRadius(1)) + obs_dim.norm());
				env->setRobotMaxVel(ss->robot->getMaxVel(0)); 	// Only velocity of the first joint matters

				generateRandomStartAndGoal(scenario, min_dist_start_goal);
				output_file << "  run_" << i << ": \n";
				output_file << "    start: [";
				for (size_t idx = 0; idx < ss->num_dimensions; idx++)
					output_file << scenario.getStart()->getCoord(idx) << (idx < ss->num_dimensions-1 ? ", " : "");
				output_file << "]\n";

				output_file << "    goal:  [";
				for (size_t idx = 0; idx < ss->num_dimensions; idx++)
					output_file << scenario.getGoal()->getCoord(idx) << (idx < ss->num_dimensions-1 ? ", " : "");
				output_file << "]\n";

				initRandomObstacles(init_num_obs, obs_dim, scenario, max_vel_obs, max_acc_obs);
				std::vector<std::shared_ptr<env::Object>> objects { env->getObjects() };
				for (size_t idx = 0; idx < objects.size(); idx++)
				{
					if (objects[idx]->getLabel() != "dynamic_obstacle")
						continue;

					Eigen::Vector3f pos = objects[idx]->getPosition();
					Eigen::Vector3f vel = objects[idx]->getVelocity();
					output_file << "    object_" << idx << ": \n";
					output_file << "      pos: [" << pos.x() << ", " << pos.y() << ", " << pos.z() << "]\n";
					output_file << "      vel: [" << vel.x() << ", " << vel.y() << ", " << vel.z() << "]\n";
				}
			}
		}
		catch (std::exception &e)
		{
			LOG(ERROR) << e.what();
		}

		init_num_obs += (init_num_obs > 0) ? std::pow(10, std::floor(std::log10(init_num_obs))) : 1;
	}
	output_file.close();
	LOG(INFO) << "Random scenarios are saved to the file: " << directory_path + "/" + output_file_name;

	google::ShutDownCommandLineFlags();
	return 0;
}
