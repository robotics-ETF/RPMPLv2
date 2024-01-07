#include <DRGBT.h>
#include <ConfigurationReader.h>
#include <CommonFunctions.h>

int main(int argc, char **argv)
{
	// std::string scenario_file_path = "/data/planar_2dof/scenario_test/scenario_test.yaml";
	// std::string scenario_file_path = "/data/planar_2dof/scenario1/scenario1.yaml";
	// std::string scenario_file_path = "/data/planar_2dof/scenario2/scenario2.yaml";

	// std::string scenario_file_path = "/data/xarm6/scenario_test/scenario_test.yaml";
	// std::string scenario_file_path = "/data/xarm6/scenario1/scenario1.yaml";
	// std::string scenario_file_path = "/data/xarm6/scenario2/scenario2.yaml";
	std::string scenario_file_path = "/data/xarm6/scenario_real_time/scenario_real_time.yaml";

	std::vector<std::string> routines = { 	// Routines of which the time executions are stored
		"replan [ms]",
		"checkMotionValidity [us]", 
		"computeDistance [us]", 
		"generateGBur [ms]", 
		"generateHorizon [us]", 
		"updateHorizon [us]"
	};

	// -------------------------------------------------------------------------------------- //

	initGoogleLogging(argv);
	int clp = commandLineParser(argc, argv, scenario_file_path);
	if (clp != 0) return clp;

	const std::string project_path = getProjectPath();
	ConfigurationReader::initConfiguration(project_path);
    YAML::Node node = YAML::LoadFile(project_path + scenario_file_path);
	int num_random_obstacles_init = node["testing"]["num_random_obstacles_init"].as<int>();
	const int max_num_random_obstacles = node["testing"]["max_num_random_obstacles"].as<int>();
	const float max_obs_vel = node["testing"]["max_obs_vel"].as<float>();
	const float max_obs_acc = node["testing"]["max_obs_acc"].as<float>();
	DRGBTConfig::MAX_NUM_VALIDITY_CHECKS = std::ceil((max_obs_vel * DRGBTConfig::MAX_ITER_TIME * 0.001) / 0.01); // In order to obtain check when obstacle moves at most 1 [cm]
	int num_test_init = node["testing"]["num_test_init"].as<int>();
	int num_success_test_init = node["testing"]["num_success_test_init"].as<int>();
	const int max_num_tests = node["testing"]["max_num_tests"].as<int>();
	bool successful_tests = node["testing"]["successful_tests"].as<bool>();
    if (DRGBTConfig::STATIC_PLANNER_NAME == "RGBMT*") 
        RGBMTStarConfig::TERMINATE_WHEN_PATH_IS_FOUND = true;

	while (num_random_obstacles_init <= max_num_random_obstacles)
	{
		LOG(INFO) << "Number of obstacles " << num_random_obstacles_init << " of " << max_num_random_obstacles;
		
		std::ofstream output_file;
		if (num_test_init == 1)
		{
			output_file.open(project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) + 
							"_routine_times" + std::to_string(num_random_obstacles_init) + ".log", std::ofstream::out);
			output_file << "Dynamic planner: DRGBT" << std::endl;
			output_file	<< "Maximal iteration time: " << DRGBTConfig::MAX_ITER_TIME << " [ms]" << std::endl;
			output_file << "Maximal algorithm time: " << DRGBTConfig::MAX_PLANNING_TIME << " [ms]" << std::endl;
			output_file << "Initial horizon size: " << DRGBTConfig::INIT_HORIZON_SIZE << std::endl;
			output_file << "Maximal angular velocity of each robot's joint: " << DRGBTConfig::MAX_ANG_VEL << " [rad/s]" << std::endl;
			output_file << "Treshold for the replanning assessment: " << DRGBTConfig::TRESHOLD_WEIGHT << std::endl;
			output_file << "Critical distance in W-space: " << DRGBTConfig::D_CRIT << " [m]" << std::endl;
			output_file << "Maximal number of attempts when modifying bad or critical states: " << DRGBTConfig::MAX_NUM_MODIFY_ATTEMPTS << std::endl;
			output_file << "Static planner for replanning: " << DRGBTConfig::STATIC_PLANNER_NAME << std::endl;
			output_file << "Number of layers (extensions) for generating a generalized bur: " << RGBTConnectConfig::NUM_LAYERS << std::endl;
			output_file << "Number of iterations when computing a single spine: " << RBTConnectConfig::NUM_ITER_SPINE << std::endl;
			output_file << "Using expanded bubble when generating a spine: " << (RBTConnectConfig::USE_EXPANDED_BUBBLE ? "true" : "false") << std::endl;
			output_file << "Real-time scheduling: " << (DRGBTConfig::REAL_TIME_SCHEDULING.empty() ? "NONE" : DRGBTConfig::REAL_TIME_SCHEDULING) << std::endl;
			output_file << "Maximal time of Task 1: " << (DRGBTConfig::REAL_TIME_SCHEDULING.empty() ? "NONE" : std::to_string(DRGBTConfig::MAX_TIME_TASK1) + " [ms]") << std::endl;
			output_file << "Number of obstacles: " << num_random_obstacles_init << std::endl;
			output_file << "Maximal velocity of each obstacle: " << max_obs_vel << " [m/s] " << std::endl;
			output_file << "Maximal acceleration of each obstacle: " << max_obs_acc << " [m/s²] " << std::endl;
			output_file << "--------------------------------------------------------------------\n";
			output_file.close();
		}
		
		std::vector<float> alg_times;
		std::vector<float> iter_times;
		std::vector<float> path_lengths;
		int num_test = num_test_init;
		int num_success_tests = num_success_test_init;

		while (true)
		{
			try
			{
				scenario::Scenario scenario(scenario_file_path, project_path);
				std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();
				std::shared_ptr<base::State> start = scenario.getStart();
				std::shared_ptr<base::State> goal = scenario.getGoal();
				std::shared_ptr<env::Environment> env = scenario.getEnvironment();
				initRandomObstacles(scenario, max_obs_vel, max_obs_acc, num_random_obstacles_init);

				LOG(INFO) << "Test number: " << num_test;
				LOG(INFO) << "Using scenario: " << project_path + scenario_file_path;
				LOG(INFO) << "Environment parts: " << env->getParts().size();
				LOG(INFO) << "Number of DOFs: " << ss->getNumDimensions();
				LOG(INFO) << "State space type: " << ss->getStateSpaceType();
				LOG(INFO) << "Start: " << scenario.getStart();
				LOG(INFO) << "Goal: " << scenario.getGoal();
				
				std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::drbt::DRGBT>(ss, start, goal);
				bool result = planner->solve();
				
				LOG(INFO) << "DRGBT planning finished with " << (result ? "SUCCESS!" : "FAILURE!");
				LOG(INFO) << "Number of iterations: " << planner->getPlannerInfo()->getNumIterations();
				LOG(INFO) << "Algorithm time: " << planner->getPlannerInfo()->getPlanningTime() << " [ms]";
				LOG(INFO) << "Task 1 interrupted: " << (planner->getPlannerInfo()->getTask1Interrupted() ? "true" : "false");
				// LOG(INFO) << "Planner data is saved at: " << project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) 
				// 		  	 + "_drgbt_test" + std::to_string(num_test) + ".log";
				// planner->outputPlannerData(project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) 
				// 						   + "_drgbt_test" + std::to_string(num_test) + ".log");

				float path_length = 0;
				if (result)
				{
					// LOG(INFO) << "Found path: ";
					std::vector<std::shared_ptr<base::State>> path = planner->getPath();
					for (int i = 0; i < path.size(); i++)
					{
						// std::cout << i << ": " << path.at(i)->getCoord().transpose() << std::endl;
						if (i > 0) path_length += ss->getNorm(path.at(i-1), path.at(i));
					}
					path_lengths.emplace_back(path_length);
					alg_times.emplace_back(planner->getPlannerInfo()->getPlanningTime());
					iter_times.emplace_back(planner->getPlannerInfo()->getPlanningTime() / planner->getPlannerInfo()->getNumIterations());
					num_success_tests++;
				}

				output_file.open(project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) + 
								 "_routine_times" + std::to_string(num_random_obstacles_init) + ".log", std::ofstream::app);
				output_file << "Test number: " << num_test << std::endl;
				output_file << "Number of successful tests: " << num_success_tests << " of " << num_test 
							<< " = " << 100.0 * num_success_tests / num_test << " %" << std::endl;
				output_file << "Success:\n" << result << std::endl;
				output_file << "Number of iterations:\n" << planner->getPlannerInfo()->getNumIterations() << std::endl;
				output_file << "Algorithm execution time [ms]:\n" << planner->getPlannerInfo()->getPlanningTime() << std::endl;
				output_file << "Path length [rad]:\n" << (result ? path_length : INFINITY) << std::endl;
				output_file << "Task 1 interrupted:\n" << planner->getPlannerInfo()->getTask1Interrupted() << std::endl;

				if (result)
				{
					std::vector<std::vector<int>> routine_times = planner->getPlannerInfo()->getRoutineTimes();
					for (int idx = 0; idx < routines.size(); idx++)
					{
						// LOG(INFO) << "Routine " << routines[idx];
						// LOG(INFO) << "\tAverage time: " << getMean(routine_times[idx]) << " +- " << getStd(routine_times[idx]);
						// LOG(INFO) << "\tMaximal time: " << *std::max_element(routine_times[idx].begin(), routine_times[idx].end());
						// LOG(INFO) << "\tData size: " << routine_times[idx].size(); 
						
						output_file << "Routine " << routines[idx] << " times: " << std::endl;
						for (float t : routine_times[idx])
							output_file << t << std::endl;
					}
				}

				output_file << "--------------------------------------------------------------------\n";
				output_file.close();
				LOG(INFO) << "Number of successful tests: " << num_success_tests << " of " << num_test 
						  << " = " << 100.0 * num_success_tests / num_test << " %";
				LOG(INFO) << "\n--------------------------------------------------------------------\n\n";
				num_test++;
			}
			catch (std::exception &e)
			{
				LOG(ERROR) << e.what();
			}

			if (successful_tests && num_success_tests == max_num_tests || !successful_tests && num_test > max_num_tests)
				break;
		}

		num_test_init = 1;
		num_success_test_init = 0;
		num_random_obstacles_init += std::pow(10, std::floor(std::log10(num_random_obstacles_init)));

		LOG(INFO) << "Success rate: " << 100.0 * num_success_tests / (num_test - 1) << " %";
		LOG(INFO) << "Average algorithm execution time: " << getMean(alg_times) << " +- " << getStd(alg_times) << " [ms]";
		LOG(INFO) << "Average iteration execution time: " << getMean(iter_times) << " +- " << getStd(iter_times) << " [ms]";
		LOG(INFO) << "Average path length: " << getMean(path_lengths) << " +- " << getStd(path_lengths) << " [rad]";
		LOG(INFO) << "\n--------------------------------------------------------------------\n\n";
	}

	google::ShutDownCommandLineFlags();
	return 0;
}
