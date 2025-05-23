#include "DRGBT.h"
#include "ConfigurationReader.h"
#include "CommonFunctions.h"

int main(int argc, char **argv)
{
	std::string scenario_file_path
	{
		// "/data/planar_2dof/scenario_random_obstacles/scenario_random_obstacles.yaml"

		"/data/xarm6/scenario_random_obstacles/scenario_random_obstacles.yaml"

		// "/data/spatial_10dof/scenario_random_obstacles/scenario_random_obstacles.yaml"
		// "/data/spatial_14dof/scenario_random_obstacles/scenario_random_obstacles.yaml"
		// "/data/spatial_18dof/scenario_random_obstacles/scenario_random_obstacles.yaml"
		// "/data/spatial_22dof/scenario_random_obstacles/scenario_random_obstacles.yaml"
	};
	const std::string random_scenarios_path { scenario_file_path.substr(0, scenario_file_path.find_last_of("/\\")) + "/random_scenarios.yaml" };

	// std::vector<std::string> routines		// Routines of which the time executions are stored
	// { 	
	// 	"replan [ms]",						// 0
	// 	"computeDistance [us]", 			// 1
	// 	"generateGBur [ms]", 				// 2
	// 	"generateHorizon [us]", 			// 3
	// 	"updateHorizon [us]",				// 4
	// 	"updateCurrentState [us]"			// 5
	// };

	// -------------------------------------------------------------------------------------- //

	initGoogleLogging(argv);
	int clp = commandLineParser(argc, argv, scenario_file_path);
	if (clp != 0) return clp;

	const std::string project_path { getProjectPath() };
	const std::string directory_path { project_path + scenario_file_path.substr(0, scenario_file_path.find_last_of("/\\")) + "/DRGBT_data" };
	std::filesystem::create_directory(directory_path);
	ConfigurationReader::initConfiguration(project_path);
    YAML::Node node { YAML::LoadFile(project_path + scenario_file_path) };
    YAML::Node node2 { YAML::LoadFile(project_path + random_scenarios_path) };

	size_t init_num_obs { node["random_obstacles"]["init_num"].as<size_t>() };
	const size_t max_num_obs { node["random_obstacles"]["max_num"].as<size_t>() };
	const float max_vel_obs { node["random_obstacles"]["max_vel"].as<float>() };
	const float max_acc_obs { node["random_obstacles"]["max_acc"].as<float>() };
	Eigen::Vector3f obs_dim {};
	for (size_t i = 0; i < 3; i++)
		obs_dim(i) = node["random_obstacles"]["dim"][i].as<float>();

	[[maybe_unused]] float min_dist_start_goal { node["robot"]["min_dist_start_goal"].as<float>() };
	size_t init_num_test { node["testing"]["init_num"].as<size_t>() };
	size_t init_num_success_test { node["testing"]["init_num_success"].as<size_t>() };
	const size_t max_num_tests { node["testing"]["max_num"].as<size_t>() };
	bool reach_successful_tests { node["testing"]["reach_successful_tests"].as<bool>() };
    if (DRGBTConfig::STATIC_PLANNER_TYPE == planning::PlannerType::RGBMTStar) {
        RGBMTStarConfig::TERMINATE_WHEN_PATH_IS_FOUND = true;
	}

	while (init_num_obs <= max_num_obs)
	{
		LOG(INFO) << "Number of obstacles " << init_num_obs << " of " << max_num_obs;
		
		std::ofstream output_file {};
		if (init_num_test == 1)
		{
			output_file.open(directory_path + "/results" + std::to_string(init_num_obs) + ".log", std::ofstream::out);
			output_file << "Using scenario:                                         " << scenario_file_path << std::endl;
			output_file << "Dynamic planner:                                        " << planning::PlannerType::DRGBT << std::endl;
			output_file << "Static planner for replanning:                          " << DRGBTConfig::STATIC_PLANNER_TYPE << std::endl;
			output_file << "Maximal algorithm time [s]:                             " << DRGBTConfig::MAX_PLANNING_TIME << std::endl;
			output_file << "Initial horizon size:                                   " << DRGBTConfig::INIT_HORIZON_SIZE << std::endl;
			output_file << "Treshold for the replanning assessment:                 " << DRGBTConfig::TRESHOLD_WEIGHT << std::endl;
			output_file << "Critical distance in W-space [m]:                       " << DRGBTConfig::D_CRIT << std::endl;
			output_file << "Maximal number of attempts when modifying states:       " << DRGBTConfig::MAX_NUM_MODIFY_ATTEMPTS << std::endl;
			output_file << "Number of extensions for generating a generalized bur:  " << RGBTConnectConfig::NUM_LAYERS << std::endl;
			output_file << "Number of iterations when computing a single spine:     " << RBTConnectConfig::NUM_ITER_SPINE << std::endl;
			output_file << "Using expanded bubble when generating a spine:          " << (RBTConnectConfig::USE_EXPANDED_BUBBLE ? "true" : "false") << std::endl;
			output_file << "Resolution for collision checking [m]:                  " << DRGBTConfig::RESOLUTION_COLL_CHECK << std::endl;
			output_file << "Trajectory interpolation:                               " << DRGBTConfig::TRAJECTORY_INTERPOLATION << std::endl;
			output_file << "Guaranteed safe motion:                                 " << (DRGBTConfig::GUARANTEED_SAFE_MOTION ? "true" : "false") << std::endl;
			output_file << "--------------------------------------------------------------------\n";
			output_file << "Real-time scheduling:                                   " << DRGBTConfig::REAL_TIME_SCHEDULING << std::endl;
			output_file	<< "Maximal iteration time [s]:                             " << DRGBTConfig::MAX_ITER_TIME << std::endl;
			output_file << "Maximal time of Task 1 [s]:                             " << (DRGBTConfig::REAL_TIME_SCHEDULING == planning::RealTimeScheduling::None ? 
																						 "None" : std::to_string(DRGBTConfig::MAX_TIME_TASK1)) << std::endl;
			output_file << "--------------------------------------------------------------------\n";
			output_file << "Number of obstacles:                                    " << init_num_obs << std::endl;
			output_file << "Obstacles motion:                                       " << "random" << std::endl;
			output_file << "Maximal velocity of each obstacle [m/s]:                " << max_vel_obs << std::endl;
			output_file << "Maximal acceleration of each obstacle [m/s²]:           " << max_acc_obs << std::endl;
			output_file << "--------------------------------------------------------------------\n";
			output_file.close();
		}
		
		std::vector<float> alg_times {};
		std::vector<float> iter_times {};
		std::vector<float> num_iters {};
		std::vector<float> path_lengths {};
		size_t num_test { init_num_test };
		size_t num_success_tests { init_num_success_test };

		while (true)
		{
			try
			{
				scenario::Scenario scenario(scenario_file_path, project_path);
				std::shared_ptr<base::StateSpace> ss { scenario.getStateSpace() };
				std::shared_ptr<env::Environment> env { scenario.getEnvironment() };
				std::unique_ptr<planning::AbstractPlanner> planner { nullptr };
				
				env->setBaseRadius(std::max(ss->robot->getCapsuleRadius(0), ss->robot->getCapsuleRadius(1)) + obs_dim.norm());
				env->setRobotMaxVel(ss->robot->getMaxVel(0)); 	// Only velocity of the first joint matters

				// First option (generating here in the code):
				// if (min_dist_start_goal > 0)
				// 	generateRandomStartAndGoal(scenario, min_dist_start_goal);
				// initRandomObstacles(init_num_obs, obs_dim, scenario, max_vel_obs, max_acc_obs);
				// ------------------------------------------------------------------------------- //

				// Second option (reading from a yaml file):
				Eigen::VectorXf start { Eigen::VectorXf::Zero(ss->num_dimensions) };
				Eigen::VectorXf goal { Eigen::VectorXf::Zero(ss->num_dimensions) };
				for (size_t i = 0; i < ss->num_dimensions; i++)
				{
					start(i) = node2["scenario_" + std::to_string(init_num_obs)]["run_" + std::to_string(num_test-1)]["start"][i].as<float>();
					goal(i) = node2["scenario_" + std::to_string(init_num_obs)]["run_" + std::to_string(num_test-1)]["goal"][i].as<float>();
				}
				scenario.setStart(ss->getNewState(start));
				scenario.setGoal(ss->getNewState(goal));

				Eigen::Vector3f pos {}, vel {};
				for (size_t j = 0; j < init_num_obs; j++)
				{
					for (size_t i = 0; i < 3; i++)
					{
						pos(i) = node2["scenario_" + std::to_string(init_num_obs)]["run_" + std::to_string(num_test-1)]
								 ["object_" + std::to_string(j)]["pos"][i].as<float>();
						vel(i) = node2["scenario_" + std::to_string(init_num_obs)]["run_" + std::to_string(num_test-1)]
								 ["object_" + std::to_string(j)]["vel"][i].as<float>();
					}
					
					std::shared_ptr<env::Object> object { nullptr };
					object = std::make_shared<env::Box>(obs_dim, pos, Eigen::Quaternionf::Identity(), "dynamic_obstacle");
					object->setMaxVel(max_vel_obs);
					object->setMaxAcc(max_acc_obs);
					env->addObject(object, vel);
				}
				// ------------------------------------------------------------------------------- //

				LOG(INFO) << "Test number:       " << num_test;
				LOG(INFO) << "Using scenario:    " << project_path + scenario_file_path;
				LOG(INFO) << "Environment parts: " << env->getNumObjects();
				LOG(INFO) << "Number of DOFs:    " << ss->num_dimensions;
				LOG(INFO) << "State space type:  " << ss->getStateSpaceType();
				LOG(INFO) << "Start:             " << scenario.getStart();
				LOG(INFO) << "Goal:              " << scenario.getGoal();
				
				planner = std::make_unique<planning::drbt::DRGBT>(ss, scenario.getStart(), scenario.getGoal());
				bool result { planner->solve() };
				
				LOG(INFO) << planner->getPlannerType() << " planning finished with " << (result ? "SUCCESS!" : "FAILURE!");
				LOG(INFO) << "Number of iterations: " << planner->getPlannerInfo()->getNumIterations();
				LOG(INFO) << "Algorithm time:       " << planner->getPlannerInfo()->getPlanningTime() << " [s]";
				LOG(INFO) << "Task 1 interrupted:   " << (planner->getPlannerInfo()->getTask1Interrupted() ? "true" : "false");
				// LOG(INFO) << "Planner data is saved at: " << directory_path + "/test" + std::to_string(init_num_obs) + "_" + std::to_string(num_test) + ".log";
				// planner->outputPlannerData(directory_path + "/test" + std::to_string(init_num_obs) + "_" + std::to_string(num_test) + ".log");

				float path_length { 0 };
				if (result)
				{
					// LOG(INFO) << "Found path: ";
					std::vector<std::shared_ptr<base::State>> path { planner->getPath() };
					for (size_t i = 0; i < path.size(); i++)
					{
						// std::cout << i << ": " << path.at(i)->getCoord().transpose() << std::endl;
						if (i > 0) path_length += ss->getNorm(path.at(i-1), path.at(i));
					}
					path_lengths.emplace_back(path_length);
					alg_times.emplace_back(planner->getPlannerInfo()->getPlanningTime());
					iter_times.emplace_back(planner->getPlannerInfo()->getPlanningTime() / planner->getPlannerInfo()->getNumIterations());
					num_iters.emplace_back(planner->getPlannerInfo()->getNumIterations());
					num_success_tests++;
				}

				output_file.open(directory_path + "/results" + std::to_string(init_num_obs) + ".log", std::ofstream::app);
				output_file << "Test number: " << num_test << std::endl;
				output_file << "Number of successful tests: " << num_success_tests << " of " << num_test 
							<< " = " << 100.0 * num_success_tests / num_test << " %" << std::endl;
				output_file << "Success:\n" << result << std::endl;
				output_file << "Number of iterations:\n" << planner->getPlannerInfo()->getNumIterations() << std::endl;
				output_file << "Algorithm execution time [s]:\n" << planner->getPlannerInfo()->getPlanningTime() << std::endl;
				output_file << "Path length [rad]:\n" << (result ? path_length : INFINITY) << std::endl;
				output_file << "Task 1 interrupted:\n" << planner->getPlannerInfo()->getTask1Interrupted() << std::endl;

				// if (result)
				// {
				// 	std::vector<std::vector<float>> routine_times { planner->getPlannerInfo()->getRoutineTimes() };
				// 	for (size_t idx = 0; idx < routines.size(); idx++)
				// 	{
				// 		// LOG(INFO) << "Routine " << routines[idx];
				// 		// LOG(INFO) << "\tAverage time: " << getMean(routine_times[idx]) << " +- " << getStd(routine_times[idx]);
				// 		// LOG(INFO) << "\tMaximal time: " << *std::max_element(routine_times[idx].begin(), routine_times[idx].end());
				// 		// LOG(INFO) << "\tData size: " << routine_times[idx].size(); 
						
				// 		output_file << "Routine " << routines[idx] << " times: " << std::endl;
				// 		for (float t : routine_times[idx])
				// 			output_file << t << std::endl;
				// 	}
				// }

				output_file << "--------------------------------------------------------------------\n";
				output_file.close();
				LOG(INFO) << "Number of successful tests: " << num_success_tests << " of " << num_test 
						  << " = " << 100.0 * num_success_tests / num_test << " %";
				LOG(INFO) << "--------------------------------------------------------------------\n\n";
			}
			catch (std::exception &e)
			{
				LOG(ERROR) << e.what();
			}

			num_test++;
			if ((reach_successful_tests && num_success_tests == max_num_tests) || (!reach_successful_tests && num_test > max_num_tests))
				break;
		}

		init_num_test = 1;
		init_num_success_test = 0;
		init_num_obs += (init_num_obs > 0) ? std::pow(10, std::floor(std::log10(init_num_obs))) : 1;

		LOG(INFO) << "Success rate:                     " << 100.0 * num_success_tests / (num_test - 1) << " [%]";
		LOG(INFO) << "Average algorithm execution time: " << getMean(alg_times) << " +- " << getStd(alg_times) << " [s]";
		LOG(INFO) << "Average iteration execution time: " << getMean(iter_times) * 1e3 << " +- " << getStd(iter_times) * 1e3 << " [ms]";
		LOG(INFO) << "Average number of iterations:     " << getMean(num_iters) << " +- " << getStd(num_iters);
		LOG(INFO) << "Average path length:              " << getMean(path_lengths) << " +- " << getStd(path_lengths) << " [rad]";
		LOG(INFO) << "\n--------------------------------------------------------------------\n\n";
	}

	google::ShutDownCommandLineFlags();
	return 0;
}
