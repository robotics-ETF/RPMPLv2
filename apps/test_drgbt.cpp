#include "DRGBT.h"
#include "ConfigurationReader.h"
#include "CommonFunctions.h"

int main(int argc, char **argv)
{
	std::string scenario_file_path
	{
		// "/data/planar_2dof/scenario_test/scenario_test.yaml"
		// "/data/planar_2dof/scenario1/scenario1.yaml"
		// "/data/planar_2dof/scenario2/scenario2.yaml"

		// "/data/xarm6/scenario_test/scenario_test.yaml"
		"/data/xarm6/scenario1/scenario1.yaml"
		// "/data/xarm6/scenario2/scenario2.yaml"
	};
	// -------------------------------------------------------------------------------------- //

	initGoogleLogging(argv);
	int clp = commandLineParser(argc, argv, scenario_file_path);
	if (clp != 0) return clp;

	const std::string project_path { getProjectPath() };
	ConfigurationReader::initConfiguration(project_path);
    YAML::Node node { YAML::LoadFile(project_path + scenario_file_path) };
	const float max_vel_obs { node["testing"]["max_vel_obs"].as<float>() };
	const size_t max_num_tests { node["testing"]["max_num"].as<size_t>() };
    if (DRGBTConfig::STATIC_PLANNER_TYPE == planning::PlannerType::RGBMTStar) {
        RGBMTStarConfig::TERMINATE_WHEN_PATH_IS_FOUND = true;
	}

	std::ofstream output_file {};
	output_file.open(project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) + "_DRGBT_data/test.log", std::ofstream::out);
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
	output_file << "Obstacles motion:                                       " << "predefined" << std::endl;
	output_file << "Maximal velocity of each obstacle [m/s]:                " << max_vel_obs << std::endl;
	output_file << "--------------------------------------------------------------------\n";
	output_file.close();
	
	std::vector<float> alg_times {};
	std::vector<float> iter_times {};
	std::vector<float> num_iters {};
	std::vector<float> path_lengths {};
	size_t num_test { 0 };
	size_t num_success_tests { 0 };

	while (num_test++ < max_num_tests)
	{
		try
		{
			scenario::Scenario scenario(scenario_file_path, project_path);
			std::shared_ptr<base::StateSpace> ss { scenario.getStateSpace() };
			std::shared_ptr<env::Environment> env { scenario.getEnvironment() };
			std::unique_ptr<planning::AbstractPlanner> planner { nullptr };

			float vel { float(rand()) / RAND_MAX * max_vel_obs };
			for (auto obj : env->getObjects())
			{
				if (obj->getLabel() != "ground")
				{
					obj->setLabel("dynamic_obstacle");
					obj->setMaxVel(vel);
				}
			}

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
			// LOG(INFO) << "Planner data is saved at: " << project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) 
			// 		  	 + "_DRGBT_data/test" + std::to_string(num_test) + ".log";
			// planner->outputPlannerData(project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) 
			// 						   + "_DRGBT_data/test" + std::to_string(num_test) + ".log");

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

			output_file.open(project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) + "_DRGBT_data/test.log", std::ofstream::app);
			output_file << "Test number: " << num_test << std::endl;
			output_file << "Number of successful tests: " << num_success_tests << " of " << num_test 
						<< " = " << 100.0 * num_success_tests / num_test << " %" << std::endl;
			output_file << "Success:\n" << result << std::endl;
			output_file << "Number of iterations:\n" << planner->getPlannerInfo()->getNumIterations() << std::endl;
			output_file << "Algorithm execution time [s]:\n" << planner->getPlannerInfo()->getPlanningTime() << std::endl;
			output_file << "Path length [rad]:\n" << (result ? path_length : INFINITY) << std::endl;
			output_file << "Task 1 interrupted:\n" << planner->getPlannerInfo()->getTask1Interrupted() << std::endl;

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
	}

	LOG(INFO) << "Success rate:                     " << 100.0 * num_success_tests / (num_test - 1) << " [%]";
	LOG(INFO) << "Average algorithm execution time: " << getMean(alg_times) << " +- " << getStd(alg_times) << " [s]";
	LOG(INFO) << "Average iteration execution time: " << getMean(iter_times) * 1e3 << " +- " << getStd(iter_times) * 1e3 << " [ms]";
	LOG(INFO) << "Average number of iterations:     " << getMean(num_iters) << " +- " << getStd(num_iters);
	LOG(INFO) << "Average path length:              " << getMean(path_lengths) << " +- " << getStd(path_lengths) << " [rad]";
	LOG(INFO) << "\n--------------------------------------------------------------------\n\n";

	google::ShutDownCommandLineFlags();
	return 0;
}
