#include "RRTx.h"
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
	const std::string directory_path { project_path + scenario_file_path.substr(0, scenario_file_path.find_last_of("/\\")) + "/RRTx_data" };
	std::filesystem::create_directory(directory_path);
	ConfigurationReader::initConfiguration(project_path);
    YAML::Node node { YAML::LoadFile(project_path + scenario_file_path) };
	const float max_vel_obs { node["testing"]["max_vel_obs"].as<float>() };
	const size_t max_num_tests { node["testing"]["max_num"].as<size_t>() };

	std::ofstream output_file {};
	output_file.open(directory_path + "/results.log", std::ofstream::out);
	output_file << "Using scenario:                                         " << scenario_file_path << std::endl;
	output_file << "Dynamic planner:                                        " << planning::PlannerType::RRTx << std::endl;
	output_file << "Maximal algorithm time [s]:                             " << RRTxConfig::MAX_PLANNING_TIME << std::endl;
	output_file	<< "Maximal iteration time [s]:                             " << RRTxConfig::MAX_ITER_TIME << std::endl;
	output_file << "Step size for extending [rad]:                          " << RRTxConfig::EPS_STEP << std::endl;
	output_file << "Radius for rewiring [rad]:                              " << RRTxConfig::R_REWIRE << std::endl;
	output_file << "Maximal number of neighbors to consider:                " << RRTxConfig::MAX_NEIGHBORS << std::endl;
	output_file << "Process obstacles every N iterations:                   " << RRTxConfig::REPLANNING_THROTTLE << std::endl;
	output_file << "Probability of sampling start directly:                 " << RRTxConfig::START_BIAS << std::endl;
	output_file << "Resolution for collision checking [m]:                  " << RRTxConfig::RESOLUTION_COLL_CHECK << std::endl;
	output_file << "Trajectory interpolation:                               " << RRTxConfig::TRAJECTORY_INTERPOLATION << std::endl;
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
			
			planner = std::make_unique<planning::rrtx::RRTx>(ss, scenario.getStart(), scenario.getGoal());
			bool result { planner->solve() };
			
			LOG(INFO) << planner->getPlannerType() << " planning finished with " << (result ? "SUCCESS!" : "FAILURE!");
			LOG(INFO) << "Number of iterations: " << planner->getPlannerInfo()->getNumIterations();
			LOG(INFO) << "Algorithm time:       " << planner->getPlannerInfo()->getPlanningTime() << " [s]";
			// LOG(INFO) << "Planner data is saved at: " << directory_path + "/test" + std::to_string(num_test) + ".log";
			// planner->outputPlannerData(directory_path + "/test" + std::to_string(num_test) + ".log");

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

			output_file.open(directory_path + "/results.log", std::ofstream::app);
			output_file << "Test number: " << num_test << std::endl;
			output_file << "Number of successful tests: " << num_success_tests << " of " << num_test 
						<< " = " << 100.0 * num_success_tests / num_test << " %" << std::endl;
			output_file << "Success:\n" << result << std::endl;
			output_file << "Number of iterations:\n" << planner->getPlannerInfo()->getNumIterations() << std::endl;
			output_file << "Algorithm execution time [s]:\n" << planner->getPlannerInfo()->getPlanningTime() << std::endl;
			output_file << "Time for the initial path [s]:\n" << (result ? planner->getPlannerInfo()->getIterationTimes().front() : INFINITY) << std::endl;
			output_file << "Path length [rad]:\n" << (result ? path_length : INFINITY) << std::endl;

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
