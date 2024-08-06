#include "RGBMTStar.h"
#include "ConfigurationReader.h"
#include "CommonFunctions.h"

int main(int argc, char **argv)
{
	std::string scenario_file_path
	{
		// "/data/planar_2dof/scenario_test/scenario_test.yaml"
		// "/data/planar_2dof/scenario1/scenario1.yaml"
		// "/data/planar_2dof/scenario2/scenario2.yaml"
		// "/data/planar_2dof/scenario3/scenario3.yaml"

		// "/data/planar_10dof/scenario_test/scenario_test.yaml"
		// "/data/planar_10dof/scenario1/scenario1.yaml"
		// "/data/planar_10dof/scenario2/scenario2.yaml"

		"/data/xarm6/scenario_test/scenario_test.yaml"
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

	scenario::Scenario scenario(scenario_file_path, project_path);
	std::shared_ptr<base::StateSpace> ss { scenario.getStateSpace() };
	std::shared_ptr<base::State> q_start { scenario.getStart() };
	std::shared_ptr<base::State> q_goal { scenario.getGoal() };
	std::shared_ptr<env::Environment> env { scenario.getEnvironment() };
	std::unique_ptr<planning::AbstractPlanner> planner { nullptr };

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

	LOG(INFO) << "Using scenario: " << project_path + scenario_file_path;
	LOG(INFO) << "Environment parts: " << env->getNumObjects();
	LOG(INFO) << "Number of DOFs: " << ss->num_dimensions;
	LOG(INFO) << "State space type: " << ss->getStateSpaceType();
	LOG(INFO) << "Start: " << q_start;
	LOG(INFO) << "Goal: " << q_goal;

	std::vector<float> initial_costs {};
	std::vector<float> final_costs {};
	std::vector<float> initial_times {};
	std::vector<float> final_times {};
	std::vector<float> initial_num_states {};
	std::vector<float> final_num_states {};
	std::ofstream output_file {};
	output_file.open(project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) + "_rgbmtstar.log", std::ofstream::out);
	
	size_t num_test { 0 };
	size_t num_success { 0 };
	while (num_test++ < max_num_tests)
	{
		try
		{
			LOG(INFO) << "Test number " << num_test << " of " << max_num_tests;
			planner = std::make_unique<planning::rbt_star::RGBMTStar>(ss, q_start, q_goal);					
			result = planner->solve();

			LOG(INFO) << planner->getPlannerType() << " planning finished with " << (result ? "SUCCESS!" : "FAILURE!");
			LOG(INFO) << "Number of states: " << planner->getPlannerInfo()->getNumStates();
			LOG(INFO) << "Planning time: " << planner->getPlannerInfo()->getPlanningTime() << " [s]";
			
			if (result)
			{
				num_success++;
				LOG(INFO) << "Path cost: " << planner->getPlannerInfo()->getOptimalCost();
				final_costs.emplace_back(planner->getPlannerInfo()->getOptimalCost());
				final_times.emplace_back(planner->getPlannerInfo()->getPlanningTime());
				final_num_states.emplace_back(planner->getPlannerInfo()->getNumStates());
				for (size_t i = 0; i < planner->getPlannerInfo()->getNumStates(); i++)
				{
					if (planner->getPlannerInfo()->getCostConvergence()[i] < INFINITY)
					{
						initial_costs.emplace_back(planner->getPlannerInfo()->getCostConvergence()[i]);
						initial_times.emplace_back(planner->getPlannerInfo()->getStateTimes()[i]);
						initial_num_states.emplace_back(i);
						LOG(INFO) << "Path is found after " << i << " states (after " << initial_times.back() << " [s])";
						break;
					}
				}		
			}

			LOG(INFO) << "Planner data is saved at: " << project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) 
						 + "_rgbmtstar_test" + std::to_string(num_test) + ".log";
			planner->outputPlannerData(project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) 
									   + "_rgbmtstar_test" + std::to_string(num_test) + ".log");

			// output_file << "Cost convergence: \n" 
            //             << "Cost [rad]\t\tNum. states\t\tTime [s]" << std::endl;
			for (size_t i = 0; i < planner->getPlannerInfo()->getNumStates(); i++)
                output_file << planner->getPlannerInfo()->getCostConvergence()[i] << "\t\t"
							<< i+1 << "\t\t"
							<< planner->getPlannerInfo()->getStateTimes()[i] << std::endl;

			LOG(INFO) << "\n--------------------------------------------------------------------\n\n";
		}
		catch (std::exception &e)
		{
			LOG(ERROR) << e.what();
		}
	}

	output_file << std::string(75, '-') << std::endl;
	output_file << "Space Type:      " << ss->getStateSpaceType() << std::endl;
	output_file << "Dimensionality:  " << ss->num_dimensions << std::endl;
	output_file << "Planner type:    " << planner->getPlannerType() << std::endl;
	output_file << "Using scenario:  " << project_path + scenario_file_path << std::endl;
	output_file << "Planner info:\n";
	output_file << "\t Success rate [%]:                                    " << (float) num_success / max_num_tests * 100  << std::endl;
	output_file << "\t Average initial path cost [rad]:                     " << getMean(initial_costs) << " +- " << getStd(initial_costs) << std::endl;
	output_file << "\t Average final path cost [rad]:                       " << getMean(final_costs) << " +- " << getStd(final_costs) << std::endl;
	output_file << "\t Average time when the first path is found [s]:       " << getMean(initial_times) << " +- " << getStd(initial_times) << std::endl;
	output_file << "\t Average planning time [s]:                           " << getMean(final_times) << " +- " << getStd(final_times) << std::endl;
	output_file << "\t Average num. of states when the first path is found: " << getMean(initial_num_states) << " +- " << getStd(initial_num_states) << std::endl;
	output_file << "\t Average num. of states:                              " << getMean(final_num_states) << " +- " << getStd(final_num_states) << std::endl;
	output_file << std::string(75, '-') << std::endl;		
	output_file.close();
	
	google::ShutDownCommandLineFlags();
	return 0;
}
