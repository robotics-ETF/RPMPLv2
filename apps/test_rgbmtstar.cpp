#include <RGBMTStar.h>
#include <ConfigurationReader.h>
#include <CommonFunctions.h>

int main(int argc, char **argv)
{
	// std::string scenario_file_path = "/data/planar_2dof/scenario_test/scenario_test.yaml";
	// std::string scenario_file_path = "/data/planar_2dof/scenario1/scenario1.yaml";
	// std::string scenario_file_path = "/data/planar_2dof/scenario2/scenario2.yaml";
	// std::string scenario_file_path = "/data/planar_2dof/scenario3/scenario3.yaml";

	// std::string scenario_file_path = "/data/planar_10dof/scenario_test/scenario_test.yaml";
	// std::string scenario_file_path = "/data/planar_10dof/scenario1/scenario1.yaml";
	// std::string scenario_file_path = "/data/planar_10dof/scenario2/scenario2.yaml";

	// std::string scenario_file_path = "/data/xarm6/scenario_test/scenario_test.yaml";
	std::string scenario_file_path = "/data/xarm6/scenario1/scenario1.yaml";
	// std::string scenario_file_path = "/data/xarm6/scenario2/scenario2.yaml";
	// std::string scenario_file_path = "/data/xarm6/scenario3/scenario3.yaml";

	int max_num_tests = 30;
	int num_random_obstacles = 10;  	// If set to zero, random obstacles will not be initialized
	bool use_recommended_planning_times = false;
	
	// -------------------------------------------------------------------------------------- //

	initGoogleLogging(argv);
	int clp = commandLineParser(argc, argv, scenario_file_path);
	if (clp != 0) return clp;

	const std::string project_path = getProjectPath();
	ConfigurationReader::initConfiguration(project_path);
	scenario::Scenario scenario(scenario_file_path, project_path);
	std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();
	std::shared_ptr<base::State> start = scenario.getStart();
	std::shared_ptr<base::State> goal = scenario.getGoal();
	std::shared_ptr<env::Environment> env = scenario.getEnvironment();

	bool result = false;
	int num_obs = env->getParts().size();
	while (!result && num_random_obstacles > 0)
	{
		LOG(INFO) << "Adding " << num_random_obstacles << " random obstacles...";
		env->removeCollisionObjects(num_obs);
		initRandomObstacles(scenario, num_random_obstacles);
		std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rbt_star::RGBMTStar>(ss, start, goal);
		RGBMTStarConfig::TERMINATE_WHEN_PATH_IS_FOUND = true;
		result = planner->solve();
		RGBMTStarConfig::TERMINATE_WHEN_PATH_IS_FOUND = false;
	}

	LOG(INFO) << "Using scenario: " << project_path + scenario_file_path;
	LOG(INFO) << "Environment parts: " << env->getParts().size();
	LOG(INFO) << "Number of DOFs: " << ss->getNumDimensions();
	LOG(INFO) << "State space type: " << ss->getStateSpaceType();
	LOG(INFO) << "Start: " << start;
	LOG(INFO) << "Goal: " << goal;

	std::vector<float> initial_costs;
	std::vector<float> final_costs;
	std::vector<float> initial_times;
	std::vector<float> final_times;
	std::vector<float> initial_num_states;
	std::vector<float> final_num_states;
	std::ofstream output_file;
	output_file.open(project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) + "_rgbmtstar.log", std::ofstream::out);
	
	if (use_recommended_planning_times)
	{
		if (ss->getNumDimensions() == 2)
			RGBMTStarConfig::MAX_PLANNING_TIME = 10e3;		// 10 sec
		else if (ss->getNumDimensions() == 6)
			RGBMTStarConfig::MAX_PLANNING_TIME = 120e3; 	// 2 min
		else
			RGBMTStarConfig::MAX_PLANNING_TIME = 60e3;		// 1 min
	}
	
	int num_test = 0;
	int num_success = 0;
	while (num_test++ < max_num_tests)
	{
		try
		{
			LOG(INFO) << "Test number " << num_test << " of " << max_num_tests;
			std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rbt_star::RGBMTStar>(ss, start, goal);					
			result = planner->solve();

			LOG(INFO) << "RGBMT* planning finished with " << (result ? "SUCCESS!" : "FAILURE!");
			LOG(INFO) << "Number of states: " << planner->getPlannerInfo()->getNumStates();
			LOG(INFO) << "Planning time: " << planner->getPlannerInfo()->getPlanningTime() << " [ms]";
			
			if (result)
			{
				num_success++;
				LOG(INFO) << "Path cost: " << planner->getPlannerInfo()->getCostConvergence().back();
				final_costs.emplace_back(planner->getPlannerInfo()->getCostConvergence().back());
				final_times.emplace_back(planner->getPlannerInfo()->getPlanningTime());
				final_num_states.emplace_back(planner->getPlannerInfo()->getNumStates());
				for (size_t i = 0; i < planner->getPlannerInfo()->getNumStates(); i++)
				{
					if (planner->getPlannerInfo()->getCostConvergence()[i] < INFINITY)
					{
						initial_costs.emplace_back(planner->getPlannerInfo()->getCostConvergence()[i]);
						initial_times.emplace_back(planner->getPlannerInfo()->getStateTimes()[i]);
						initial_num_states.emplace_back(i);
						LOG(INFO) << "Path is found after " << i << " states (after " << initial_times.back() << " [ms])";
						break;
					}
				}		
			}

			LOG(INFO) << "Planner data is saved at: " << project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) 
						 + "_rgbmtstar_test" + std::to_string(num_test) + ".log";
			planner->outputPlannerData(project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) 
									   + "_rgbmtstar_test" + std::to_string(num_test) + ".log");

			// output_file << "Cost convergence: \n" 
            //             << "Cost [rad]\t\tNum. states\t\tTime [ms]" << std::endl;
			for (int i = 0; i < planner->getPlannerInfo()->getNumStates(); i++)
                output_file << planner->getPlannerInfo()->getCostConvergence()[i] << "\t\t"
							<< i+1 << "\t\t"
							<< planner->getPlannerInfo()->getStateTimes()[i] << std::endl;

			LOG(INFO) << "\n--------------------------------------------------------------------\n\n";
		}
		catch (std::exception &e)
		{
			LOG(ERROR) << e.what();
			num_test--;
		}
	}

	output_file << std::string(75, '-') << std::endl;
	output_file << "Space Type:      " << ss->getStateSpaceType() << std::endl;
	output_file << "Dimensionality:  " << ss->getNumDimensions() << std::endl;
	output_file << "Planner type:    " << "RGBMT*" << std::endl;
	output_file << "Using scenario:  " << project_path + scenario_file_path << std::endl;
	output_file << "Planner info:\n";
	output_file << "\t Success rate [%]:                                    " << (float) num_success / max_num_tests * 100  << std::endl;
	output_file << "\t Average initial path cost [rad]:                     " << getMean(initial_costs) << " +- " << getStd(initial_costs) << std::endl;
	output_file << "\t Average final path cost [rad]:                       " << getMean(final_costs) << " +- " << getStd(final_costs) << std::endl;
	output_file << "\t Average time when the first path is found [s]:       " << getMean(initial_times) / 1000 << " +- " << getStd(initial_times) / 1000 << std::endl;
	output_file << "\t Average planning time [s]:                           " << getMean(final_times) / 1000 << " +- " << getStd(final_times) / 1000 << std::endl;
	output_file << "\t Average num. of states when the first path is found: " << getMean(initial_num_states) << " +- " << getStd(initial_num_states) << std::endl;
	output_file << "\t Average num. of states:                              " << getMean(final_num_states) << " +- " << getStd(final_num_states) << std::endl;
	output_file << std::string(75, '-') << std::endl;		
	output_file.close();
	
	google::ShutDownCommandLineFlags();
	return 0;
}
