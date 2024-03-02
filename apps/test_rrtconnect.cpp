#include <RRTConnect.h>
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

	int max_num_tests = 10;

	// -------------------------------------------------------------------------------------- //
	
	initGoogleLogging(argv);
	int clp = commandLineParser(argc, argv, scenario_file_path);
	if (clp != 0) return clp;

	const std::string project_path = getProjectPath();
	ConfigurationReader::initConfiguration(project_path);
    YAML::Node node = YAML::LoadFile(project_path + scenario_file_path);
	int num_random_obstacles = node["random_obstacles"]["num"].as<int>();
	Eigen::Vector3f obs_dim;
	for (int i = 0; i < 3; i++)
		obs_dim(i) = node["random_obstacles"]["dim"][i].as<float>();
		
	scenario::Scenario scenario(scenario_file_path, project_path);
	std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();
	std::shared_ptr<base::State> q_start = scenario.getStart();
	std::shared_ptr<base::State> q_goal = scenario.getGoal();
	std::shared_ptr<env::Environment> env = scenario.getEnvironment();

	bool result = false;
	int num_obs = env->getNumObjects();
	while (!result && num_random_obstacles > 0)
	{
		env->removeObjects(num_obs);
		initRandomObstacles(num_random_obstacles, obs_dim, scenario);
		std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rrt::RRTConnect>(ss, q_start, q_goal);
		result = planner->solve();
		LOG(INFO) << "A path to the goal can " << (result ? "" : "not ") << "be found!";
	}

	LOG(INFO) << "Using scenario: " << project_path + scenario_file_path;
	LOG(INFO) << "Environment parts: " << env->getNumObjects();
	LOG(INFO) << "Number of DOFs: " << ss->getNumDimensions();
	LOG(INFO) << "State space type: " << ss->getStateSpaceType();
	LOG(INFO) << "Start: " << q_start;
	LOG(INFO) << "Goal: " << q_goal;

	int num_test = 0;
	int num_success = 0;
	std::vector<int> planning_times;
	while (num_test++ < max_num_tests)
	{
		try
		{
			LOG(INFO) << "Test number " << num_test << " of " << max_num_tests;
			std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rrt::RRTConnect>(ss, q_start, q_goal);
			result = planner->solve();

			LOG(INFO) << "RRTConnect planning finished with " << (result ? "SUCCESS!" : "FAILURE!");
			LOG(INFO) << "Number of states: " << planner->getPlannerInfo()->getNumStates();
			LOG(INFO) << "Number of iterations: " << planner->getPlannerInfo()->getNumIterations();
			LOG(INFO) << "Planning time: " << planner->getPlannerInfo()->getPlanningTime() << " [ms]";
				
			if (result)
			{
				num_success++;
				planning_times.emplace_back(planner->getPlannerInfo()->getPlanningTime());
				// LOG(INFO) << "Found path: ";
				// std::vector<std::shared_ptr<base::State>> path = planner->getPath();
				// for (int i = 0; i < path.size(); i++)
				// 	std::cout << i << ": " << path.at(i)->getCoord().transpose() << std::endl;
			}

			LOG(INFO) << "Planner data is saved at: " << project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) 
						 + "_rrtconnect_test" + std::to_string(num_test) + ".log";
			planner->outputPlannerData(project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) 
									   + "_rrtconnect_test" + std::to_string(num_test) + ".log");
			LOG(INFO) << "\n--------------------------------------------------------------------\n\n";
		}
		catch (std::exception &e)
		{
			LOG(ERROR) << e.what();
			num_test--;
		}
	}
	LOG(INFO) << "Success rate: " << (float) num_success / max_num_tests * 100 << " [%]";
	LOG(INFO) << "Average planning time: " << getMean(planning_times) << " +- " << getStd(planning_times) << " [ms]";

	google::ShutDownCommandLineFlags();
	return 0;
}
