#include <RGBTConnect.h>
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
	
	std::string scenario_file_path = "/data/xarm6/scenario_test/scenario_test.yaml";
	// std::string scenario_file_path = "/data/xarm6/scenario1/scenario1.yaml";
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
	int num_random_obstacles = node["num_random_obstacles"].as<int>();
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
		std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rbt::RGBTConnect>(ss, start, goal);
		result = planner->solve();
	}

	LOG(INFO) << "Using scenario: " << project_path + scenario_file_path;
	LOG(INFO) << "Environment parts: " << env->getParts().size();
	LOG(INFO) << "Number of DOFs: " << ss->getNumDimensions();
	LOG(INFO) << "State space type: " << ss->getStateSpaceType();
	LOG(INFO) << "Start: " << start;
	LOG(INFO) << "Goal: " << goal;

	int num_test = 0;
	std::vector<float> planning_times;
	while (num_test++ < max_num_tests)
	{
		try
		{
			LOG(INFO) << "Test number " << num_test << " of " << max_num_tests;
			std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rbt::RGBTConnect>(ss, start, goal);
			result = planner->solve();

			LOG(INFO) << "RGBTConnect planning finished with " << (result ? "SUCCESS!" : "FAILURE!");
			LOG(INFO) << "Number of states: " << planner->getPlannerInfo()->getNumStates();
			LOG(INFO) << "Number of iterations: " << planner->getPlannerInfo()->getNumIterations();
			LOG(INFO) << "Planning time: " << planner->getPlannerInfo()->getPlanningTime() << " [ms]";
				
			if (result)
			{
				// LOG(INFO) << "Found path: ";
				// std::vector<std::shared_ptr<base::State>> path = planner->getPath();
				// for (int i = 0; i < path.size(); i++)
				// 	std::cout << i << ": " << path.at(i)->getCoord().transpose() << std::endl;
				planning_times.emplace_back(planner->getPlannerInfo()->getPlanningTime());
			}

			LOG(INFO) << "Planner data is saved at: " << project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) 
						 + "_rgbtconnect_test" + std::to_string(num_test) + ".log";
			planner->outputPlannerData(project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) 
									   + "_rgbtconnect_test" + std::to_string(num_test) + ".log");
			LOG(INFO) << "\n--------------------------------------------------------------------\n\n";
		}
		catch (std::exception &e)
		{
			LOG(ERROR) << e.what();
			num_test--;
		}
	}
	LOG(INFO) << "Average planning time: " << getMean(planning_times) << " +- " << getStd(planning_times) << " [ms].";

	google::ShutDownCommandLineFlags();
	return 0;
}
