#include <AbstractPlanner.h>
#include <RRTConnect.h>
#include <iostream>
#include <Scenario.h>
#include <ConfigurationReader.h>
#include <CommandLine.h>
#include <glog/logging.h>


int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

	std::string scenario_file_path = "/data/planar_2dof/scenario_test/scenario_test.yaml";
	// std::string scenario_file_path = "/data/planar_2dof/scenario1/scenario1.yaml";
	// std::string scenario_file_path = "/data/planar_2dof/scenario2/scenario2.yaml";
	// std::string scenario_file_path = "/data/planar_10dof/scenario_test/scenario_test.yaml";
	// std::string scenario_file_path = "/data/planar_10dof/scenario1/scenario1.yaml";
	// std::string scenario_file_path = "/data/xarm6/scenario_test/scenario_test.yaml";
	// std::string scenario_file_path = "/data/xarm6/scenario1/scenario1.yaml";
	// std::string scenario_file_path = "/data/xarm6/scenario2/scenario2.yaml";

	bool print_help = false;
	CommandLine args("Test RRTConnect command line parser.");
	args.addArgument({"-s", "--scenario"}, &scenario_file_path, "Scenario .yaml description file path");
	args.addArgument({"-h", "--help"},     &print_help, "Use --scenario scenario_yaml_file_path to run with different scenario");

	try
	{
		args.parse(argc, argv);
	}
	catch (std::runtime_error const &e)
	{
		LOG(INFO) << e.what() << std::endl;
		return -1;
	}

	// When oPrintHelp was set to true, we print a help message and exit.
	if (print_help)
	{
		args.printHelp();
		return 0;
	}

	std::string project_path(__FILE__);
    for (int i = 0; i < 2; i++)
        project_path = project_path.substr(0, project_path.find_last_of("/\\"));

	ConfigurationReader::initConfiguration(project_path);
	scenario::Scenario scenario(scenario_file_path, project_path);
	std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();

	LOG(INFO) << "Using scenario: " << project_path + scenario_file_path;
	LOG(INFO) << "Environment parts: " << scenario.getEnvironment()->getParts().size();
	LOG(INFO) << "Number of DOFs: " << ss->getNumDimensions();
	LOG(INFO) << "State space type: " << ss->getStateSpaceType();
	LOG(INFO) << "Start: " << scenario.getStart();
	LOG(INFO) << "Goal: " << scenario.getGoal();
	
	try
	{
		std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rrt::RRTConnect>(ss, scenario.getStart(), scenario.getGoal());
		bool res = planner->solve();
		LOG(INFO) << "RRTConnect planning finished with " << (res ? "SUCCESS!" : "FAILURE!");
		LOG(INFO) << "Number of states: " << planner->getPlannerInfo()->getNumStates();
		LOG(INFO) << "Number of iterations: " << planner->getPlannerInfo()->getNumIterations();
		LOG(INFO) << "Planning time: " << planner->getPlannerInfo()->getPlanningTime() << " [ms]";
			
		if (res)
		{
			LOG(INFO) << "Found path: ";
			std::vector<std::shared_ptr<base::State>> path = planner->getPath();
			for (int i = 0; i < path.size(); i++)
				std::cout << i << ": " << path.at(i)->getCoord().transpose() << std::endl;
		}
		LOG(INFO) << "Planner data is saved at: " << project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) + "_planner_data.log";
		planner->outputPlannerData(project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) + "_planner_data.log");
		
	}
	catch (std::exception &e)
	{
		LOG(ERROR) << e.what();
	}
	google::ShutDownCommandLineFlags();
	return 0;
}
