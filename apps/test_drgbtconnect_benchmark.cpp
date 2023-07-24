#include <AbstractPlanner.h>
#include <DRGBTConnect.h>
#include <iostream>
#include <Scenario.h>
#include <ConfigurationReader.h>
#include <CommandLine.h>
#include <glog/logging.h>

float getMean(std::vector<float> &v);

float getStd(std::vector<float> &v);

int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

	std::string scenario_file_path = "/data/planar_2dof/scenario_test/scenario_test.yaml";
	// std::string scenario_file_path = "/data/planar_2dof/scenario1/scenario1.yaml";
	// std::string scenario_file_path = "/data/planar_2dof/scenario2/scenario2.yaml";
	// std::string scenario_file_path = "/data/xarm6/scenario_test/scenario_test.yaml";
	// std::string scenario_file_path = "/data/xarm6/scenario1/scenario1.yaml";
	// std::string scenario_file_path = "/data/xarm6/scenario2/scenario2.yaml";

	bool print_help = false;
	CommandLine args("Test RGBTConnect command line parser.");
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

	int num_test = 0;
	int max_num_tests = 100;
	std::vector<std::string> routines = {"generateHorizon [us]", "updateHorizon [us]", "updateCurrentState [us]", "generateGBur [ms]", "RGBTConnect [ms]"};
	std::vector<std::vector<float>> routine_times(routines.size());
	std::ofstream output_file;
	output_file.open("../" + scenario_file_path.substr(0, scenario_file_path.size()-5) + "_routine_times.log", std::ofstream::out);

	while (num_test++ < max_num_tests)
	{
		LOG(INFO) << "Test number " << num_test << " of " << max_num_tests;
		scenario::Scenario scenario(scenario_file_path, project_path);
		std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();
		std::unique_ptr<planning::AbstractPlanner> planner;

		LOG(INFO) << "Using scenario: " << project_path + scenario_file_path;
		LOG(INFO) << "Environment parts: " << scenario.getEnvironment()->getParts().size();
		LOG(INFO) << "Dimensions: " << ss->getDimensions();
		LOG(INFO) << "State space type: " << ss->getStateSpaceType();
		LOG(INFO) << "Start: " << scenario.getStart();
		LOG(INFO) << "Goal: " << scenario.getGoal();

		try
		{
			planner = std::make_unique<planning::drbt::DRGBTConnect>(ss, scenario.getStart(), scenario.getGoal());
			bool res = planner->solve();
			LOG(INFO) << "DRGBTConnect planning finished with " << (res ? "SUCCESS!" : "FAILURE!");
			LOG(INFO) << "Number of iterations: " << planner->getPlannerInfo()->getNumIterations();
			LOG(INFO) << "Planning time: " << planner->getPlannerInfo()->getPlanningTime() << " [ms]";
			
			// if (res)
			// {
			// 	LOG(INFO) << "Path: ";
			// 	std::vector<std::shared_ptr<base::State>> path = planner->getPath();
			// 	for (int i = 0; i < path.size(); i++)
			// 		std::cout << i << ": " << path.at(i)->getCoord().transpose() << std::endl;
			// }

			LOG(INFO) << "Planner data is saved at: " << project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) 
						+ "_test" + std::to_string(num_test) + ".log";
			planner->outputPlannerData(project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) 
									   + "_test" + std::to_string(num_test) + ".log");

			std::vector<std::vector<float>> times = planner->getPlannerInfo()->getRoutineTimes();
			for (int idx = 0; idx < times.size(); idx++)
				for (int i = 0; i < times[idx].size(); i++)
					routine_times[idx].emplace_back(times[idx][i]);
			
			LOG(INFO) << "\n--------------------------------------------------------------------\n\n";
		}
		catch (std::domain_error &e)
		{
			LOG(ERROR) << e.what();
		}
	}

	for (int idx = 0; idx < routines.size(); idx++)
	{
		LOG(INFO) << "Average time of the routine " << routines[idx] << ": " << getMean(routine_times[idx]) << " +- " << getStd(routine_times[idx]) << "\t"
				  << "Maximal time: " << *std::max_element(routine_times[idx].begin(), routine_times[idx].end()) << "\t"
				  << "Size " << routine_times[idx].size();
		output_file << "Routine " << idx << ": " << routines[idx] << std::endl;
		for (float t : routine_times[idx])
			output_file << t << std::endl;
		output_file << "-------------------------------------------------------------\n\n";
	}
	
	output_file.close();
	google::ShutDownCommandLineFlags();
	return 0;
}

float getMean(std::vector<float> &v)
{
	float sum = std::accumulate(v.begin(), v.end(), 0.0);
	return sum / v.size();
}

float getStd(std::vector<float> &v)
{
	float mean = getMean(v);
	float sum = 0;
	for (int i = 0; i < v.size(); i++)
		sum += (v[i] - mean) * (v[i] - mean);

	return std::sqrt(sum / v.size());
}
