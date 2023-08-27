#include <AbstractPlanner.h>
#include <RGBMTStar.h>
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
	// std::string scenario_file_path = "/data/planar_2dof/scenario3/scenario3.yaml";
	// std::string scenario_file_path = "/data/planar_10dof/scenario_test/scenario_test.yaml";
	// std::string scenario_file_path = "/data/planar_10dof/scenario1/scenario1.yaml";
	// std::string scenario_file_path = "/data/planar_10dof/scenario1/scenario2.yaml";
	// std::string scenario_file_path = "/data/xarm6/scenario_test/scenario_test1.yaml";
	// std::string scenario_file_path = "/data/xarm6/scenario_test/scenario_test2.yaml";
	// std::string scenario_file_path = "/data/xarm6/scenario1/scenario1.yaml";
	// std::string scenario_file_path = "/data/xarm6/scenario2/scenario2.yaml";

	bool print_help = false;
	CommandLine args("Test RGBMTStar command line parser.");
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

	int max_num_tests = 30;
	int num_test = 0;
	int num_success = 0;
	std::vector<float> initial_costs;
	std::vector<float> final_costs;
	std::vector<float> initial_times;
	std::vector<float> final_times;
	std::vector<float> initial_num_states;
	std::vector<float> final_num_states;
	std::unique_ptr<planning::AbstractPlanner> planner;
	std::ofstream output_file;
	output_file.open(project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) + "_sometest.log", std::ofstream::out);
	
	if (ss->getNumDimensions() == 2)
		RGBMTStarConfig::MAX_PLANNING_TIME = 10e3;		// 10 sec
	else if (ss->getNumDimensions() == 6)
		RGBMTStarConfig::MAX_PLANNING_TIME = 120e3; 	// 2 min
	else
		RGBMTStarConfig::MAX_PLANNING_TIME = 60e3;		// 1 min
	
	while (num_test++ < max_num_tests)
	{
		try
		{
			LOG(INFO) << "Test number " << num_test << " of " << max_num_tests;
			planner = std::make_unique<planning::rbt_star::RGBMTStar>(ss, scenario.getStart(), scenario.getGoal());					
			bool res = planner->solve();

			LOG(INFO) << "RGBMT* planning finished with " << (res ? "SUCCESS!" : "FAILURE!");
			LOG(INFO) << "Number of states: " << planner->getPlannerInfo()->getNumStates();
			LOG(INFO) << "Planning time: " << planner->getPlannerInfo()->getPlanningTime() << " [ms]";
			
			if (res)
			{
				num_success++;
				LOG(INFO) << "Path cost: " << planner->getPlannerInfo()->getCostConvergence().back();
				final_costs.emplace_back(planner->getPlannerInfo()->getCostConvergence().back());
				final_times.emplace_back(planner->getPlannerInfo()->getPlanningTime());
				final_num_states.emplace_back(planner->getPlannerInfo()->getNumStates());
				for (int i = 0; i < planner->getPlannerInfo()->getNumStates(); i++)
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
						+ "_test" + std::to_string(num_test) + ".log";
			planner->outputPlannerData(project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) 
									   + "_test" + std::to_string(num_test) + ".log");

			// output_file << "Cost convergence: \n" 
            //             << "Cost [rad]\t\tNum. states\t\tTime [ms]" << std::endl;
			for (int i = 0; i < planner->getPlannerInfo()->getNumStates(); i++)
                output_file << planner->getPlannerInfo()->getCostConvergence()[i] << "\t\t"
							<< i+1 << "\t\t"
							<< planner->getPlannerInfo()->getStateTimes()[i] << std::endl;

			LOG(INFO) << "\n--------------------------------------------------------------------\n\n";
		}
		catch (std::domain_error &e)
		{
			LOG(ERROR) << e.what();
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
