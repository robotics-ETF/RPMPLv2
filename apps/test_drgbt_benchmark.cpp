#include <AbstractPlanner.h>
#include <DRGBT.h>
#include <iostream>
#include <Scenario.h>
#include <ConfigurationReader.h>
#include <CommandLine.h>
#include <glog/logging.h>

float getMean(std::vector<float> &v);

float getStd(std::vector<float> &v);

void initObstacles(scenario::Scenario &scenario, float max_obs_vel, int num_obstacles);

int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

	// std::string scenario_file_path = "/data/planar_2dof/scenario_test/scenario_test.yaml";
	// std::string scenario_file_path = "/data/planar_2dof/scenario1/scenario1.yaml";
	// std::string scenario_file_path = "/data/planar_2dof/scenario2/scenario2.yaml";
	std::string scenario_file_path = "/data/xarm6/scenario_real_time/scenario_real_time.yaml";
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
    YAML::Node node = YAML::LoadFile(project_path + scenario_file_path);
	int num_obstacles_init = node["testing"]["num_obstacles_init"].as<int>();
	const int max_num_obstacles = node["testing"]["max_num_obstacles"].as<int>();
	const float max_obs_vel = node["testing"]["max_obs_vel"].as<float>();
	int num_test_init = node["testing"]["num_test_init"].as<int>();
	int num_success_test_init = node["testing"]["num_success_test_init"].as<int>();
	const int max_num_tests = node["testing"]["max_num_tests"].as<int>();
	std::string static_planner_name = node["testing"]["static_planner_name"].as<std::string>();
	std::vector<std::string> routines;
    for (int i = 0; i < node["testing"]["routines"].size(); i++)
        routines.emplace_back(node["testing"]["routines"][i].as<std::string>());
	DRGBTConfig::MAX_NUM_VALIDITY_CHECKS = 100 * max_obs_vel;

	while (num_obstacles_init <= max_num_obstacles)
	{
		LOG(INFO) << "Number of obstacles " << num_obstacles_init << " of " << max_num_obstacles;
		
		std::ofstream output_file;
		if (num_test_init == 1)
		{
			output_file.open(project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) + 
							"_routine_times" + std::to_string(num_obstacles_init) + ".log", std::ofstream::out);
			output_file << "Dynamic planner: DRGBT" << std::endl;
			output_file << "Static planner for replanning: " << static_planner_name << std::endl;
			output_file << "Number of obstacles: " << num_obstacles_init << std::endl;
			output_file << "Maximal velocity of each obstacle: " << max_obs_vel << " [m/iter.] " << std::endl;
			output_file << "--------------------------------------------------------------------\n";
			output_file.close();
		}
		std::vector<float> alg_times;
		std::vector<float> path_lengths;
		int num_test = num_test_init;
		int num_success_tests = num_success_test_init;
		while (num_success_tests < max_num_tests)
		{
			output_file.open(project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) + 
							"_routine_times" + std::to_string(num_obstacles_init) + ".log", std::ofstream::app);

			scenario::Scenario scenario(scenario_file_path, project_path);
			std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();
			std::shared_ptr<env::Environment> env = scenario.getEnvironment();
			initObstacles(scenario, max_obs_vel, num_obstacles_init);

			LOG(INFO) << "Test number: " << num_test << " (" << num_success_tests << ")";
			LOG(INFO) << "Using scenario: " << project_path + scenario_file_path;
			LOG(INFO) << "Environment parts: " << env->getParts().size();
			LOG(INFO) << "Number of DOFs: " << ss->getNumDimensions();
			LOG(INFO) << "State space type: " << ss->getStateSpaceType();
			LOG(INFO) << "Start: " << scenario.getStart();
			LOG(INFO) << "Goal: " << scenario.getGoal();

			try
			{
				std::unique_ptr<planning::AbstractPlanner> planner = 
					std::make_unique<planning::drbt::DRGBT>(ss, scenario.getStart(), scenario.getGoal(), static_planner_name);
				bool res = planner->solve();
				
				LOG(INFO) << "DRGBT planning finished with " << (res ? "SUCCESS!" : "FAILURE!");
				LOG(INFO) << "Number of iterations: " << planner->getPlannerInfo()->getNumIterations();
				LOG(INFO) << "Algorithm time: " << planner->getPlannerInfo()->getPlanningTime() << " [ms]";
				// LOG(INFO) << "Planner data is saved at: " << project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) 
				// 			+ "_test" + std::to_string(num_test) + ".log";
				// planner->outputPlannerData(project_path + scenario_file_path.substr(0, scenario_file_path.size()-5) 
				// 						   + "_test" + std::to_string(num_test) + ".log");

				float path_length = 0;
				if (res)
				{
					// LOG(INFO) << "Found path: ";
					std::vector<std::shared_ptr<base::State>> path = planner->getPath();
					for (int i = 0; i < path.size(); i++)
					{
						// std::cout << i << ": " << path.at(i)->getCoord().transpose() << std::endl;
						if (i > 0) path_length += ss->getNorm(path.at(i-1), path.at(i));
					}
					path_lengths.emplace_back(path_length);
					alg_times.emplace_back(planner->getPlannerInfo()->getPlanningTime());
					num_success_tests++;
				}

				output_file << "Test number: " << num_test << std::endl;
				output_file << "Number of successful tests: " << num_success_tests << " of " << num_test << std::endl;
				output_file << "Success:\n" << res << std::endl;
				output_file << "Algorithm time [ms]:\n" << planner->getPlannerInfo()->getPlanningTime() << std::endl;
				output_file << "Path length [rad]:\n" << (res ? path_length : INFINITY) << std::endl;

				if (res)
				{
					std::vector<std::vector<float>> routine_times = planner->getPlannerInfo()->getRoutineTimes();
					for (int idx = 0; idx < routines.size(); idx++)
					{
						// LOG(INFO) << "Routine " << routines[idx];
						// LOG(INFO) << "\tAverage time: " << getMean(routine_times[idx]) << " +- " << getStd(routine_times[idx]);
						// LOG(INFO) << "\tMaximal time: " << *std::max_element(routine_times[idx].begin(), routine_times[idx].end());
						// LOG(INFO) << "\tData size: " << routine_times[idx].size(); 
						
						output_file << "Routine " << routines[idx] << " routine_times: " << std::endl;
						for (float t : routine_times[idx])
							output_file << t << std::endl;
					}
				}
			}
			catch (std::exception &e)
			{
				LOG(ERROR) << e.what();
			}

			LOG(INFO) << "Number of successful tests: " << num_success_tests << " of " << num_test 
					  << " = " << 100.0 * num_success_tests / num_test << " %";
			LOG(INFO) << "\n--------------------------------------------------------------------\n\n";
			output_file << "--------------------------------------------------------------------\n";
			output_file.close();
			num_test++;
		}

		num_test_init = 1;
		num_success_test_init = 0;
		num_obstacles_init += std::pow(10, std::floor(std::log10(num_obstacles_init)));

		LOG(INFO) << "Success rate: " << 100.0 * num_success_tests / (num_test - 1) << " %";
		LOG(INFO) << "Average algorithm time: " << getMean(alg_times) << " +- " << getStd(alg_times) << " [ms]";
		LOG(INFO) << "Average path length: " << getMean(path_lengths) << " +- " << getStd(path_lengths) << " [rad]";
		LOG(INFO) << "\n--------------------------------------------------------------------\n\n";
	}

	google::ShutDownCommandLineFlags();
	return 0;
}

float getMean(std::vector<float> &v)
{
	if (v.empty()) 
		return INFINITY;
	
	float sum = std::accumulate(v.begin(), v.end(), 0.0);
	return sum / v.size();
}

float getStd(std::vector<float> &v)
{
	if (v.empty()) 
		return INFINITY;
	
	float mean = getMean(v);
	float sum = 0;
	for (size_t i = 0; i < v.size(); i++)
		sum += (v[i] - mean) * (v[i] - mean);

	return std::sqrt(sum / v.size());
}

void initObstacles(scenario::Scenario &scenario, float max_obs_vel, int num_obstacles)
{
	const Eigen::Vector3f dim = Eigen::Vector3f(0.01, 0.01, 0.01);
	const Eigen::Vector3f WS_center = Eigen::Vector3f(0, 0, 0.267);
	const float WS_radius = 1.0;
	std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();
	std::shared_ptr<env::Environment> env = scenario.getEnvironment();

	env->setMaxVel(max_obs_vel);
	Eigen::Vector3f pos;
	Eigen::Matrix3f rot = fcl::Quaternionf(0, 0, 0, 0).matrix();
	float r, fi, theta;
	for (int i = 0; i < num_obstacles; i++)
	{
		r = float(rand()) / RAND_MAX * WS_radius;
		fi = float(rand()) / RAND_MAX * 2 * M_PI;
		theta = float(rand()) / RAND_MAX * M_PI;
		pos.x() = WS_center.x() + r * std::cos(fi) * std::sin(theta);
		pos.y() = WS_center.y() + r * std::sin(fi) * std::sin(theta);
		pos.z() = WS_center.z() + r * std::cos(theta);
		std::shared_ptr<fcl::CollisionGeometryf> box = std::make_shared<fcl::Boxf>(dim);
		std::shared_ptr<fcl::CollisionObjectf> ob = std::make_shared<fcl::CollisionObjectf>(box, rot, pos);
		env->addCollisionObject(ob);
		if (ss->computeDistance(scenario.getStart(), true) < 6 * DRGBTConfig::D_CRIT) // Just to ensure safety of init. conf.
		{
			env->removeCollisionObjects(i);
			i--;
		}
	}
}
