#include "RGBTConnect.h"
#include "ConfigurationReader.h"
#include "CommonFunctions.h"
#include "Splines.h"

int main(int argc, char **argv)
{
	std::string scenario_file_path
	{
		"/data/planar_2dof/scenario_test/scenario_test.yaml"
		// "/data/planar_2dof/scenario1/scenario1.yaml"
		// "/data/planar_2dof/scenario2/scenario2.yaml"
		// "/data/planar_2dof/scenario3/scenario3.yaml"

		// "/data/planar_10dof/scenario_test/scenario_test.yaml"
		// "/data/planar_10dof/scenario1/scenario1.yaml"
		// "/data/planar_10dof/scenario2/scenario2.yaml"

		// "/data/xarm6/scenario_test/scenario_test.yaml"
		// "/data/xarm6/scenario1/scenario1.yaml"
		// "/data/xarm6/scenario2/scenario2.yaml"
		// "/data/xarm6/scenario3/scenario3.yaml"
	};
	
	initGoogleLogging(argv);
	int clp = commandLineParser(argc, argv, scenario_file_path);
	if (clp != 0) return clp;

	const std::string project_path { getProjectPath() };
	const std::string directory_path { project_path + scenario_file_path.substr(0, scenario_file_path.find_last_of("/\\")) + "/RGBTConnect_data" };
	std::filesystem::create_directory(directory_path);
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
    std::shared_ptr<planning::trajectory::Trajectory> trajectory { nullptr };

	bool result { false };
	size_t num_obs { env->getNumObjects() };
	while (!result && num_random_obstacles > 0)
	{
		env->removeObjects(num_obs);
		initRandomObstacles(num_random_obstacles, obs_dim, scenario);
		planner = std::make_unique<planning::rbt::RGBTConnect>(ss, q_start, q_goal);
		result = planner->solve();
		LOG(INFO) << "A path to the goal can " << (result ? "" : "not ") << "be found!";
	}

	LOG(INFO) << "Using scenario: " << project_path + scenario_file_path;
	LOG(INFO) << "Environment parts: " << env->getNumObjects();
	LOG(INFO) << "Number of DOFs: " << ss->num_dimensions;
	LOG(INFO) << "State space type: " << ss->getStateSpaceType();
	LOG(INFO) << "Start: " << q_start;
	LOG(INFO) << "Goal: " << q_goal;

	size_t num_test { 0 };
	size_t num_success { 0 };
	std::vector<float> comp_times1 {}, comp_times2 {};
	std::vector<float> final_times1 {}, final_times2 {};
    const float delta_time { 0.005 };

	while (num_test++ < max_num_tests)
	{
		try
		{
            std::ofstream output_file {};
            output_file.open(directory_path + "/trajectory" + std::to_string(num_test) + ".log", std::ofstream::out);

			LOG(INFO) << "Test number " << num_test << " of " << max_num_tests;
			planner = std::make_unique<planning::rbt::RGBTConnect>(ss, q_start, q_goal);
			result = planner->solve();

			LOG(INFO) << planner->getPlannerType() << " planning finished with " << (result ? "SUCCESS!" : "FAILURE!");
			LOG(INFO) << "Number of states: " << planner->getPlannerInfo()->getNumStates();
			LOG(INFO) << "Number of iterations: " << planner->getPlannerInfo()->getNumIterations();
			LOG(INFO) << "Planning time: " << planner->getPlannerInfo()->getPlanningTime() << " [s]";
			
			if (result)
			{
				num_success++;
				// LOG(INFO) << "Found path: ";
				// std::vector<std::shared_ptr<base::State>> path { planner->getPath() };
				// for (size_t i = 0; i < path.size(); i++)
				// 	std::cout << i << ": " << path.at(i)->getCoord().transpose() << std::endl;

                std::vector<std::shared_ptr<base::State>> new_path { };
                ss->preprocessPath(planner->getPath(), new_path, 10*RRTConnectConfig::EPS_STEP);

                output_file << "Path: \n";
                for (size_t i = 0; i < new_path.size(); i++)
                    output_file << new_path.at(i)->getCoord().transpose() << "\n";
                output_file << "--------------------------------------------------------------------\n";

                trajectory = std::make_shared<planning::trajectory::Trajectory>(ss);
                auto time1 = std::chrono::steady_clock::now();
                trajectory->path2traj_v1(new_path);
				comp_times1.emplace_back(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time1).count() * 1e-3);
                final_times1.emplace_back(trajectory->composite_spline->getTimeFinal());

                output_file << "Trajectory 1 (position): \n";
                for (float t = 0; t < trajectory->composite_spline->getTimeFinal(); t += delta_time)
                    output_file << trajectory->composite_spline->getPosition(t).transpose() << "\n";
                output_file << "--------------------------------------------------------------------\n";
                
                auto time2 = std::chrono::steady_clock::now();
                trajectory->path2traj_v2(new_path);
				comp_times2.emplace_back(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time2).count() * 1e-3);
                final_times2.emplace_back(trajectory->composite_spline->getTimeFinal());

                output_file << "Trajectory 2 (position): \n";
                for (float t = 0; t < trajectory->composite_spline->getTimeFinal(); t += delta_time)
                    output_file << trajectory->composite_spline->getPosition(t).transpose() << "\n";
                output_file << "--------------------------------------------------------------------\n";
                output_file.close();
			}

			// LOG(INFO) << "Planner data is saved at: " << directory_path + "/test" + std::to_string(num_test) + ".log";
			// planner->outputPlannerData(directory_path + "/test" + std::to_string(num_test) + ".log");
			LOG(INFO) << "\n--------------------------------------------------------------------\n\n";
		}
		catch (std::exception &e)
		{
			LOG(ERROR) << e.what();
		}
	}
	LOG(INFO) << "Success rate: " << (float) num_success / max_num_tests * 100 << " [%]";
	LOG(INFO) << "Comp. times1: " << getMean(comp_times1) << " [ms]";
	LOG(INFO) << "Comp. times2: " << getMean(comp_times2) << " [ms]";
	LOG(INFO) << "Final times1: " << getMean(final_times1) << " [s]";
	LOG(INFO) << "Final times2: " << getMean(final_times2) << " [s]";

	google::ShutDownCommandLineFlags();
	return 0;
}
