#include "ONNX/Predictor.h"

#include "PatternTree.h"
#include "ConfigurationReader.h"
#include "CommonFunctions.h"

#include <chrono>
#include <iomanip>
#include <iostream>

int main(int argc, char** argv) 
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

    const std::string onnx_model_path { "/data/planar_2dof/onnx_models/GNN_8obs.onnx" };

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
	float min_dist_start_goal { node["robot"]["min_dist_start_goal"].as<float>() };
	size_t num_layers { node["testing"]["num_layers"].as<size_t>() };

	scenario::Scenario scenario(scenario_file_path, project_path);
	std::shared_ptr<base::StateSpace> ss { scenario.getStateSpace() };
	std::shared_ptr<base::State> q_start { scenario.getStart() };
	std::shared_ptr<base::State> q_goal { scenario.getGoal() };
	std::shared_ptr<env::Environment> env { scenario.getEnvironment() };
	if (ss->num_dimensions != 2 || num_layers != 3)
	{
		LOG(ERROR) << "GNN_8obs.onnx requires a planar 2-DOF scenario and a three-layer pattern tree.";
		google::ShutDownCommandLineFlags();
		return 1;
	}
	std::shared_ptr<planning::rbt::PatternTree> pattern_tree { std::make_shared<planning::rbt::PatternTree>(ss, num_layers) };
	Predictor onnxcnn(project_path + onnx_model_path);
		
	size_t num_test { 0 };
	size_t num_failed_tests { 0 };
	while (num_test++ < max_num_tests)
	{
		try
		{
			LOG(INFO) << "Test number " << num_test << " of " << max_num_tests;

			generateRandomStartAndGoal(scenario, min_dist_start_goal);
			q_start = scenario.getStart();
			q_goal = scenario.getGoal();

			if (num_random_obstacles > 0)
				initRandomObstacles(num_random_obstacles, obs_dim, scenario);

			LOG(INFO) << "Using scenario:    " << project_path + scenario_file_path;
			LOG(INFO) << "Environment parts: " << env->getNumObjects();
			LOG(INFO) << "Number of DOFs:    " << ss->num_dimensions;
			LOG(INFO) << "State space type:  " << ss->getStateSpaceType();
			LOG(INFO) << "Start:             " << scenario.getStart();
			LOG(INFO) << "Goal:              " << scenario.getGoal();
			
			// Generate input data:
			std::shared_ptr<base::Tree> tree { pattern_tree->generateLocalTree(q_start) };
			// LOG(INFO) << *tree;
            std::vector<float> input {};
            input.reserve(Predictor::InputCount);

			for (size_t j = 0; j < ss->num_dimensions; j++)
				input.emplace_back(q_start->getCoord(j));

			for (size_t j = 0; j < ss->num_dimensions; j++)
				input.emplace_back(q_goal->getCoord(j));

			for (size_t i = 1; i < pattern_tree->getNumNodes(num_layers); i++)
				input.emplace_back((tree->getState(i)->getCoord() - tree->getState(i)->getParent()->getCoord()).norm());

			for (size_t i = 0; i < pattern_tree->getNumNodes(num_layers-1); i++)
			{
				// input.emplace_back(tree->getState(i)->getDistance());	// If only distance to obstacles is used

				std::vector<float> d_c_profile { tree->getState(i)->getDistanceProfile() };
				for (size_t j = 0; j < ss->num_dimensions; j++)
					input.emplace_back(d_c_profile[j]);
			}

			for (size_t i = 0; i < pattern_tree->getNumNodes(num_layers-1); i++)
			{
				std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points { tree->getState(i)->getNearestPoints() };
				Eigen::Vector3f R {}, R_min {}, O {}, O_min {}, OR {};
				for (size_t j = 0; j < ss->num_dimensions; j++)
				{
					float d_min { RealVectorSpaceConfig::MAX_DISTANCE };
					for (size_t k = 0; k < env->getNumObjects(); k++)
					{
						R = nearest_points->at(k).col(j).head(3);
						O = nearest_points->at(k).col(j).tail(3);
						R += (O - R).normalized() * ss->robot->getCapsuleRadius(j);
						if ((R - O).norm() < d_min)
						{
							R_min = R;
							O_min = O;
							d_min = (R_min - O_min).norm();
						}
					}

					OR = (R_min - O_min).normalized();
					input.emplace_back(OR.x());
                    input.emplace_back(OR.y());
                    input.emplace_back(OR.z());
				}
			}

            std::cout << "Raw input (" << input.size() << " values): \n";
            for (std::size_t i = 0; i < input.size(); ++i) 
            {
                if (i) std::cout << ';';
                std::cout << input[i];
            }
            std::cout << '\n';

            const auto output = onnxcnn.predict(input);
            std::cout << "Output probabilities: " << std::setprecision(4);
            for (std::size_t i = 0; i < output.size(); ++i) 
            {
                if (i) std::cout << ' ';
                std::cout << output[i];
            }
            std::cout << '\n';
            
            constexpr std::size_t bounds[]{0,4,7,10,13,16};
            std::cerr << "Group sums:";
            for (std::size_t g = 0; g < 5; ++g) 
            {
                double sum = 0;
                for (std::size_t j = bounds[g]; j < bounds[g+1]; ++j) sum += output[j];
                std::cerr << ' ' << sum;
            }

            for (int i = 0; i < 20; ++i) onnxcnn.predict(input);
            const auto start = std::chrono::steady_clock::now();
            constexpr int repetitions = 200;
            for (int i = 0; i < repetitions; ++i) onnxcnn.predict(input);
            const double ms = std::chrono::duration<double, std::milli>
                (std::chrono::steady_clock::now() - start).count() / repetitions;
            std::cerr << "\nMean raw-input prediction time: " << ms << " [ms]\n";

			LOG(INFO) << "\n--------------------------------------------------------------------\n\n";
		}
		catch (std::exception &e)
		{
			LOG(ERROR) << e.what();
			++num_failed_tests;
		}
	}
	
	google::ShutDownCommandLineFlags();
	return num_failed_tests == 0 ? 0 : 1;
}
