// Measures forward kinematics and the generation of one SL_DRGBT network input (pattern tree + features), i.e.,
// the work done before every ONNX prediction, for planar 2-DOF and xArm6 robots. All inputs come from fixed seeds,
// so results are comparable between builds. The printed feature hashes must not change when FK/distance code is
// optimised, otherwise the trained models are invalid.

#include "SL_DRGBT.h"
#include "ConfigurationReader.h"
#include "CommonFunctions.h"
#include "Spatial10DOF.h"

#include <chrono>
#include <cstring>
#include <iomanip>
#include <random>

namespace
{
	Eigen::VectorXf randomConfig(const std::vector<std::pair<float, float>> &limits, std::mt19937 &generator)
	{
		Eigen::VectorXf q(limits.size());
		for (size_t i = 0; i < limits.size(); i++)
			q(i) = std::uniform_real_distribution<float>(limits[i].first, limits[i].second)(generator);

		return q;
	}

	// Mean time per call of 'f' in [us], measured over 'num' calls.
	template <class F>
	double meanTimeUs(size_t num, F &&f)
	{
		auto time_start { std::chrono::steady_clock::now() };
		for (size_t i = 0; i < num; i++)
			f(i);

		return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - time_start).count() / num;
	}

	// FNV-1a hash of the raw bytes of all features.
	uint64_t hashFeatures(const std::vector<std::vector<float>> &inputs)
	{
		uint64_t hash { 14695981039346656037ull };
		for (const std::vector<float> &input : inputs)
		{
			for (float value : input)
			{
				uint32_t bits {};
				std::memcpy(&bits, &value, sizeof(bits));
				for (size_t k = 0; k < 4; k++)
					hash = (hash ^ ((bits >> (8 * k)) & 0xFF)) * 1099511628211ull;
			}
		}
		return hash;
	}

	void benchmarkForwardKinematics(const std::string &name, robots::AbstractRobot &robot, size_t num)
	{
		std::mt19937 generator(42);
		std::vector<std::shared_ptr<base::State>> states {};
		for (size_t i = 0; i < num; i++)
			states.emplace_back(std::make_shared<base::RealVectorSpaceState>(randomConfig(robot.getLimits(), generator)));

		// Each state is fresh, so its frames are not cached yet.
		double time_fk { meanTimeUs(num, [&](size_t i) { robot.computeForwardKinematics(states[i]); }) };
		std::cout << std::left << std::setw(16) << name << std::right << std::fixed << std::setprecision(3)
				  << std::setw(10) << time_fk << " us per FK call\n";
	}

	// Same placement as 'initRandomObstacles' for static obstacles, but with a fixed seed.
	void addObstacles(scenario::Scenario &scenario, size_t num_obstacles, const Eigen::Vector3f &dim, std::mt19937 &generator)
	{
		std::shared_ptr<base::StateSpace> ss { scenario.getStateSpace() };
		std::shared_ptr<env::Environment> env { scenario.getEnvironment() };
		std::uniform_real_distribution<float> distribution(0.0, 1.0);
		size_t num_obs { env->getNumObjects() };

		for (size_t i = num_obs; i < num_obs + num_obstacles; i++)
		{
			float r { distribution(generator) * env->getWSRadius() };
			float fi { distribution(generator) * 2 * float(M_PI) };
			Eigen::Vector3f pos { env->getWSCenter() + r * Eigen::Vector3f(std::cos(fi), std::sin(fi), 0) };
			env->addObject(std::make_shared<env::Box>(dim, pos, Eigen::Quaternionf::Identity(), "random_static_obstacle"));

			if (!env->isValid(pos, 0) || !ss->isValid(scenario.getStart()) || !ss->isValid(scenario.getGoal()))
			{
				env->removeObject(i);
				i--;
			}
		}
	}

	// Generates 'num_inputs' network inputs from random valid configurations, and predicts the next states for them.
	void benchmarkNetworkInput(scenario::Scenario &scenario, const std::string &model_path, size_t num_inputs, std::mt19937 &generator)
	{
		std::shared_ptr<base::StateSpace> ss { scenario.getStateSpace() };
		SL_DRGBTConfig::ONNX_MODEL_PATH = model_path;
		planning::drbt::SL_DRGBT planner(ss, scenario.getStart(), scenario.getGoal());
		Predictor predictor(model_path);

		std::vector<std::shared_ptr<base::State>> roots {};
		while (roots.size() < num_inputs)
		{
			std::shared_ptr<base::State> q { ss->getNewState(randomConfig(ss->robot->getLimits(), generator)) };
			if (ss->isValid(q) && !ss->robot->checkSelfCollision(q))
				roots.emplace_back(q);
		}

		std::vector<std::vector<float>> inputs(num_inputs);
		const double time_input { meanTimeUs(num_inputs, [&](size_t i) { planner.generateInput(roots[i], inputs[i]); }) };
		const double time_predict { meanTimeUs(num_inputs, [&](size_t i) { predictor.predict(inputs[i]); }) };

		// 'generateHorizon' is estimated as 'NUM_NEW_STATES' consecutive predictions (input generation + ONNX predict).
		std::cout << std::fixed << std::setprecision(1)
				  << "  generateInput:     " << std::setw(9) << time_input << " us per input (" << inputs.front().size() << " values)\n"
				  << "  ONNX predict:      " << std::setw(9) << time_predict << " us per input (" << model_path.substr(model_path.find("/data/") + 1) << ")\n"
				  << "  generateHorizon:   " << std::setw(9) << SL_DRGBTConfig::NUM_NEW_STATES * (time_input + time_predict)
				  << " us per iteration (" << SL_DRGBTConfig::NUM_NEW_STATES << " predictions)\n"
				  << "  feature hash:      " << std::hex << hashFeatures(inputs) << std::dec << "\n";
	}
}

int main([[maybe_unused]] int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	FLAGS_logtostderr = true;
	FLAGS_minloglevel = google::WARNING;

	const std::string project_path { getProjectPath() };
	ConfigurationReader::initConfiguration(project_path);

	// ---------------------------------------------------------------------------------------------------- //
	std::cout << "Forward kinematics:\n";
	const size_t num_fk { 20000 };
	robots::Planar2DOF planar_2dof(project_path + "/data/planar_2dof/planar_2dof.urdf");
	robots::Spatial10DOF spatial_10dof(project_path + "/data/spatial_10dof/spatial_10dof.urdf", 0);
	robots::xArm6 xarm6(project_path + "/data/xarm6/xarm6.urdf");
	benchmarkForwardKinematics("planar_2dof", planar_2dof, num_fk);
	benchmarkForwardKinematics("spatial_10dof", spatial_10dof, num_fk);
	benchmarkForwardKinematics("xarm6", xarm6, num_fk);

	// ---------------------------------------------------------------------------------------------------- //
	{
		std::cout << "\nSL_DRGBT network input (planar_2dof, 8 obstacles, " << SL_DRGBTConfig::NUM_LAYERS << "-layer pattern tree):\n";
		const Eigen::Vector3f obs_dim(0.05, 0.05, 0.05);
		std::mt19937 generator(7);

		scenario::Scenario scenario("/data/planar_2dof/scenario_test/scenario_test.yaml", project_path);
		std::shared_ptr<base::StateSpace> ss { scenario.getStateSpace() };
		std::shared_ptr<env::Environment> env { scenario.getEnvironment() };
		env->setBaseRadius(std::max(ss->robot->getCapsuleRadius(0), ss->robot->getCapsuleRadius(1)) + obs_dim.norm());
		env->setRobotMaxVel(ss->robot->getMaxVel(0));
		addObstacles(scenario, 8, obs_dim, generator);
		benchmarkNetworkInput(scenario, project_path + "/data/planar_2dof/onnx_models/GNN_8obs.onnx", 500, generator);
	}

	// ---------------------------------------------------------------------------------------------------- //
	{
		// Without the gripper: with it, self-collision checking (used when generating spines) depends on uninitialised
		// memory (see TODO in 'xArm6::checkRealSelfCollision'), so the feature hash would not be reproducible.
		std::cout << "\nSL_DRGBT network input (xarm6, 11 obstacles, no gripper, " << SL_DRGBTConfig::NUM_LAYERS << "-layer pattern tree):\n";
		std::mt19937 generator(7);
		scenario::Scenario scenario("/data/xarm6/scenario_random_obstacles/scenario_random_obstacles1.yaml", project_path);
		benchmarkNetworkInput(scenario, project_path + "/data/xarm6/onnx_models/GATR.onnx", 50, generator);
	}

	google::ShutDownCommandLineFlags();
	return 0;
}
