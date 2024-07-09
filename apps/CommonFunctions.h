#include <vector>
#include <string>
#include <memory>
#include <ostream>
#include <numeric>
#include <glog/logging.h>

#include "Scenario.h"
#include "CommandLine.h"

void initGoogleLogging(char **argv)
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";
}

int commandLineParser(int argc, char **argv, std::string &scenario_file_path)
{
	bool print_help { false };
	CommandLine args("Test command line parser.");
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
		return 1;
	}

	return 0;
}

const std::string getProjectPath()
{
	std::string project_path(__FILE__);
    for (size_t i = 0; i < 2; i++) {	// This depends on how deep is this file located
        project_path = project_path.substr(0, project_path.find_last_of("/\\"));
	}
	return project_path;
}

float getMean(std::vector<float> &v)
{
	if (v.empty()) 
		return INFINITY;
		
	return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}

float getStd(std::vector<float> &v)
{
	if (v.empty()) 
		return INFINITY;
		
	float mean { getMean(v) };
	float sum { 0 };
	for (size_t i = 0; i < v.size(); i++)
		sum += (v[i] - mean) * (v[i] - mean);

	return std::sqrt(sum / v.size());
}

void initRandomObstacles(size_t num_obstacles, const Eigen::Vector3f &dim, scenario::Scenario &scenario)
{
	LOG(INFO) << "Adding " << num_obstacles << " random obstacles...";

	std::shared_ptr<base::StateSpace> ss { scenario.getStateSpace() };
	std::shared_ptr<env::Environment> env { scenario.getEnvironment() };
	const Eigen::Vector3f WS_center { env->getWSCenter() };

	Eigen::Vector3f pos {};
	float r { 0 }, fi { 0 }, theta { 0 };
	size_t num_obs { env->getNumObjects() };
	std::random_device rd {};
	std::mt19937 generator(rd());
	std::uniform_real_distribution<float> distribution(0.0, 1.0);
	std::shared_ptr<env::Object> object { nullptr };
	
	for (size_t i = num_obs; i < num_obs + num_obstacles; i++)
	{
		r = distribution(generator) * env->getWSRadius();
		fi = distribution(generator) * 2 * M_PI;
		theta = distribution(generator) * M_PI;
		pos.x() = WS_center.x() + r * std::cos(fi) * std::sin(theta);
		pos.y() = WS_center.y() + r * std::sin(fi) * std::sin(theta);
		pos.z() = WS_center.z() + r * std::cos(theta);

		object = std::make_shared<env::Box>(dim, pos, Eigen::Quaternionf::Identity(), "random_obstacle");
		env->addObject(object);

		if (!env->isValid(pos, 0) || !ss->isValid(scenario.getStart()) || !ss->isValid(scenario.getGoal()) ||
			ss->robot->checkSelfCollision(scenario.getStart()) || ss->robot->checkSelfCollision(scenario.getGoal()))
		{
			env->removeObject(i);
			i--;
		}
		// else std::cout << "Added " << i << ". " << object;
	}
}

void initRandomObstacles(size_t num_obstacles, const Eigen::Vector3f &dim, scenario::Scenario &scenario, 
	float max_vel, float max_acc)
{
	LOG(INFO) << "Adding " << num_obstacles << " random obstacles...";

	std::shared_ptr<base::StateSpace> ss { scenario.getStateSpace() };
	std::shared_ptr<env::Environment> env { scenario.getEnvironment() };
	const Eigen::Vector3f WS_center { env->getWSCenter() };

	std::vector<std::shared_ptr<env::Object>> fixed_objects { env->getObjects() };
	env->removeAllObjects();

	Eigen::Vector3f pos {}, vel {}, acc {};
	size_t num_obs { env->getNumObjects() };
	float r { 0 }, fi { 0 }, theta { 0 };
	std::random_device rd {};
	std::mt19937 generator(rd());
	std::uniform_real_distribution<float> distribution(0.0, 1.0);
	std::shared_ptr<env::Object> object { nullptr };
	
	for (size_t i = num_obs; i < num_obs + num_obstacles; i++)
	{
		r = distribution(generator) * env->getWSRadius();
		fi = distribution(generator) * 2 * M_PI;
		theta = distribution(generator) * M_PI;
		pos.x() = WS_center.x() + r * std::cos(fi) * std::sin(theta);
		pos.y() = WS_center.y() + r * std::sin(fi) * std::sin(theta);
		pos.z() = WS_center.z() + r * std::cos(theta);

		object = std::make_shared<env::Box>(dim, pos, Eigen::Quaternionf::Identity(), "dynamic_obstacle");
		object->setMaxVel(max_vel);
		object->setMaxAcc(max_acc);

		vel = Eigen::Vector3f::Random(3);
		vel.normalize();
		vel *= distribution(generator) * max_vel;

		acc = Eigen::Vector3f::Random(3);
		acc.normalize();
		acc *= distribution(generator) * max_acc;
		
		env->addObject(object, vel);
		// env->addObject(object, vel, acc);

		if (!env->isValid(pos, vel.norm()) || 
			ss->computeDistance(scenario.getStart(), true) < 6 * DRGBTConfig::D_CRIT) // Just to ensure safety of init. conf.
		{
			env->removeObject(i);
			i--;
		}
		// else
    	// 	std::cout << "Added " << i << ". " << object;
	}

	// Restore deleted fixed objects from the beginning
	for (std::shared_ptr<env::Object> obj : fixed_objects)
        env->addObject(obj);
	
	// Reset all additional data set before for start conf.
	scenario.getStart()->setDistance(-1);
	scenario.getStart()->setDistanceProfile(std::vector<float>());
	scenario.getStart()->setNearestPoints(nullptr);
}

void generateRandomStartAndGoal(scenario::Scenario &scenario, float min_dist_start_goal)
{
	LOG(INFO) << "Generating random start and goal configuration, where minimal distance is " << min_dist_start_goal << " [m]...";

	std::shared_ptr<base::StateSpace> ss { scenario.getStateSpace() };
	std::shared_ptr<base::State> q_start { nullptr };
	std::shared_ptr<base::State> q_goal { nullptr };
	std::shared_ptr<base::State> q_middle { nullptr };
	std::shared_ptr<Eigen::MatrixXf> skeleton_start {};
	std::shared_ptr<Eigen::MatrixXf> skeleton_goal {};
	std::shared_ptr<Eigen::MatrixXf> skeleton_middle {};
	float dist { 0 };

	while (dist < min_dist_start_goal)
	{
		q_start = ss->getRandomState();
		q_goal = ss->getRandomState();
		if (ss->computeDistance(q_start, true) < DRGBTConfig::D_CRIT || ss->computeDistance(q_goal, true) < DRGBTConfig::D_CRIT ||
			ss->robot->checkSelfCollision(q_start) || ss->robot->checkSelfCollision(q_goal))
			continue;
		
		q_middle = ss->getNewState((q_start->getCoord() + q_goal->getCoord()) / 2);
		skeleton_start = ss->robot->computeSkeleton(q_start);
		skeleton_goal = ss->robot->computeSkeleton(q_goal);
		skeleton_middle = ss->robot->computeSkeleton(q_middle);

		dist = 0;
		for (size_t k = 1; k <= ss->robot->getNumLinks(); k++)
		{
			dist += (skeleton_start->col(k) - skeleton_middle->col(k)).norm() 
					+ (skeleton_middle->col(k) - skeleton_goal->col(k)).norm();
		}
		// std::cout << "dist: " << dist << "\n";
	}

	scenario.setStart(q_start);
	scenario.setGoal(q_goal);
}
