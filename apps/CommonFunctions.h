#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include <numeric>

#include <Scenario.h>
#include <CommandLine.h>
#include <glog/logging.h>

void initGoogleLogging(char **argv)
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";
}

int commandLineParser(int argc, char **argv, std::string &scenario_file_path)
{
	bool print_help = false;
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
    for (int i = 0; i < 2; i++) 	// This depends on how deep is this file located
        project_path = project_path.substr(0, project_path.find_last_of("/\\"));

	return project_path;
}

float getMean(std::vector<float> &v)
{
	if (v.empty()) 
		return INFINITY;
		
	float sum = std::accumulate(v.begin(), v.end(), 0.0);
	return sum / v.size();
}

float getMean(std::vector<int> &v)
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
	for (int i = 0; i < v.size(); i++)
		sum += (v[i] - mean) * (v[i] - mean);

	return std::sqrt(sum / v.size());
}

float getStd(std::vector<int> &v)
{
	if (v.empty()) 
		return INFINITY;
		
	float mean = getMean(v);
	float sum = 0;
	for (int i = 0; i < v.size(); i++)
		sum += (v[i] - mean) * (v[i] - mean);

	return std::sqrt(sum / v.size());
}

void initRandomObstacles(scenario::Scenario &scenario, int num_obstacles)
{
	const Eigen::Vector3f dim = Eigen::Vector3f(0.1, 0.1, 0.1);
	const Eigen::Vector3f WS_center = Eigen::Vector3f(0, 0, 0.267);
	const float WS_radius = 1.0;
	std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();
	std::shared_ptr<env::Environment> env = scenario.getEnvironment();

	Eigen::Vector3f pos;
	Eigen::Matrix3f rot = fcl::Quaternionf(0, 0, 0, 0).matrix();
	float r, fi, theta;
	int num_obs = env->getParts().size();
	std::random_device rd;
	std::mt19937 generator(rd());
	std::uniform_real_distribution<float> distribution(0.0, 1.0);
	
	for (int i = num_obs; i < num_obs + num_obstacles; i++)
	{
		r = distribution(generator) * WS_radius;
		fi = distribution(generator) * 2 * M_PI;
		theta = distribution(generator) * M_PI;
		pos.x() = WS_center.x() + r * std::cos(fi) * std::sin(theta);
		pos.y() = WS_center.y() + r * std::sin(fi) * std::sin(theta);
		pos.z() = WS_center.z() + r * std::cos(theta);
		std::shared_ptr<fcl::CollisionGeometryf> box = std::make_shared<fcl::Boxf>(dim);
		std::shared_ptr<fcl::CollisionObjectf> ob = std::make_shared<fcl::CollisionObjectf>(box, rot, pos);
		env->addCollisionObject(ob);
		if (!ss->isValid(scenario.getStart()) || !ss->isValid(scenario.getGoal()))
		{
			env->removeCollisionObjects(i);
			i--;
		}
		else
    		std::cout << i << ". Obstacle range: (" << ob->getAABB().min_.transpose() << ")\t(" << ob->getAABB().max_.transpose() << ")\n";
	}
}

void initRandomObstacles(scenario::Scenario &scenario, float max_obs_vel, float max_obs_acc, int num_obstacles)
{
	const Eigen::Vector3f dim = Eigen::Vector3f(0.01, 0.01, 0.01);
	const Eigen::Vector3f WS_center = Eigen::Vector3f(0, 0, 0.267);
	const float WS_radius = 1.0;
	std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();
	std::shared_ptr<env::Environment> env = scenario.getEnvironment();

	env->setMaxVel(max_obs_vel);
	env->setMaxAcc(max_obs_acc);
	Eigen::Vector3f pos;
	Eigen::Matrix3f rot = fcl::Quaternionf(0, 0, 0, 0).matrix();
	float r, fi, theta;
	int num_obs = env->getParts().size();
	std::random_device rd;
	std::mt19937 generator(rd());
	std::uniform_real_distribution<float> distribution(0.0, 1.0);
	
	for (int i = num_obs; i < num_obs + num_obstacles; i++)
	{
		r = distribution(generator) * WS_radius;
		fi = distribution(generator) * 2 * M_PI;
		theta = distribution(generator) * M_PI;
		pos.x() = WS_center.x() + r * std::cos(fi) * std::sin(theta);
		pos.y() = WS_center.y() + r * std::sin(fi) * std::sin(theta);
		pos.z() = WS_center.z() + r * std::cos(theta);
		std::shared_ptr<fcl::CollisionGeometryf> box = std::make_shared<fcl::Boxf>(dim);
		std::shared_ptr<fcl::CollisionObjectf> ob = std::make_shared<fcl::CollisionObjectf>(box, rot, pos);
		fcl::Vector3f vel = fcl::Vector3f::Random(3);
		vel.normalize();
		vel *= distribution(generator) * max_obs_vel;
		float acc_sign = (distribution(generator) > 0.5) ? 1 : -1;
		env->addCollisionObject(ob, vel, acc_sign);
		if (ss->computeDistance(scenario.getStart(), true) < 6 * DRGBTConfig::D_CRIT) // Just to ensure safety of init. conf.
		{
			env->removeCollisionObjects(i);
			i--;
		}
		// else
    	// 	std::cout << i << ". Obstacle range: (" << ob->getAABB().min_.transpose() << ")\t(" << ob->getAABB().max_.transpose() << ")\n";
	}
}