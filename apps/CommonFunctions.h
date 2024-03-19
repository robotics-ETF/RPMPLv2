#include <vector>
#include <string>
#include <memory>
#include <ostream>
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
    for (int i = 0; i < 2; i++) {	// This depends on how deep is this file located
        project_path = project_path.substr(0, project_path.find_last_of("/\\"));
	}
	return project_path;
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

void initRandomObstacles(int num_obstacles, const Eigen::Vector3f &dim, scenario::Scenario &scenario)
{
	LOG(INFO) << "Adding " << num_obstacles << " random obstacles...";

	std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();
	std::shared_ptr<env::Environment> env = scenario.getEnvironment();
	const Eigen::Vector3f WS_center = env->getWSCenter();

	Eigen::Vector3f pos;
	float r, fi, theta;
	int num_obs = env->getNumObjects();
	std::random_device rd;
	std::mt19937 generator(rd());
	std::uniform_real_distribution<float> distribution(0.0, 1.0);
	
	for (int i = num_obs; i < num_obs + num_obstacles; i++)
	{
		r = distribution(generator) * env->getWSRadius();
		fi = distribution(generator) * 2 * M_PI;
		theta = distribution(generator) * M_PI;
		pos.x() = WS_center.x() + r * std::cos(fi) * std::sin(theta);
		pos.y() = WS_center.y() + r * std::sin(fi) * std::sin(theta);
		pos.z() = WS_center.z() + r * std::cos(theta);

		std::shared_ptr<env::Object> object = 
			std::make_shared<env::Box>(dim, pos, Eigen::Quaternionf::Identity(), "random_obstacle");
		env->addObject(object);

		if (!env->isValid(pos, 0) || !ss->isValid(scenario.getStart()) || !ss->isValid(scenario.getGoal()))
		{
			env->removeObject(i);
			i--;
		}
		else
    		std::cout << "Added " << i << ". " << object;
	}
}

void initRandomObstacles(int num_obstacles, const Eigen::Vector3f &dim, scenario::Scenario &scenario, 
	float max_vel, float max_acc)
{
	LOG(INFO) << "Adding " << num_obstacles << " random obstacles...";

	std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();
	std::shared_ptr<env::Environment> env = scenario.getEnvironment();
	const Eigen::Vector3f WS_center = env->getWSCenter();

	std::vector<std::shared_ptr<env::Object>> fixed_objects = env->getObjects();
	env->removeAllObjects();

	Eigen::Vector3f pos, vel, acc;
	int num_obs = env->getNumObjects();
	float r, fi, theta;
	std::random_device rd;
	std::mt19937 generator(rd());
	std::uniform_real_distribution<float> distribution(0.0, 1.0);
	
	for (int i = num_obs; i < num_obs + num_obstacles; i++)
	{
		r = distribution(generator) * env->getWSRadius();
		fi = distribution(generator) * 2 * M_PI;
		theta = distribution(generator) * M_PI;
		pos.x() = WS_center.x() + r * std::cos(fi) * std::sin(theta);
		pos.y() = WS_center.y() + r * std::sin(fi) * std::sin(theta);
		pos.z() = WS_center.z() + r * std::cos(theta);

		std::shared_ptr<env::Object> object = 
			std::make_shared<env::Box>(dim, pos, Eigen::Quaternionf::Identity(), "dynamic_obstacle");
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
	
}