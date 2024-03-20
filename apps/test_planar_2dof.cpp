#include <ostream>
#include <memory>

#include "Planar2DOF.h"
#include <glog/logging.h>
#include <RealVectorSpace.h>
#include "Scenario.h"
#include "CommandLine.h"
#include "ConfigurationReader.h"
#include "RealVectorSpaceFCL.h"

Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");

int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int)time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

	std::string scenario_file_path = "/data/planar_2dof/scenario_easy.yaml";
	bool print_help = false;

	CommandLine args("Test Planar2DOF command line parser.");
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
    for (size_t i = 0; i < 2; i++) {
        project_path = project_path.substr(0, project_path.find_last_of("/\\"));
	}

	ConfigurationReader::initConfiguration();
	scenario::Scenario scenario(project_path + scenario_file_path);
	std::shared_ptr<base::RealVectorSpaceFCL> ss = std::dynamic_pointer_cast<base::RealVectorSpaceFCL>(scenario.getStateSpace());

	std::shared_ptr<robots::Planar2DOF> robot =  std::dynamic_pointer_cast<robots::Planar2DOF>(scenario.getRobot());
	std::shared_ptr<base::State> testState = std::make_shared<base::RealVectorSpaceState>(Eigen::Vector2f({1.35, -2.0}));
	robot->setState(testState);
	std::shared_ptr<fcl::CollisionObject<float>> ob = scenario.getEnvironment()->getCollObject(0);

	//robot->test(scenario.getEnvironment(), testState);
	fcl::DefaultDistanceData<float> distance_data;
	distance_data.request.enable_nearest_points = true;

	ss->getCollisionManagerRobot()->distance(ob.get(), &distance_data, fcl::DefaultDistanceFunction);
	LOG(INFO) << "distance from robot : " << distance_data.result.min_distance << " p1: " << distance_data.result.nearest_points[0].transpose().format(fmt)
			<< "\t p2: " << distance_data.result.nearest_points[1].transpose().format(fmt);

	google::ShutDownCommandLineFlags();
	return 0;
}