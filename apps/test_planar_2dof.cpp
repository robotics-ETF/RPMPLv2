#include <ostream>
#include <memory>

#include "ConfigurationReader.h"
#include "CommonFunctions.h"
#include "Planar2DOF.h"
#include "RealVectorSpaceFCL.h"

Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");

int main(int argc, char **argv)
{
	std::string scenario_file_path { "/data/planar_2dof/scenario_test/scenario_test.yaml" };
	
	initGoogleLogging(argv);
	int clp = commandLineParser(argc, argv, scenario_file_path);
	if (clp != 0) return clp;

	const std::string project_path { getProjectPath() };
	ConfigurationReader::initConfiguration(project_path);

	scenario::Scenario scenario(scenario_file_path, project_path);
	std::shared_ptr<base::RealVectorSpaceFCL> ss { std::dynamic_pointer_cast<base::RealVectorSpaceFCL>(scenario.getStateSpace()) };
	std::shared_ptr<robots::Planar2DOF> robot { std::dynamic_pointer_cast<robots::Planar2DOF>(scenario.getRobot()) };
	std::shared_ptr<base::State> testState { std::make_shared<base::RealVectorSpaceState>(Eigen::Vector2f({1.35, -2.0})) };
	std::shared_ptr<fcl::CollisionObject<float>> ob { scenario.getEnvironment()->getCollObject(0) };
	
	robot->setState(testState);

	// robot->test(scenario.getEnvironment(), testState);
	fcl::DefaultDistanceData<float> distance_data;
	distance_data.request.enable_nearest_points = true;

	ss->getCollisionManagerRobot()->distance(ob.get(), &distance_data, fcl::DefaultDistanceFunction);
	LOG(INFO) << "Distance from robot: " << distance_data.result.min_distance << "\t"
			  << "p1: " << distance_data.result.nearest_points[0].transpose().format(fmt) << "\t"
			  << "p2: " << distance_data.result.nearest_points[1].transpose().format(fmt);

	google::ShutDownCommandLineFlags();
	return 0;
}
