#include <ostream>
#include <memory>

#include "ConfigurationReader.h"
#include "CommonFunctions.h"
#include "xArm6.h"
#include "RealVectorSpaceFCL.h"

int main([[maybe_unused]] int argc, char **argv)
{
	initGoogleLogging(argv);

	const std::string project_path { getProjectPath() };
	ConfigurationReader::initConfiguration(project_path);

	std::shared_ptr<robots::xArm6> robot { std::make_shared<robots::xArm6>(project_path + "/data/xarm6/xarm6.urdf") };

	// LOG(INFO) << robot->getRobotTree().getNrOfSegments();
	// LOG(INFO) << robot->getNumLinks();
	// LOG(INFO) << robot->getLinks().at(0)->getCollisionGeometry()->computeVolume();
	// LOG(INFO) << robot->getLinks().at(0)->getAABB().height() << ";" 
	// 		  << robot->getLinks().at(0)->getAABB().depth() << ";" 
	// 		  << robot->getLinks().at(0)->getAABB().width();

	// Eigen::VectorXf q { Eigen::VectorXf::Zero(6) };
	// q << 0, 0, 0, M_PI, M_PI_2, 0;
	// q << 90, 33, -96, 0, 63, 0; q *= M_PI / 180;
	// std::shared_ptr<base::State> state { std::make_shared<base::RealVectorSpaceState>(q) };
	// LOG(INFO) << "State: " << state;
	// robot->setState(state);
	// robot->test();
	

	// Test inverse kinematics
	const std::vector<std::pair<float, float>> limits { robot->getLimits() };
	for (size_t k = 0; k < 10000; k++)
	{
		LOG(INFO) << "k = " << k;
		Eigen::VectorXf rand { Eigen::VectorXf::Random(6) };
		for (size_t i = 0; i < 6; i++)
			rand(i) = ((limits[i].second - limits[i].first) * rand(i) + limits[i].first + limits[i].second) / 2;
		std::shared_ptr<base::State> state { std::make_shared<base::RealVectorSpaceState>(rand) };
		// LOG(INFO) << "State: " << state;
				
		std::shared_ptr<std::vector<KDL::Frame>> frames { robot->computeForwardKinematics(state) };
		KDL::Rotation R { frames->back().M };
		KDL::Vector p { frames->back().p };
		// LOG(INFO) << "R:\n" << R;
		// LOG(INFO) << "p: " << p;

		std::shared_ptr<base::State> state_new { robot->computeInverseKinematics(R, p) };
		// LOG(INFO) << "State new: " << state_new;
		if (state_new == nullptr)
			break;

		std::shared_ptr<std::vector<KDL::Frame>> frames_new { robot->computeForwardKinematics(state_new) };
		KDL::Rotation R_new { frames_new->back().M };
		KDL::Vector p_new { frames_new->back().p };
		// LOG(INFO) << "R_new:\n" << R_new;
		// LOG(INFO) << "p_new: " << p_new;

		float error { 0 };
		for (size_t i = 0; i < 9; i++)
			error += std::pow(R.data[i] - R_new.data[i], 2);
		for (size_t i = 0; i < 3; i++)
			error += std::pow(p.data[i] - p_new.data[i], 2);
		if (error > 1e-5)
		{
			LOG(INFO) << "****************** DIFFERENT ******************";
			LOG(INFO) << "State: " << state;
			LOG(INFO) << "State new: " << state_new;
			LOG(INFO) << "R:\n" << R;
			LOG(INFO) << "R_new:\n" << R_new;
			LOG(INFO) << "p: " << p;
			LOG(INFO) << "p_new: " << p_new;
			break;
		}
	}

	return 0;
}
