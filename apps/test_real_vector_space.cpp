#include <iostream>
#include <RealVectorSpaceFCL.h>
#include <Environment.h>
#include <Planar2DOF.h>
#include <Scenario.h>
#include <ConfigurationReader.h>
#include <RGBTConnect.h>

#include <glog/logging.h>


int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

	ConfigurationReader::initConfiguration();
	scenario::Scenario scenario("/data/xarm6/scenario_test.yaml");
	scenario::Scenario scenario_FCL("/data/xarm6/scenario1.yaml");

	std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();
	std::shared_ptr<base::StateSpace> ss_FCL = scenario_FCL.getStateSpace();
	LOG(INFO) << "Environment parts: " << scenario.getEnvironment()->getParts().size();
	LOG(INFO) << "Dimensions: " << ss->getDimensions();
	LOG(INFO) << "State space ss type:     " << ss->getStateSpaceType();
	LOG(INFO) << "State space ss_FCL type: " << ss_FCL->getStateSpaceType();

	try
	{
		// std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rbt::RGBTConnect>(ss, scenario.getStart(), scenario.getGoal());
		float d_c;
		std::shared_ptr<std::vector<Eigen::MatrixXf>> planes;
		int num = 0;
		while (num++ < 1)
		{
			// std::shared_ptr<base::State> q = ss->randomState();
			// Eigen::VectorXf Q(6); Q << 1.5708, 1.5708, -2.3562, 0, 0, 0;
			// std::shared_ptr<base::State> q = std::make_shared<base::RealVectorSpaceState>(Q);
			std::shared_ptr<base::State> q = scenario.getStart();
			LOG(INFO) << "Num: " << num << " Configuration: " << q << std::endl;
			LOG(INFO) << *ss->robot->computeSkeleton(q) << std::endl;
			ss->robot->setState(q);
			
			// Test distance underestimation
			// if (ss->isValid(q))
			// {
			// 	tie(d_c, planes) = ss->computeDistanceAndPlanes(q);
			// 	float d_c_under = planner->computeDistanceUnderestimation(q, planes);
			// 	if (abs(d_c - d_c_under) > 1e-3)
			// 	{
			// 		LOG(INFO) << "************************ different ************************" << std::endl;
			// 		LOG(INFO) << "d_c = " << d_c << std::endl;
			// 		LOG(INFO) << "d_c_under = " << d_c_under << std::endl;
			// 		throw;
			// 	}
			// }
			// else
			// 	LOG(INFO) << "invalid " << std::endl;
			

			// LOG(INFO) << "-------------------- WITHOUT FCL --------------------" << std::endl;
			// bool valid = ss->isValid(q);
			// LOG(INFO) << "Is valid: " << (valid ? "true" : "false") << std::endl;
			// float d_c = ss->computeDistance(q); if (d_c == 0) d_c = -1;
			// LOG(INFO) << "Distance: " << d_c << std::endl;

			// LOG(INFO) << "-------------------- WITH FCL -----------------------" << std::endl;
			// bool valid_FCL = ss_FCL->isValid(q);
			// LOG(INFO) << "Is valid: " << (valid_FCL ? "true" : "false") << std::endl;
			// float d_c_FCL = ss_FCL->computeDistance(q);
			// // float d_c_FCL = std::get<0>(ss_FCL->computeDistanceAndPlanes(q));
			// LOG(INFO) << "Distance: " << d_c_FCL << std::endl;
			
			// if (valid != valid_FCL)
			// 	throw std::domain_error("DIFFERENT ISVALID");
			
			// if (std::abs(d_c - d_c_FCL) > 1e-2)
			// 	throw std::domain_error("DIFFERENT DISTANCE");
			
			LOG(INFO) << std::endl;
		}
		LOG(INFO) << "Test completed successfully! " << std::endl;
	}
	catch (std::domain_error &e)
	{
		LOG(ERROR) << e.what();
	}
	google::ShutDownCommandLineFlags();
	return 0;
}
