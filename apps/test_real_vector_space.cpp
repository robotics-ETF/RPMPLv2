#include "ConfigurationReader.h"
#include "RealVectorSpace.h"
#include "RealVectorSpaceFCL.h"
#include "CommonFunctions.h"

int main(int argc, char **argv)
{
	std::string scenario_file_path1 { "/data/xarm6/scenario_test/scenario_test.yaml" };
	std::string scenario_file_path2 { "/data/xarm6/scenario1/scenario1.yaml" };

	initGoogleLogging(argv);
	int clp = commandLineParser(argc, argv, scenario_file_path1);
	if (clp != 0) return clp;

	const std::string project_path { getProjectPath() };
	ConfigurationReader::initConfiguration(project_path);

	scenario::Scenario scenario(scenario_file_path1, project_path);
	scenario::Scenario scenario_FCL(scenario_file_path2, project_path);

	std::shared_ptr<base::StateSpace> ss { scenario.getStateSpace() };
	std::shared_ptr<base::StateSpace> ss_FCL { scenario_FCL.getStateSpace() };
	LOG(INFO) << "Number of objects in environment: " << scenario.getEnvironment()->getNumObjects();
	LOG(INFO) << "Number of DOFs: " << ss->num_dimensions;
	LOG(INFO) << "State space ss type:     " << ss->getStateSpaceType();
	LOG(INFO) << "State space ss_FCL type: " << ss_FCL->getStateSpaceType();

	try
	{
		float d_c {};
		std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points { nullptr };
		size_t num { 0 };

		while (num++ < 1)
		{
			std::shared_ptr<base::State> q { ss->getRandomState() };
			// Eigen::VectorXf Q(6); Q << 1.5708, 1.5708, -2.3562, 0, 0, 0;
			// std::shared_ptr<base::State> q { ss->getNewState(Q) };
			// std::shared_ptr<base::State> q { scenario.getStart() };

			LOG(INFO) << "Num: " << num << " Configuration: " << q << std::endl;
			LOG(INFO) << *ss->robot->computeSkeleton(q) << std::endl;
			ss->robot->setState(q);
			
			// Test distance underestimation
			if (ss->isValid(q))
			{
				d_c = ss->computeDistance(q);
				float d_c_under { ss->computeDistanceUnderestimation(q, q->getNearestPoints()) };
				if (abs(d_c - d_c_under) > 1e-3)
				{
					LOG(INFO) << "************************ different ************************" << std::endl;
					LOG(INFO) << "d_c = " << d_c << std::endl;
					LOG(INFO) << "d_c_under = " << d_c_under << std::endl;
					throw;
				}
			}
			else
				LOG(INFO) << "invalid " << std::endl;
			

			LOG(INFO) << "-------------------- WITHOUT FCL --------------------" << std::endl;
			bool valid { ss->isValid(q) };
			LOG(INFO) << "Is valid: " << (valid ? "true" : "false") << std::endl;
			float d_c { ss->computeDistance(q) }; 
			if (d_c == 0) d_c = -1;
			LOG(INFO) << "Distance: " << d_c << std::endl;

			LOG(INFO) << "-------------------- WITH FCL -----------------------" << std::endl;
			bool valid_FCL { ss_FCL->isValid(q) };
			LOG(INFO) << "Is valid: " << (valid_FCL ? "true" : "false") << std::endl;
			float d_c_FCL { ss_FCL->computeDistance(q) };
			LOG(INFO) << "Distance: " << d_c_FCL << std::endl;
			
			if (valid != valid_FCL)
				throw std::domain_error("DIFFERENT ISVALID");
			
			if (std::abs(d_c - d_c_FCL) > 1e-2)
				throw std::domain_error("DIFFERENT DISTANCE");
			
			LOG(INFO) << std::endl;
		}
		LOG(INFO) << "Test completed successfully! " << std::endl;
	}
	catch (std::exception &e)
	{
		LOG(ERROR) << e.what();
	}

	google::ShutDownCommandLineFlags();
	return 0;
}
