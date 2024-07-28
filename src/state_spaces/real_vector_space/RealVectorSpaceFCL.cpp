#include "RealVectorSpaceFCL.h"

base::RealVectorSpaceFCL::~RealVectorSpaceFCL() {}

base::RealVectorSpaceFCL::RealVectorSpaceFCL(size_t num_dimensions_, const std::shared_ptr<robots::AbstractRobot> robot_, 
											 const std::shared_ptr<env::Environment> env_) : RealVectorSpace(num_dimensions_, robot_, env_)
{
	setStateSpaceType(base::StateSpaceType::RealVectorSpaceFCL);
	collision_manager_robot = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
	collision_manager_env = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
}

bool base::RealVectorSpaceFCL::isValid(const std::shared_ptr<base::State> q)
{
	robot->setState(q);	
	fcl::DefaultCollisionData<float> collision_data {};

	for (size_t i = 0; i < robot->getNumLinks(); i++)
	{	
		for (size_t j = 0; j < env->getNumObjects(); j++)
		{
			if (env->getObject(j)->getLabel() == "ground" && i < robot->getGroundIncluded())
				continue;
			else
			{
				collision_manager_robot->clear();
				collision_manager_env->clear();
				collision_manager_robot->registerObject(robot->getLinks()[i].get());
				collision_manager_env->registerObject(env->getCollObject(j).get());
				collision_manager_robot->setup();
				collision_manager_env->setup();

				collision_manager_robot->collide(collision_manager_env.get(), &collision_data, fcl::DefaultCollisionFunction);
				if (collision_data.result.isCollision())
					return false;
			}			
		}
	}
	return true;
}

/// @brief Compute a minimal distance from the robot in configuration 'q' to obstacles using FCL library.
/// In other words, compute a minimal distance from each robot's link in configuration 'q' to obstacles, 
/// i.e., compute a distance profile function. 
/// Moreover, set 'd_c', 'd_c_profile', and corresponding 'nearest_points' for the configuation 'q'.
/// @param q Configuration of the robot.
/// @param compute_again If true, a new distance profile will be computed again! Default: false.
/// @return Minimal distance from the robot in configuration 'q' to obstacles.
float base::RealVectorSpaceFCL::computeDistance(const std::shared_ptr<base::State> q, bool compute_again)
{
	if (!compute_again && q->getDistance() > 0 && q->getIsRealDistance())
		return q->getDistance();
	
	float d_c { INFINITY };
	std::vector<float> d_c_profile(robot->getNumLinks(), 0);
	std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points { std::make_shared<std::vector<Eigen::MatrixXf>>
		(env->getNumObjects(), Eigen::MatrixXf(6, robot->getNumLinks())) };
	std::shared_ptr<Eigen::MatrixXf> nearest_pts { std::make_shared<Eigen::MatrixXf>(3, 2) };
	fcl::DefaultDistanceData<float> distance_data;
	robot->setState(q);
	
	for (size_t i = 0; i < robot->getNumLinks(); i++)
	{
		d_c_profile[i] = INFINITY;
		for (size_t j = 0; j < env->getNumObjects(); j++)
		{
			if (env->getObject(j)->getLabel() == "ground" && i < robot->getGroundIncluded())
			{
				nearest_pts->col(0) << 0, 0, 0; 			// Robot nearest point
				nearest_pts->col(1) << 0, 0, -INFINITY;		// Obstacle nearest point
			}
			else
			{
				collision_manager_robot->clear();
				collision_manager_env->clear();
				collision_manager_robot->registerObject(robot->getLinks()[i].get());
				collision_manager_env->registerObject(env->getCollObject(j).get());
				collision_manager_robot->setup();
				collision_manager_env->setup();

				distance_data.request.enable_nearest_points = true;
				distance_data.result.clear();
				collision_manager_robot->distance(collision_manager_env.get(), &distance_data, fcl::DefaultDistanceFunction);

				if (distance_data.result.min_distance > env->getObject(j)->getMinDistTol())
					distance_data.result.min_distance = INFINITY;

				d_c_profile[i] = std::min(d_c_profile[i], distance_data.result.min_distance);
				nearest_pts->col(0) = distance_data.result.nearest_points[0];
				nearest_pts->col(1) = distance_data.result.nearest_points[1];

				// LOG(INFO) << "(i, j) = (" << i << ", " << j << ")" << std::endl;
				// LOG(INFO) << "d_c = " << distance_data.result.min_distance << std::endl;
				// LOG(INFO) << "NP robot: " << nearest_pts->col(0).transpose() << std::endl;
				// LOG(INFO) << "NP env:   " << nearest_pts->col(1).transpose() << std::endl;
			}

			if (d_c_profile[i] <= 0)		// The collision occurs
            {
				q->setDistance(0);
				q->setDistanceProfile(d_c_profile);
				q->setIsRealDistance(true);
				q->setNearestPoints(nullptr);
				return 0;
			}
			
			// 'nearest_pts->col(0)' is robot nearest point, and 'nearest_pts->col(1)' is obstacle nearest point
			nearest_points->at(j).col(i) << nearest_pts->col(0), nearest_pts->col(1);
		}
		d_c = std::min(d_c, d_c_profile[i]);
	}
	
	q->setDistance(d_c);
	q->setDistanceProfile(d_c_profile);
	q->setIsRealDistance(true);
	q->setNearestPoints(nearest_points);
	
	return d_c;
}
