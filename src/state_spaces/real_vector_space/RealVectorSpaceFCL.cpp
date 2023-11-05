//
// Created by dinko on 7.3.21..
//
#include "RealVectorSpaceFCL.h"
#include "RealVectorSpaceConfig.h"
#include "xArm6.h"
// #include <glog/log_severity.h>
// #include <glog/logging.h>

base::RealVectorSpaceFCL::~RealVectorSpaceFCL() {}

base::RealVectorSpaceFCL::RealVectorSpaceFCL(int num_dimensions_, const std::shared_ptr<robots::AbstractRobot> robot_, 
											 const std::shared_ptr<env::Environment> env_) : RealVectorSpace(num_dimensions_, robot_, env_)
{
	setStateSpaceType(StateSpaceType::RealVectorSpaceFCL);
	collision_manager_robot = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
	collision_manager_env = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
}

bool base::RealVectorSpaceFCL::isValid(const std::shared_ptr<base::State> q)
{
	robot->setState(q);	
	fcl::DefaultCollisionData<float> collision_data;

	for (size_t i = 0; i < robot->getParts().size(); i++)
	{	
		for (size_t j = 0; j < env->getParts().size(); j++)
		{
			// 'j == 0' always represents the table when it is included
			if ((j == 0 && (i == 0 || i == 1)) && robot->getType() == "xarm6_with_table")
				continue;
			else
			{
				collision_manager_robot->clear();
				collision_manager_env->clear();
				collision_manager_robot->registerObject(robot->getParts()[i].get());
				collision_manager_env->registerObject(env->getParts()[j].get());
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

// Get minimal distance from robot in configuration 'q' to obstacles
// Moreover, set corresponding 'nearest_points' for the configuation 'q'
// If 'compute_again' is true, the new distance will be computed again!
float base::RealVectorSpaceFCL::computeDistance(const std::shared_ptr<base::State> q, bool compute_again)
{
	if (!compute_again && q->getDistance() > 0)
		return q->getDistance();

	robot->setState(q);
	float d_c = INFINITY;
	std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points = std::make_shared<std::vector<Eigen::MatrixXf>>
		(std::vector<Eigen::MatrixXf>(env->getParts().size(), Eigen::MatrixXf(6, robot->getParts().size())));
	std::shared_ptr<Eigen::MatrixXf> nearest_pts = std::make_shared<Eigen::MatrixXf>(3, 2);
	fcl::DefaultDistanceData<float> distance_data;
	
	for (size_t i = 0; i < robot->getParts().size(); i++)
	{	
		for (size_t j = 0; j < env->getParts().size(); j++)
		{
			// 'j == 0' always represents the table when it is included
			if ((j == 0 && (i == 0 || i == 1)) && robot->getType() == "xarm6_with_table")
			{
				nearest_pts->col(0) << 0, 0, 0; 			// Robot nearest point
				nearest_pts->col(1) << 0, 0, -INFINITY;		// Obstacle nearest point
			}
			else
			{
				collision_manager_robot->clear();
				collision_manager_env->clear();
				collision_manager_robot->registerObject(robot->getParts()[i].get());
				collision_manager_env->registerObject(env->getParts()[j].get());
				collision_manager_robot->setup();
				collision_manager_env->setup();

				distance_data.request.enable_nearest_points = true;
				distance_data.result.clear();
				collision_manager_robot->distance(collision_manager_env.get(), &distance_data, fcl::DefaultDistanceFunction);
				d_c = std::min(d_c, distance_data.result.min_distance);
				nearest_pts->col(0) = distance_data.result.nearest_points[0];
				nearest_pts->col(1) = distance_data.result.nearest_points[1];

				// LOG(INFO) << "(i, j) = (" << i << ", " << j << ")" << std::endl;
				// LOG(INFO) << "d_c = " << distance_data.result.min_distance << std::endl;
				// LOG(INFO) << "NP robot: " << nearest_pts->col(0).transpose() << std::endl;
				// LOG(INFO) << "NP env:   " << nearest_pts->col(1).transpose() << std::endl;
			}

			if (d_c <= 0)		// The collision occurs
            {
				q->setDistance(0);
				q->setNearestPoints(nullptr);
				return 0;
			}
			
			// 'nearest_pts->col(0)' is robot nearest point, and 'nearest_pts->col(1)' is obstacle nearest point
			nearest_points->at(j).col(i) << nearest_pts->col(0), nearest_pts->col(1);
		}
	}
	
	q->setDistance(d_c);
	q->setNearestPoints(nearest_points);
	return d_c;
}
