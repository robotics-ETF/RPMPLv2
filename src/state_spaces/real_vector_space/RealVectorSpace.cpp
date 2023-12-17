//
// Created by dinko on 07.03.21.
// Modified by nermin on 07.03.22.
//

#include "RealVectorSpace.h"
#include "RealVectorSpaceConfig.h"
#include "xArm6.h"

base::RealVectorSpace::RealVectorSpace(int num_dimensions_) : num_dimensions(num_dimensions_)
{
	setStateSpaceType(StateSpaceType::RealVectorSpace);
}

base::RealVectorSpace::RealVectorSpace(int num_dimensions_, const std::shared_ptr<robots::AbstractRobot> robot_, 
									   const std::shared_ptr<env::Environment> env_) : num_dimensions(num_dimensions_)
{
	srand((unsigned int) time(0));
	setStateSpaceType(StateSpaceType::RealVectorSpace);
	robot = robot_;
	env = env_;
}

base::RealVectorSpace::~RealVectorSpace() {}

std::ostream &base::operator<<(std::ostream &os, const base::RealVectorSpace &space)
{
	os << " Num. of dimensions: " << space.num_dimensions;
	return os;
}

// Get a random state with uniform distribution, which is limited with robot joint limits
// If 'q_center' is passed, it is added to the random state 
std::shared_ptr<base::State> base::RealVectorSpace::getRandomState(const std::shared_ptr<base::State> q_center)
{
	Eigen::VectorXf rand = Eigen::VectorXf::Random(num_dimensions);
	std::vector<std::vector<float>> limits = robot->getLimits();
	for (size_t i = 0; i < num_dimensions; ++i)
		rand(i) = ((limits[i][1] - limits[i][0]) * rand(i) + limits[i][0] + limits[i][1]) / 2;

	if (q_center != nullptr)
		rand += q_center->getCoord();

	// std::cout << "Random state coord: " << rand.transpose();
	return std::make_shared<base::RealVectorSpaceState>(rand);
}

// Get a copy of 'state'
std::shared_ptr<base::State> base::RealVectorSpace::getNewState(const std::shared_ptr<base::State> state)
{
	return std::make_shared<base::RealVectorSpaceState>(state);
}

// Get completely a new state with same coordinates as 'state'
std::shared_ptr<base::State> base::RealVectorSpace::getNewState(const Eigen::VectorXf &coord)
{
	return std::make_shared<base::RealVectorSpaceState>(coord);
}

// Get Euclidean distance between two states (get norm of the vector 'q2 - q1')
inline float base::RealVectorSpace::getNorm(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2)
{
	return (q1->getCoord() - q2->getCoord()).norm();
}

// Check if two states are equal
bool base::RealVectorSpace::isEqual(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2)
{
	if (getNorm(q1, q2) < RealVectorSpaceConfig::EQUALITY_THRESHOLD)
		return true;
	
	return false;
}

// Interpolate edge from 'q1' to 'q2' for step 'step'
// 'dist' (optional parameter) is the distance between 'q1' and 'q2'
// Return new state
std::shared_ptr<base::State> base::RealVectorSpace::interpolateEdge
	(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, float step, float dist)
{
	if (dist < 0) 	// 'dist' = -1 is default value
		dist = getNorm(q1, q2);
	
	Eigen::VectorXf q_new_coord;
	if (dist > 0)
		q_new_coord = q1->getCoord() + (q2->getCoord() - q1->getCoord()) * (step / dist);
	else
		q_new_coord = q2->getCoord();

	return std::make_shared<base::RealVectorSpaceState>(q_new_coord);
}

// Interpolate edge from 'q1' to 'q2' for step 'step'
// 'dist' (optional parameter) is the distance between 'q1' and 'q2'
// Return status of interpolation (Advanced or Reached), and new state
std::tuple<base::State::Status, std::shared_ptr<base::State>> base::RealVectorSpace::interpolateEdge2
	(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, float step, float dist)
{
	if (dist < 0) 	// 'dist' = -1 is default value
		dist = getNorm(q1, q2);
	
	std::shared_ptr<base::State> q_new;
	base::State::Status status;
	if (step < dist)
	{
		q_new = interpolateEdge(q1, q2, step, dist);
		status = base::State::Status::Advanced;
	}
	else
	{
		q_new = getNewState(q2);
		status = base::State::Status::Reached;
	}

	return {status, q_new};
}

// Prune edge from 'q1' to 'q2', if it comes out of C-space domain or specified 'limits_'.
// Return result state: 'q_new' if there is prunning, and 'q2' if not.
// If 'limits_' are not passed, robot joint limits are used as deafult.
std::shared_ptr<base::State> base::RealVectorSpace::pruneEdge(const std::shared_ptr<base::State> q1, 
	const std::shared_ptr<base::State> q2, const std::vector<std::vector<float>> &limits_)
{
	std::vector<float> bounds(num_dimensions);
	std::vector<int> indices;
	std::vector<std::vector<float>> limits = (limits_.empty()) ? robot->getLimits() : limits_;
	for (int k = 0; k < num_dimensions; k++)
	{
		if (q2->getCoord(k) > limits[k][1])
		{
			bounds[k] = limits[k][1];
			indices.push_back(k);
		}
		else if (q2->getCoord(k) < limits[k][0])
		{
			bounds[k] = limits[k][0];
			indices.push_back(k);
		}
	}

	float t;
	Eigen::VectorXf q_new_coord;
	bool found;
	for (int idx : indices)
	{
		t = (bounds[idx] - q1->getCoord(idx)) / (q2->getCoord(idx) - q1->getCoord(idx));
		q_new_coord = q1->getCoord() + t * (q2->getCoord() - q1->getCoord());

		found = true;
		for (int k = 0; k < num_dimensions; k++)
		{
			if (q_new_coord(k) < limits[k][0] - RealVectorSpaceConfig::EQUALITY_THRESHOLD || 
				q_new_coord(k) > limits[k][1] + RealVectorSpaceConfig::EQUALITY_THRESHOLD)
			{
				found = false;
				break;
			}
		}
		if (found)
			return std::make_shared<base::RealVectorSpaceState>(q_new_coord);
	}

	return q2;
}

bool base::RealVectorSpace::isValid(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2)
{
	int num_checks = RealVectorSpaceConfig::NUM_INTERPOLATION_VALIDITY_CHECKS;
	float dist = getNorm(q1, q2);
	std::shared_ptr<base::State> q_new;
	for (float k = num_checks; k >= 1; k--)
	{
		q_new = interpolateEdge(q1, q2, k / num_checks * dist, dist);
		if (!isValid(q_new))
			return false;
	}
	return true;
}

bool base::RealVectorSpace::isValid(const std::shared_ptr<base::State> q)
{
	bool collision;
	std::shared_ptr<Eigen::MatrixXf> skeleton = robot->computeSkeleton(q);
	
	for (int i = 0; i < robot->getParts().size(); i++)
	{
    	for (int j = 0; j < env->getParts().size(); j++)
		{
			// 'j == 0' always represents the table when it is included
			if ((j == 0 && (i == 0 || i == 1)) && robot->getType() == "xarm6_with_table")
				continue;
            else if (env->getParts()[j]->getNodeType() == fcl::NODE_TYPE::GEOM_BOX)
			{
				// std::cout << "(i,j) = (" <<i<<","<<j<<")" << std::endl;
				// std::cout << "r(i): " << robot->getCapsuleRadius(i) << std::endl;
				// std::cout << "skeleton(i):   " << skeleton->col(i).transpose() << std::endl;
				// std::cout << "skeleton(i+1): " << skeleton->col(i+1).transpose() << std::endl;
				fcl::AABB AABB = env->getParts()[j]->getAABB();
				Eigen::VectorXf obs(6);
				obs << AABB.min_[0], AABB.min_[1], AABB.min_[2], AABB.max_[0], AABB.max_[1], AABB.max_[2];
				if (collisionCapsuleToBox(skeleton->col(i), skeleton->col(i+1), robot->getCapsuleRadius(i), obs))
					return false;
            }
			else if (env->getParts()[j]->getNodeType() == fcl::NODE_TYPE::GEOM_SPHERE)
			{
				Eigen::VectorXf obs(4); 	// TODO
                if (collisionCapsuleToSphere(skeleton->col(i), skeleton->col(i+1), robot->getCapsuleRadius(i), obs))
					return false;
            }
        }
    }
    return true;
}

// Return minimal distance from robot in configuration 'q' to obstacles
// Compute minimal distance from each robot's link in configuration 'q' to obstacles, i.e., compute distance profile function
// Moreover, set 'd_c', 'd_c_profile', and corresponding 'nearest_points' for the configuation 'q'
// If 'compute_again' is true, the new distance profile will be computed again!
float base::RealVectorSpace::computeDistance(const std::shared_ptr<base::State> q, bool compute_again)
{
	if (!compute_again && q->getDistance() > 0 && q->getIsRealDistance())
		return q->getDistance();

	float d_c_temp;
	float d_c = INFINITY;
	std::vector<float> d_c_profile(robot->getParts().size(), 0);
	std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points = std::make_shared<std::vector<Eigen::MatrixXf>>
		(std::vector<Eigen::MatrixXf>(env->getParts().size(), Eigen::MatrixXf(6, robot->getParts().size())));
	std::shared_ptr<Eigen::MatrixXf> nearest_pts = std::make_shared<Eigen::MatrixXf>(3, 2);
	std::shared_ptr<Eigen::MatrixXf> skeleton = robot->computeSkeleton(q);

	for (int i = 0; i < robot->getParts().size(); i++)
	{
		d_c_profile[i] = INFINITY;
    	for (int j = 0; j < env->getParts().size(); j++)
		{
			// 'j == 0' always represents the table when it is included
			if ((j == 0 && (i == 0 || i == 1)) && robot->getType() == "xarm6_with_table")
			{
				d_c_temp = INFINITY;
				nearest_pts->col(0) << 0, 0, 0; 			// Robot nearest point
				nearest_pts->col(1) << 0, 0, -INFINITY;		// Obstacle nearest point
			}
            else if (env->getParts()[j]->getNodeType() == fcl::NODE_TYPE::GEOM_BOX)
			{
				fcl::AABB AABB = env->getParts()[j]->getAABB();
				Eigen::VectorXf obs(6);
				obs << AABB.min_[0], AABB.min_[1], AABB.min_[2], AABB.max_[0], AABB.max_[1], AABB.max_[2];
                tie(d_c_temp, nearest_pts) = distanceCapsuleToBox(skeleton->col(i), skeleton->col(i+1), robot->getCapsuleRadius(i), obs);
				
				// std::cout << "(i, j) = (" << i << ", " << j << "). " << std::endl;
				// std::cout << "Distance:    " << d_c_temp << std::endl;
				// // float dQP;
				// // std::shared_ptr<Eigen::MatrixXf> nearest_ptsQP;
                // // tie(dQP, nearest_ptsQP) = distanceCapsuleToBoxQP(skeleton->col(i), skeleton->col(i+1), robot->getCapsuleRadius(i), obs);
				// // std::cout << "Distance QP: " << dQP << std::endl;
				// // if (std::abs(d_c_temp - dQP) > 1e-3)
				// // 	std::cout << "****************************** DIFFERENT *************************************" << std::endl;
				// if (nearest_pts != nullptr)
				// {
				// 	std::cout << "Nearest point link:    " << nearest_pts->col(0).transpose() << std::endl;
				// 	// std::cout << "Nearest point link QP: " << nearest_ptsQP->col(0).transpose() << std::endl;
				// 	std::cout << "Nearest point obs:     " << nearest_pts->col(1).transpose() << std::endl;
				// 	// std::cout << "Nearest point obs QP:  " << nearest_ptsQP->col(1).transpose() << std::endl;
				// }
				// std::cout << "r(i): " << robot->getCapsuleRadius(i) << std::endl;
				// std::cout << "skeleton(i):   " << skeleton->col(i).transpose() << std::endl;
				// std::cout << "skeleton(i+1): " << skeleton->col(i+1).transpose() << std::endl;
				// std::cout << "-------------------------------------------------------------" << std::endl;
            }
			else if (env->getParts()[j]->getNodeType() == fcl::NODE_TYPE::GEOM_SPHERE)
			{
				Eigen::VectorXf obs(4); 	// TODO
                tie(d_c_temp, nearest_pts) = distanceCapsuleToSphere(skeleton->col(i), skeleton->col(i+1), robot->getCapsuleRadius(i), obs);
            }

			d_c_profile[i] = std::min(d_c_profile[i], d_c_temp);
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

// Return the underestimation of distance-to-obstacles 'd_c', i.e. return the distance-to-planes, 
// Compute the underestimation of distance-to-obstacles 'd_c' for each robot's link, 
// i.e. compute the distance-to-planes profile function, when robot is in the configuration 'q', 
// where planes approximate obstacles, and are generated according to 'nearest_points'
float base::RealVectorSpace::computeDistanceUnderestimation(const std::shared_ptr<base::State> q, 
	const std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points)
{
	if (q->getDistance() > 0 && q->getIsRealDistance()) 	// Real distance was already computed
		return q->getDistance();
	
	float d_c_temp;
    float d_c = INFINITY;
	std::vector<float> d_c_profile(robot->getParts().size(), 0);
    Eigen::Vector3f R, O;    // 'R' is robot nearest point, and 'O' is obstacle nearest point
	std::shared_ptr<Eigen::MatrixXf> skeleton = robot->computeSkeleton(q);
    
    for (int i = 0; i < robot->getParts().size(); i++)
    {
		d_c_profile[i] = INFINITY;
        for (int j = 0; j < env->getParts().size(); j++)
        {
            O = nearest_points->at(j).col(i).tail(3);
			if (O.norm() < INFINITY)
			{
				R = nearest_points->at(j).col(i).head(3);
				d_c_temp = std::min(std::abs((R - O).dot(skeleton->col(i) - O)) / (R - O).norm(), 
									std::abs((R - O).dot(skeleton->col(i+1) - O)) / (R - O).norm()) 
									- robot->getCapsuleRadius(i);
				d_c_profile[i] = std::min(d_c_profile[i], d_c_temp);

				// std::cout << "(i, j) = " << "(" << i << ", " << j << "):" << std::endl;
				// std::cout << "Robot nearest point:    " << R.transpose() << std::endl;
				// std::cout << "Obstacle nearest point: " << O.transpose() << std::endl;
				// std::cout << "d_c: " << d_c_profile[i] << std::endl;
			}
		}
		d_c = std::min(d_c, d_c_profile[i]);
    }

	if (d_c > q->getDistance())		// Also, if it was previously computed (q->getDistance() > 0), take "better" (greater) one
	{
		q->setDistance(d_c);
		q->setDistanceProfile(d_c_profile);
		q->setIsRealDistance(false);
	}

	return q->getDistance();
}
