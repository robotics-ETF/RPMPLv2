#include "RealVectorSpace.h"

base::RealVectorSpace::RealVectorSpace(size_t num_dimensions_) : StateSpace(num_dimensions_)
{
	setStateSpaceType(base::StateSpaceType::RealVectorSpace);
}

base::RealVectorSpace::RealVectorSpace(size_t num_dimensions_, const std::shared_ptr<robots::AbstractRobot> robot_, 
	const std::shared_ptr<env::Environment> env_) : StateSpace(num_dimensions_, robot_, env_)	
{
	setStateSpaceType(base::StateSpaceType::RealVectorSpace);
}

base::RealVectorSpace::~RealVectorSpace() {}

namespace base 
{
	std::ostream &operator<<(std::ostream &os, const base::RealVectorSpace &space)
	{
		os << " Num. of dimensions: " << space.num_dimensions;
		return os;
	}
}

// Get a random state with uniform distribution, which is limited by robot joint limits
// If 'q_center' is passed, it is added to the random state 
std::shared_ptr<base::State> base::RealVectorSpace::getRandomState(const std::shared_ptr<base::State> q_center)
{
	Eigen::VectorXf q_rand_coord { Eigen::VectorXf::Random(num_dimensions) };
	std::vector<std::pair<float, float>> limits { robot->getLimits() };

	for (size_t i = 0; i < num_dimensions; i++)
		q_rand_coord(i) = ((limits[i].second - limits[i].first) * q_rand_coord(i) + limits[i].first + limits[i].second) / 2;

	if (q_center != nullptr)
		q_rand_coord += q_center->getCoord();

	// std::cout << "Random state coord: " << q_rand_coord.transpose();
	return getNewState(q_rand_coord);
}

// Get completely a new state with the same coordinates as 'state'
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

// Check if two vectors are equal
bool base::RealVectorSpace::isEqual(const Eigen::VectorXf &q1_coord, const Eigen::VectorXf &q2_coord)
{
	if ((q1_coord - q2_coord).norm() < RealVectorSpaceConfig::EQUALITY_THRESHOLD)
		return true;
	
	return false;
}

// Interpolate edge from 'q1' to 'q2' for step 'step'
// 'dist' (optional parameter) is the distance between 'q1' and 'q2'
// Return a new state
std::shared_ptr<base::State> base::RealVectorSpace::interpolateEdge
	(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, float step, float dist)
{
	if (dist < 0) 	// 'dist' = -1 is default value
		dist = getNorm(q1, q2);
	
	Eigen::VectorXf q_new_coord {};
	if (dist > 0)
		q_new_coord = q1->getCoord() + (q2->getCoord() - q1->getCoord()) * (step / dist);
	else
		q_new_coord = q2->getCoord();

	return getNewState(q_new_coord);
}

// Interpolate edge from 'q1' to 'q2' for step 'step'
// 'dist' (optional parameter) is the distance between 'q1' and 'q2'
// Return a status of interpolation (Advanced or Reached), and a new state
std::tuple<base::State::Status, std::shared_ptr<base::State>> base::RealVectorSpace::interpolateEdge2
	(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, float step, float dist)
{
	if (dist < 0) 	// 'dist' = -1 is default value
		dist = getNorm(q1, q2);
	
	std::shared_ptr<base::State> q_new { nullptr };
	base::State::Status status { base::State::Status::None };
	
	if (step < dist)
	{
		q_new = interpolateEdge(q1, q2, step, dist);
		status = base::State::Status::Advanced;
	}
	else
	{
		q_new = getNewState(q2->getCoord());
		status = base::State::Status::Reached;
	}

	return { status, q_new };
}

// Prune the edge from 'q1' to 'q2', if it comes out of C-space domain or specified 'limits_'.
// Return a result state: 'q_new' if there is prunning, and 'q2' if not.
// If 'limits_' are not passed, robot joint limits are used as deafult.
std::shared_ptr<base::State> base::RealVectorSpace::pruneEdge(const std::shared_ptr<base::State> q1, 
	const std::shared_ptr<base::State> q2, const std::vector<std::pair<float, float>> &limits_)
{
	std::vector<float> bounds(num_dimensions);
	std::vector<size_t> indices {};
	std::vector<std::pair<float, float>> limits { (limits_.empty()) ? robot->getLimits() : limits_ };

	for (size_t k = 0; k < num_dimensions; k++)
	{
		if (q2->getCoord(k) > limits[k].second)
		{
			bounds[k] = limits[k].second;
			indices.push_back(k);
		}
		else if (q2->getCoord(k) < limits[k].first)
		{
			bounds[k] = limits[k].first;
			indices.push_back(k);
		}
	}

	float t { 0 };
	Eigen::VectorXf q_new_coord {};
	bool found { false };

	for (size_t idx : indices)
	{
		t = (bounds[idx] - q1->getCoord(idx)) / (q2->getCoord(idx) - q1->getCoord(idx));
		q_new_coord = q1->getCoord() + t * (q2->getCoord() - q1->getCoord());

		found = true;
		for (size_t k = 0; k < num_dimensions; k++)
		{
			if (q_new_coord(k) < limits[k].first - RealVectorSpaceConfig::EQUALITY_THRESHOLD || 
				q_new_coord(k) > limits[k].second + RealVectorSpaceConfig::EQUALITY_THRESHOLD)
			{
				found = false;
				break;
			}
		}
		if (found)
			return getNewState(q_new_coord);
	}

	return q2;
}

// Prune the edge from 'q1' to 'q2', where 'q1' is the center of a box which all dimensions are the same and equal to '2*max_edge_length'.
// Return a result state: 'q_new' if there is prunning, and 'q2' if not.
std::shared_ptr<base::State> base::RealVectorSpace::pruneEdge2(const std::shared_ptr<base::State> q1, 
	const std::shared_ptr<base::State> q2, float max_edge_length)
{
	Eigen::VectorXf::Index idx {};
	float delta_q12_max { (q2->getCoord() - q1->getCoord()).cwiseAbs().maxCoeff(&idx) };
	
	if (delta_q12_max > max_edge_length)
	{
		int sign { (q2->getCoord(idx) - q1->getCoord(idx) > 0) ? 1 : -1 };
		float limit { q1->getCoord(idx) + sign * max_edge_length };
		float t { (limit - q1->getCoord(idx)) / (q2->getCoord(idx) - q1->getCoord(idx)) };		
		return getNewState(q1->getCoord() + t * (q2->getCoord() - q1->getCoord()));
	}

	return q2;
}

/// @brief Generate a new path 'new_path' from a path 'original_path' in a way that the distance between two adjacent nodes
/// is fixed (if possible) to a length of 'max_edge_length'. Geometrically, the new path remains the same as the original one,
/// but only their nodes may differ.
/// @param original_path Original path that will be transformed.
/// @param new_path New resulting path.
/// @param max_edge_length Maximal edge length.
void base::RealVectorSpace::preprocessPath(const std::vector<std::shared_ptr<base::State>> &original_path, 
    std::vector<std::shared_ptr<base::State>> &new_path, float max_edge_length)
{
	std::vector<std::shared_ptr<base::State>> init_path { original_path.front() };
	for (size_t i = 1; i < original_path.size(); i++)
	{
		if (!isEqual(original_path[i], original_path[i-1]))
			init_path.emplace_back(original_path[i]);
	}
	
	std::vector<std::shared_ptr<base::State>> path { init_path.front() };
    std::shared_ptr<base::State> q0 { nullptr };
    std::shared_ptr<base::State> q1 { nullptr };
    std::shared_ptr<base::State> q2 { nullptr };

	// std::cout << "Initial path is: \n";
    // for (size_t i = 0; i < init_path.size(); i++)
    //     std::cout << init_path[i]->getCoord().transpose() << "\n";
    // std::cout << std::endl;

	for (size_t i = 1; i < init_path.size() - 1; i++)
	{
        q0 = init_path[i-1];
        q1 = init_path[i];
		q2 = init_path[i+1];

		for (size_t k = 1; k < num_dimensions; k++)
		{
			if (std::abs((q2->getCoord(k) - q1->getCoord(k)) / (q1->getCoord(k) - q0->getCoord(k)) - 
						 (q2->getCoord(k-1) - q1->getCoord(k-1)) / (q1->getCoord(k-1) - q0->getCoord(k-1))) > 
				RealVectorSpaceConfig::EQUALITY_THRESHOLD)
			{
				path.emplace_back(q1);
				break;
			}
		}
	}
	path.emplace_back(init_path.back());

	// std::cout << "Modified path is: \n";
    // for (size_t i = 0; i < path.size(); i++)
    //     std::cout << path[i]->getCoord().transpose() << "\n";
    // std::cout << std::endl;

    new_path.clear();
    new_path.emplace_back(init_path.front());
    base::State::Status status { base::State::Status::None };
    float dist {};

    for (size_t i = 1; i < path.size(); i++)
    {
        status = base::State::Status::Advanced;
        q0 = path[i-1];
        q1 = path[i];

        while (status == base::State::Status::Advanced)
        {
			dist = getNorm(q0, q1);
            if (dist > max_edge_length)
            {
				q0 = interpolateEdge(q0, q1, max_edge_length, dist);
                status = base::State::Status::Advanced;
            }
			else
			{
				q0 = q1;
                status = base::State::Status::Reached;
			}
            new_path.emplace_back(q0);
        }
    }

    // std::cout << "Preprocessed path is: \n";
    // for (size_t i = 0; i < new_path.size(); i++)
    //     std::cout << new_path[i]->getCoord().transpose() << "\n";
    // std::cout << std::endl;
}

bool base::RealVectorSpace::isValid(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2)
{
	size_t num_checks { RealVectorSpaceConfig::NUM_INTERPOLATION_VALIDITY_CHECKS };
	float dist { getNorm(q1, q2) };
	std::shared_ptr<base::State> q_new { nullptr };

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
	std::shared_ptr<Eigen::MatrixXf> skeleton { robot->computeSkeleton(q) };
	
	for (size_t i = 0; i < robot->getNumLinks(); i++)
	{
    	for (size_t j = 0; j < env->getNumObjects(); j++)
		{
			if (env->getObject(j)->getLabel() == "ground" && i < robot->getGroundIncluded())
				continue;
            else if (env->getCollObject(j)->getNodeType() == fcl::NODE_TYPE::GEOM_BOX)
			{
				// std::cout << "(i,j) = (" <<i<<","<<j<<")" << std::endl;
				// std::cout << "r(i): " << robot->getCapsuleRadius(i) << std::endl;
				// std::cout << "skeleton(i):   " << skeleton->col(i).transpose() << std::endl;
				// std::cout << "skeleton(i+1): " << skeleton->col(i+1).transpose() << std::endl;
				fcl::AABB AABB { env->getCollObject(j)->getAABB() };
				Eigen::VectorXf obs(6);
				obs << AABB.min_[0], AABB.min_[1], AABB.min_[2], AABB.max_[0], AABB.max_[1], AABB.max_[2];
				if (collisionCapsuleToBox(skeleton->col(i), skeleton->col(i+1), robot->getCapsuleRadius(i), obs))
					return false;
            }
			else if (env->getCollObject(j)->getNodeType() == fcl::NODE_TYPE::GEOM_SPHERE)
			{
				Eigen::VectorXf obs(4); 	// TODO
                if (collisionCapsuleToSphere(skeleton->col(i), skeleton->col(i+1), robot->getCapsuleRadius(i), obs))
					return false;
            }
        }
    }

    return true;
}

/// @brief Compute a minimal distance from the robot in configuration 'q' to obstacles using methods from 'CollisionAndDistance' class.
/// In other words, compute a minimal distance from each robot's link in configuration 'q' to obstacles, 
/// i.e., compute a distance profile function. 
/// Moreover, set 'd_c', 'd_c_profile', and corresponding 'nearest_points' for the configuation 'q'.
/// @param q Configuration of the robot.
/// @param compute_again If true, a new distance profile will be computed again! Default: false.
/// @return Minimal distance from the robot in configuration 'q' to obstacles.
float base::RealVectorSpace::computeDistance(const std::shared_ptr<base::State> q, bool compute_again)
{
	if (!compute_again && q->getDistance() > 0 && q->getIsRealDistance())
		return q->getDistance();

	float d_c_temp {};
	float d_c { INFINITY };
	std::vector<float> d_c_profile(robot->getNumLinks(), 0);
	std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points { std::make_shared<std::vector<Eigen::MatrixXf>>
		(env->getNumObjects(), Eigen::MatrixXf(6, robot->getNumLinks())) };
	std::shared_ptr<Eigen::MatrixXf> nearest_pts { std::make_shared<Eigen::MatrixXf>(3, 2) };
	std::shared_ptr<Eigen::MatrixXf> skeleton { robot->computeSkeleton(q) };

	for (size_t i = 0; i < robot->getNumLinks(); i++)
	{
		d_c_profile[i] = INFINITY;
    	for (size_t j = 0; j < env->getNumObjects(); j++)
		{
			if (env->getObject(j)->getLabel() == "ground" && i < robot->getGroundIncluded())
			{
				d_c_temp = INFINITY;
				nearest_pts->col(0) << 0, 0, 0; 			// Robot nearest point
				nearest_pts->col(1) << 0, 0, -INFINITY;		// Obstacle nearest point
			}
            else if (env->getCollObject(j)->getNodeType() == fcl::NODE_TYPE::GEOM_BOX)
			{
				fcl::AABB AABB { env->getCollObject(j)->getAABB() };
				Eigen::VectorXf obs(6);
				obs << AABB.min_[0], AABB.min_[1], AABB.min_[2], AABB.max_[0], AABB.max_[1], AABB.max_[2];
                tie(d_c_temp, nearest_pts) = distanceCapsuleToBox(skeleton->col(i), skeleton->col(i+1), robot->getCapsuleRadius(i), obs);
				
				// std::cout << "(i, j) = (" << i << ", " << j << "). " << std::endl;
				// std::cout << "Distance:    " << d_c_temp << std::endl;
				// if (nearest_pts != nullptr)
				// {
				// 	std::cout << "Nearest point link:    " << nearest_pts->col(0).transpose() << std::endl;
				// 	std::cout << "Nearest point obs:     " << nearest_pts->col(1).transpose() << std::endl;
				// }
				// std::cout << "r(i): " << robot->getCapsuleRadius(i) << std::endl;
				// std::cout << "skeleton(i):   " << skeleton->col(i).transpose() << std::endl;
				// std::cout << "skeleton(i+1): " << skeleton->col(i+1).transpose() << std::endl;
				// std::cout << "-------------------------------------------------------------" << std::endl;
            }
			else if (env->getCollObject(j)->getNodeType() == fcl::NODE_TYPE::GEOM_SPHERE)
			{
				Eigen::VectorXf obs(4); 	// TODO
                tie(d_c_temp, nearest_pts) = distanceCapsuleToSphere(skeleton->col(i), skeleton->col(i+1), robot->getCapsuleRadius(i), obs);
            }

			if (d_c_temp > env->getObject(j)->getMinDistTol())
				d_c_temp = INFINITY;

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

/// @brief Compute an underestimation of distance-to-obstacles 'd_c', i.e., a distance-to-planes, for each robot's link,
/// i.e., compute a distance-to-planes profile function, when the robot takes a configuration 'q'.
/// Planes approximate obstacles, and are generated according to 'nearest_points'.
/// @param q Configuration of the robot.
/// @param nearest_points Nearest points between the robot and obstacles.
/// @return Underestimation of distance-to-obstacles.
/// Note that if 'd_c' is negative, it means that one or more robot's links are penetrating through the plane,
/// or they are located on the other side of the plane.
float base::RealVectorSpace::computeDistanceUnderestimation(const std::shared_ptr<base::State> q, 
	const std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points)
{
	if (q->getDistance() > 0 && q->getIsRealDistance()) 	// Real distance was already computed
		return q->getDistance();
	
	float d_c_temp {};
    float d_c { INFINITY };
	std::vector<float> d_c_profile(robot->getNumLinks(), 0);
    Eigen::Vector3f R {};		// Robot's nearest point
	Eigen::Vector3f O {};    	// Obstacle's nearest point
	std::shared_ptr<Eigen::MatrixXf> skeleton { robot->computeSkeleton(q) };
    
    for (size_t i = 0; i < robot->getNumLinks(); i++)
    {
		d_c_profile[i] = INFINITY;
        for (size_t j = 0; j < env->getNumObjects(); j++)
        {
            O = nearest_points->at(j).col(i).tail(3);
			if (O.norm() == INFINITY)
				continue;
			
			R = nearest_points->at(j).col(i).head(3);
			d_c_temp = std::min((skeleton->col(i) - O).dot((R - O).normalized()), 
								(skeleton->col(i+1) - O).dot((R - O).normalized())) 
								- robot->getCapsuleRadius(i);
			if (d_c_temp < 0)
				return 0;

			d_c_profile[i] = std::min(d_c_profile[i], d_c_temp);

			// std::cout << "(i, j) = " << "(" << i << ", " << j << "):" << std::endl;
			// std::cout << "Robot nearest point:    " << R.transpose() << std::endl;
			// std::cout << "Obstacle nearest point: " << O.transpose() << std::endl;
			// std::cout << "d_c: " << d_c_profile[i] << std::endl;
		}
		d_c = std::min(d_c, d_c_profile[i]);
    }

	q->setDistance(d_c);
	q->setDistanceProfile(d_c_profile);
	q->setIsRealDistance(false);

	return d_c;
}
