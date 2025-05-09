#include "Spatial10DOF.h"

robots::Spatial10DOF::~Spatial10DOF() {}

robots::Spatial10DOF::Spatial10DOF(const std::string &robot_desc, size_t ground_included_, size_t num_DOFs_) : 
	Planar2DOF(robot_desc, num_DOFs_) 
{
	ground_included = ground_included_;
}

std::shared_ptr<Eigen::MatrixXf> robots::Spatial10DOF::computeEnclosingRadii(const std::shared_ptr<base::State> &q)
{
	if (q->getEnclosingRadii() != nullptr)	// It has been already computed!
		return q->getEnclosingRadii();

	std::shared_ptr<Eigen::MatrixXf> skeleton { computeSkeleton(q) };
	Eigen::MatrixXf R { Eigen::MatrixXf::Zero(num_DOFs, num_DOFs+1) };

	for (size_t j = 1; j <= num_DOFs; j++)	// Final point on skeleton
		R(0, j) = std::max(R(0, j-1), skeleton->col(j).head(2).norm() + capsules_radius[j-1]);

	for (size_t i = 1; i < num_DOFs; i++) 			// Starting point on skeleton
	{
		for (size_t j = i+1; j <= num_DOFs; j++)	// Final point on skeleton
			R(i, j) = std::max(R(i, j-1), (skeleton->col(j) - skeleton->col(i)).norm() + capsules_radius[j-1]);
	}

	q->setEnclosingRadii(std::make_shared<Eigen::MatrixXf>(R));
	return q->getEnclosingRadii();
}

bool robots::Spatial10DOF::checkSelfCollision([[maybe_unused]] const std::shared_ptr<base::State> &q1, 
											  [[maybe_unused]] std::shared_ptr<base::State> &q2)
{
    // TODO (if needed)
	return false;
}

bool robots::Spatial10DOF::checkSelfCollision([[maybe_unused]] const std::shared_ptr<base::State> &q)
{
	// TODO (if needed)
	return false;
}
