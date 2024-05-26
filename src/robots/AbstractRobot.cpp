//
// Created by dinko on 07.02.22.
// Modified by nermin on 26.05.24.
//

#include "AbstractRobot.h"

robots::AbstractRobot::~AbstractRobot() {}

// Compute step for moving from 'q1' towards 'q2' using ordinary bubble
float robots::AbstractRobot::computeStep(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, float d_c, float rho)
{
	std::shared_ptr<Eigen::MatrixXf> R { computeEnclosingRadii(q1) };
	Eigen::VectorXf r(num_DOFs);

	for (size_t k = 0; k < num_DOFs; k++)
		r(k) = R->row(k).maxCoeff();

	return (d_c - rho) / r.dot((q1->getCoord() - q2->getCoord()).cwiseAbs()); 	// 'd_c - rho' is the remaining path length in W-space
}

// Compute step for moving from 'q1' towards 'q2' using expanded bubble
float robots::AbstractRobot::computeStep2(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, 
	const std::vector<float> &d_c_profile, const std::vector<float> &rho_profile)
{
	std::shared_ptr<Eigen::MatrixXf> R { computeEnclosingRadii(q1) };
	Eigen::VectorXf delta_q { (q2->getCoord() - q1->getCoord()).cwiseAbs() };
	Eigen::VectorXf steps(num_DOFs);
	
	for (size_t n = 0; n < num_DOFs; n++)
	{
		Eigen::VectorXf r(n+1);
		for (size_t k = 0; k < n+1; k++)
			r(k) = R->row(k).head(n+2).maxCoeff();
		
		steps(n) = (d_c_profile[n] - rho_profile[n]) / r.dot(delta_q.head(n+1));
	}
	
	return steps.minCoeff();
}
