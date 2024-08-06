//
// Created by dinko on 7.3.21.
// Modified by nermin on 07.03.22.
//

#ifndef RPMPL_REALVECTORSPACE_H
#define RPMPL_REALVECTORSPACE_H

#include "StateSpace.h"
#include "RealVectorSpaceState.h"
#include "CollisionAndDistance.h"
#include "RealVectorSpaceConfig.h"
#include "xArm6.h"

namespace base
{
	class RealVectorSpace : public base::StateSpace,
							public base::CollisionAndDistance
	{
	public:
		RealVectorSpace(size_t num_dimensions_);
		RealVectorSpace(size_t num_dimensions_, const std::shared_ptr<robots::AbstractRobot> robot_, 
						const std::shared_ptr<env::Environment> env_);
		virtual ~RealVectorSpace();

		std::shared_ptr<base::State> getRandomState(const std::shared_ptr<base::State> q_center) override;
		std::shared_ptr<base::State> getNewState(const std::shared_ptr<base::State> q) override;
		std::shared_ptr<base::State> getNewState(const Eigen::VectorXf &coord) override;
		
		float getNorm(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) override;
		bool isEqual(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) override;
		bool isEqual(const Eigen::VectorXf &q1_coord, const Eigen::VectorXf &q2_coord) override;
		std::shared_ptr<base::State> interpolateEdge
			(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, float step, float dist) override;
		std::tuple<base::State::Status, std::shared_ptr<base::State>> interpolateEdge2
			(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, float step, float dist) override;
		std::shared_ptr<base::State> pruneEdge(const std::shared_ptr<base::State> q1, 
			const std::shared_ptr<base::State> q2, const std::vector<std::pair<float, float>> &limits_) override;
		std::shared_ptr<base::State> pruneEdge2(const std::shared_ptr<base::State> q1, 
			const std::shared_ptr<base::State> q2, float max_edge_length) override;
		void preprocessPath(const std::vector<std::shared_ptr<base::State>> &original_path, 
			std::vector<std::shared_ptr<base::State>> &new_path, float max_edge_length);

		bool isValid(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) override;
		virtual bool isValid(const std::shared_ptr<base::State> q) override;
		virtual float computeDistance(const std::shared_ptr<base::State> q, bool compute_again) override;
		float computeDistanceUnderestimation(const std::shared_ptr<base::State> q, 
			const std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points) override;
			
		friend std::ostream &operator<<(std::ostream &os, const RealVectorSpace &space);
		
	};
}

#endif //RPMPL_REALVECTORSPACE_H