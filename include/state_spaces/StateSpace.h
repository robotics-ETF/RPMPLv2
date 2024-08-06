//
// Created by dinko on 7.3.21.
// Modified by nermin on 18.02.22.
//

#ifndef RPMPL_STATESPACE_H
#define RPMPL_STATESPACE_H

#include "State.h"
#include "StateSpaceType.h"
#include "AbstractRobot.h"
#include "Environment.h"

namespace base
{
	class StateSpace
	{
	public:
		base::StateSpaceType state_space_type;
		size_t num_dimensions;
		std::shared_ptr<robots::AbstractRobot> robot;
		std::shared_ptr<env::Environment> env;

		StateSpace();
		StateSpace(size_t num_dimensions_);
		StateSpace(size_t num_dimensions_, const std::shared_ptr<robots::AbstractRobot> robot_, 
				   const std::shared_ptr<env::Environment> env_);
		virtual ~StateSpace() = 0;
		
		inline void setStateSpaceType(base::StateSpaceType state_space_type_) { state_space_type = state_space_type_; };
		inline size_t getNumDimensions() { return num_dimensions; }
		inline virtual base::StateSpaceType getStateSpaceType() const { return state_space_type; };
		virtual std::shared_ptr<base::State> getRandomState(const std::shared_ptr<base::State> q_center = nullptr) = 0;
		virtual std::shared_ptr<base::State> getNewState(const std::shared_ptr<base::State> q) = 0;
		virtual std::shared_ptr<base::State> getNewState(const Eigen::VectorXf &coord) = 0;

		virtual float getNorm(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) = 0;
		virtual bool isEqual(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) = 0;
		virtual bool isEqual(const Eigen::VectorXf &q1_coord, const Eigen::VectorXf &q2_coord) = 0;
		virtual std::shared_ptr<base::State> interpolateEdge
			(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, float step, float dist = -1) = 0;
		virtual std::tuple<base::State::Status, std::shared_ptr<base::State>> interpolateEdge2
			(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, float step, float dist = -1) = 0;
		virtual std::shared_ptr<base::State> pruneEdge(const std::shared_ptr<base::State> q1, 
			const std::shared_ptr<base::State> q2, const std::vector<std::pair<float, float>> &limits_ = {}) = 0;
		virtual std::shared_ptr<base::State> pruneEdge2(const std::shared_ptr<base::State> q1, 
			const std::shared_ptr<base::State> q2, float max_edge_length) = 0;
		virtual void preprocessPath(const std::vector<std::shared_ptr<base::State>> &original_path, 
			std::vector<std::shared_ptr<base::State>> &new_path, float max_edge_length) = 0;
		
		virtual bool isValid(const std::shared_ptr<base::State> q) = 0;
		virtual bool isValid(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) = 0;
		virtual float computeDistance(const std::shared_ptr<base::State> q, bool compute_again = false) = 0;
		virtual float computeDistanceUnderestimation(const std::shared_ptr<base::State> q, 
			const std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points) = 0;
	};
}

#endif //RPMPL_STATESPACE_H