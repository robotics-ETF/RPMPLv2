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
		StateSpaceType state_space_type;
		std::shared_ptr<robots::AbstractRobot> robot;
		std::shared_ptr<env::Environment> env;

		StateSpace();
		virtual ~StateSpace() = 0;
		
		void setStateSpaceType(StateSpaceType state_space_type_) { state_space_type = state_space_type_; };
		virtual int getNumDimensions() = 0;
		virtual StateSpaceType getStateSpaceType() const { return state_space_type; };
		virtual std::shared_ptr<base::State> getRandomState(const std::shared_ptr<base::State> q_center = nullptr) = 0;
		virtual std::shared_ptr<base::State> getNewState(const std::shared_ptr<base::State> q) = 0;
		virtual std::shared_ptr<base::State> getNewState(const Eigen::VectorXf &coord) = 0;

		virtual float getDistance(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) = 0;
		virtual bool isEqual(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) = 0;
		virtual std::shared_ptr<base::State> interpolateEdge
			(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, float step, float dist = -1) = 0;
		virtual std::tuple<base::State::Status, std::shared_ptr<base::State>> interpolateEdge2
			(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, float step, float dist = -1) = 0;
		virtual std::shared_ptr<base::State> pruneEdge(const std::shared_ptr<base::State> q1, 
			const std::shared_ptr<base::State> q2, const std::vector<std::vector<float>> &limits_ = {}) = 0;
		
		virtual bool isValid(const std::shared_ptr<base::State> q) = 0;
		virtual bool isValid(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) = 0;
		virtual float computeDistance(const std::shared_ptr<base::State> q) = 0;
		virtual float computeDistanceUnderestimation(const std::shared_ptr<base::State> q, 
			const std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points) = 0;
	};
}

#endif //RPMPL_STATESPACE_H