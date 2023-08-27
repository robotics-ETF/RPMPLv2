#pragma once
#include "State.h"
#include "StateSpaceType.h"
#include "AbstractRobot.h"
#include "Environment.h"
#include <vector>

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
		
		virtual int getNumDimensions() = 0;
		virtual bool isValid(const std::shared_ptr<base::State> q) = 0;
		virtual bool isValid(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) = 0;
		virtual float computeDistance(const std::shared_ptr<base::State> q) = 0;
		virtual std::tuple<float, std::shared_ptr<std::vector<Eigen::MatrixXf>>> computeDistanceAndNearestPoints(const std::shared_ptr<base::State> q) = 0;
		virtual std::shared_ptr<base::State> randomState(std::shared_ptr<base::State> q_center = nullptr) = 0;
		virtual std::shared_ptr<base::State> newState(std::shared_ptr<base::State> q) = 0;
		virtual std::shared_ptr<base::State> newState(const Eigen::VectorXf &state) = 0;
		virtual StateSpaceType getStateSpaceType() const { return state_space_type; };
		virtual void setStateSpaceType(StateSpaceType state_space_type_) { state_space_type = state_space_type_; };
		virtual std::tuple<base::State::Status, std::shared_ptr<base::State>> interpolate
			(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, float step, float D = -1) = 0;
		virtual bool isEqual(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) = 0;
		
	};
}

