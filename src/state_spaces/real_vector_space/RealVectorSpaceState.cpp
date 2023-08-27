//
// Created by dinko on 7.3.21.
// Modified by nermin on 18.02.22.
//

#include "RealVectorSpaceState.h"

base::RealVectorSpaceState::RealVectorSpaceState(Eigen::VectorXf state_) : State()
{
	setStateSpaceType(StateSpaceType::RealVectorSpace);
	setDimensions(state_.size());
	setCoord(state_);
}

base::RealVectorSpaceState::RealVectorSpaceState(int num_dimensions_) : State()
{
	setStateSpaceType(StateSpaceType::RealVectorSpace);
	setDimensions(num_dimensions_);
	setCoord(Eigen::VectorXf::Random(num_dimensions_));
}

// Make a copy of 'state'
base::RealVectorSpaceState::RealVectorSpaceState(std::shared_ptr<base::State> state)
{
	setStateSpaceType(StateSpaceType::RealVectorSpace);
	setDimensions(state->getNumDimensions());
	setCoord(state->getCoord());
	setTreeIdx(state->getTreeIdx());
	setIdx(state->getIdx());
	setDistance(state->getDistance());
	setCost(state->getCost());
	setNearestPoints(state->getNearestPoints());
	setParent(state->getParent());
	setChildren(state->getChildren());
}
