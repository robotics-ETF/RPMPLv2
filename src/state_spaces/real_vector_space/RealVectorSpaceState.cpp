//
// Created by dinko on 7.3.21.
// Modified by nermin on 18.02.22.
//

#include "RealVectorSpaceState.h"

base::RealVectorSpaceState::RealVectorSpaceState(const Eigen::VectorXf &coord_) : State(coord_)
{
	setStateSpaceType(StateSpaceType::RealVectorSpace);
}

// Make a copy of 'state'
base::RealVectorSpaceState::RealVectorSpaceState(const std::shared_ptr<base::State> state) : State(state->getCoord())
{
	setStateSpaceType(StateSpaceType::RealVectorSpace);
	setTreeIdx(state->getTreeIdx());
	setIdx(state->getIdx());
	setDistance(state->getDistance());
	setDistanceUnderestimation(state->getDistanceUnderestimation());
	setCost(state->getCost());
	setNearestPoints(state->getNearestPoints());
	setParent(state->getParent());
	setChildren(state->getChildren());
}
