//
// Created by dinko on 7.3.21.
// Modified by nermin on 18.02.22.
//

#include "RealVectorSpaceState.h"

base::RealVectorSpaceState::RealVectorSpaceState(const Eigen::VectorXf &coord_) : State(coord_)
{
	state_space_type = StateSpaceType::RealVectorSpace;
}

// Make a copy of 'state'
base::RealVectorSpaceState::RealVectorSpaceState(const std::shared_ptr<base::State> state) : State(state->getCoord())
{
	state_space_type = StateSpaceType::RealVectorSpace;
	tree_idx = state->getTreeIdx();
	idx = state->getIdx();
	d_c = state->getDistance();
	d_c_underestimation = state->getDistanceUnderestimation();
	cost = state->getCost();
	nearest_points = state->getNearestPoints();
	parent = state->getParent();
	children = state->getChildren();
}
