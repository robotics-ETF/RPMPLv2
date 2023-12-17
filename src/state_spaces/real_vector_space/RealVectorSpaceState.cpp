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
base::RealVectorSpaceState::RealVectorSpaceState(const std::shared_ptr<base::State> state) : State()
{
	state_space_type = StateSpaceType::RealVectorSpace;
	coord = state->getCoord();
	num_dimensions = state->getNumDimensions();
	tree_idx = state->getTreeIdx();
	idx = state->getIdx();
	d_c = state->getDistance();
	d_c_profile = state->getDistanceProfile();
	is_real_d_c = state->getIsRealDistance();
	cost = state->getCost();
	nearest_points = state->getNearestPoints();
	parent = state->getParent();
	children = state->getChildren();
}
