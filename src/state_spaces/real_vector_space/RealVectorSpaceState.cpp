#include "RealVectorSpaceState.h"

base::RealVectorSpaceState::RealVectorSpaceState(const Eigen::VectorXf &coord_) : State(coord_)
{
	state_space_type = base::StateSpaceType::RealVectorSpace;
}
