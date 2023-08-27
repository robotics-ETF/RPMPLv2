//
// Created by dinko on 7.3.21
// Modified by nermin on 18.02.22.
//

#ifndef RPMPL_REALVECTORSPACESTATE_H
#define RPMPL_REALVECTORSPACESTATE_H

#include "State.h"
#include <Eigen/Dense>

namespace base
{
	class RealVectorSpaceState : public State
	{
	public:
		RealVectorSpaceState(Eigen::VectorXf state_);
		RealVectorSpaceState(int num_dimensions_);
		RealVectorSpaceState(std::shared_ptr<base::State> state);
		~RealVectorSpaceState() {}
	};
}

#endif //RPMPL_REALVECTORSPACESTATE_H
