//
// Created by dinko on 7.3.21.
// Modified by nermin on 18.02.22.
//

#ifndef RPMPL_REALVECTORSPACESTATE_H
#define RPMPL_REALVECTORSPACESTATE_H

#include "State.h"

namespace base
{
	class RealVectorSpaceState : public State
	{
	public:
		RealVectorSpaceState(const Eigen::VectorXf &coord_);
		RealVectorSpaceState(const std::shared_ptr<base::State> state);
		~RealVectorSpaceState() {}
	};
}

#endif //RPMPL_REALVECTORSPACESTATE_H