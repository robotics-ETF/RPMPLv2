//
// Created by nermin on 09.02.24.
//

#ifndef RPMPL_BOX_H
#define RPMPL_BOX_H

#include "Object.h"

namespace env
{	
	class Box : public Object
	{
	public:
		Box(const fcl::Vector3f &dim, const fcl::Vector3f &pos, const fcl::Quaternionf &rot, const std::string &label_ = "");
		~Box() {}

	};
}

#endif //RPMPL_BOX_H