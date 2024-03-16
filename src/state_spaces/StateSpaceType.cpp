//
// Created by dinko on 21.5.21.
//

#include "StateSpaceType.h"

std::ostream &operator<<(std::ostream &os, const StateSpaceType &type) 
{
	switch (type)
	{
		case StateSpaceType::Abstract:
			os << "Abstract type";
			break;

		case StateSpaceType::RealVectorSpace:
			os << "RealVectorSpace type";
			break;

		case StateSpaceType::RealVectorSpaceFCL:
			os << "RealVectorSpaceFCL type";
			break;

		case StateSpaceType::SO2:
			os << "SO2 type";
			break;

		case StateSpaceType::SO3:
			os << "SO3 type";
			break;
	}

	return os;
}
