#include "StateSpaceType.h"

namespace base
{
	std::ostream &operator<<(std::ostream &os, const base::StateSpaceType &type) 
	{
		switch (type)
		{
			case base::StateSpaceType::Abstract:
				os << "Abstract type";
				break;

			case base::StateSpaceType::RealVectorSpace:
				os << "RealVectorSpace type";
				break;

			case base::StateSpaceType::RealVectorSpaceFCL:
				os << "RealVectorSpaceFCL type";
				break;

			case base::StateSpaceType::SO2:
				os << "SO2 type";
				break;

			case base::StateSpaceType::SO3:
				os << "SO3 type";
				break;
		}

		return os;
	}
}