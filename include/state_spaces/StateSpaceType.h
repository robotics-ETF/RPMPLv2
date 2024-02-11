//
// Created by dinko on 21.5.21..
//

#ifndef RPMPL_STATESPACETYPE_H
#define RPMPL_STATESPACETYPE_H

#include <ostream>
#include <string>

enum class StateSpaceType
{
	Abstract, // only here for placeholding
	RealVectorSpace,
	RealVectorSpaceFCL,
	SO2,
	SO3
};

std::ostream &operator<<(std::ostream &os, const StateSpaceType &type);


#endif //RPMPL_STATESPACETYPE_H
