//
// Created by nermin on 16.03.24.
//

#ifndef RPMPL_PLANNERTYPE_H
#define RPMPL_PLANNERTYPE_H

#include <ostream>
#include <string>

namespace planning
{
	enum class PlannerType
	{
		Abstract, 	// only here for placeholding
		RRTConnect,
		RBTConnect,
		RGBTConnect,
		RGBMTStar,
		DRGBT
	};

	std::ostream &operator<<(std::ostream &os, const planning::PlannerType &type);
}

#endif //RPMPL_PLANNERTYPE_H
