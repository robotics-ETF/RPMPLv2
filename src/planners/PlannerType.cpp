//
// Created by nermin on 16.03.24.
//

#include "PlannerType.h"

namespace planning
{
	std::ostream &operator<<(std::ostream &os, const planning::PlannerType &type) 
	{
		switch (type)
		{
			case planning::PlannerType::Abstract:
				os << "Abstract";
				break;

			case planning::PlannerType::RRTConnect:
				os << "RRT-Connect";
				break;

			case planning::PlannerType::RBTConnect:
				os << "RBT-Connect";
				break;

			case planning::PlannerType::RGBTConnect:
				os << "RGBT-Connect";
				break;

			case planning::PlannerType::RGBMTStar:
				os << "RGBMT*";
				break;

			case planning::PlannerType::DRGBT:
				os << "DRGBT";
				break;
		}

		return os;
	}
}
