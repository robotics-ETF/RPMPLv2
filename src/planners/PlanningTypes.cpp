#include "PlanningTypes.h"

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

	std::ostream &operator<<(std::ostream &os, const planning::RealTimeScheduling &type)
	{
		switch (type)
		{
			case planning::RealTimeScheduling::None:
				os << "none";
				break;

			case planning::RealTimeScheduling::FPS:
				os << "FPS";
				break;
		}

		return os;
	}

	std::ostream &operator<<(std::ostream &os, const planning::TrajectoryInterpolation &type)
	{
		switch (type)
		{
			case planning::TrajectoryInterpolation::None:
				os << "none";
				break;

			case planning::TrajectoryInterpolation::Spline:
				os << "spline";
				break;
		}

		return os;
	}
}
