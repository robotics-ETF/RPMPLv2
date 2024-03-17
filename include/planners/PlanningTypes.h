//
// Created by nermin on 16.03.24.
//

#ifndef RPMPL_PLANNINGTYPES_H
#define RPMPL_PLANNINGTYPES_H

#include <ostream>
#include <string>

namespace planning
{
	enum class TimeUnit
	{
		s,
		ms,
		us,
		ns
	};
	
	enum class PlannerType
	{
		Abstract, 	// only here for placeholding
		RRTConnect,
		RBTConnect,
		RGBTConnect,
		RGBMTStar,
		DRGBT
	};

	enum class RealTimeScheduling
	{
		None,
		FPS
	};

	enum class TrajectoryInterpolation
	{
		None,
		Spline
	};

	std::ostream &operator<<(std::ostream &os, const planning::PlannerType &type);
	std::ostream &operator<<(std::ostream &os, const planning::RealTimeScheduling &type);
	std::ostream &operator<<(std::ostream &os, const planning::TrajectoryInterpolation &type);
}

#endif //RPMPL_PLANNINGTYPES_H
