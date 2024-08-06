//
// Created by nermin on 16.03.24.
//

#ifndef RPMPL_PLANNINGTYPES_H
#define RPMPL_PLANNINGTYPES_H

#include <ostream>
#include <string>
#include <unordered_map>

namespace planning
{
	enum class TimeUnit
	{
		s,
		ms,
		us,
		ns
	};

	static std::unordered_map<std::string, planning::TimeUnit> time_unit_map = 
	{
		{ "s", planning::TimeUnit::s },
		{ "ms", planning::TimeUnit::ms },
		{ "us", planning::TimeUnit::us },
		{ "ns", planning::TimeUnit::ns }
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

	static std::unordered_map<std::string, planning::PlannerType> planner_type_map = 
	{
		{ "Abstract", planning::PlannerType::Abstract },
		{ "RRT-Connect", planning::PlannerType::RRTConnect },
		{ "RBT-Connect", planning::PlannerType::RBTConnect },
		{ "RGBT-Connect", planning::PlannerType::RGBTConnect },
		{ "RGBMT*", planning::PlannerType::RGBMTStar },
		{ "DRGBT", planning::PlannerType::DRGBT }
	};

	enum class RealTimeScheduling
	{
		None,
		FPS
	};

	static std::unordered_map<std::string, planning::RealTimeScheduling> real_time_scheduling_map = 
	{
		{ "None", planning::RealTimeScheduling::None },
		{ "FPS", planning::RealTimeScheduling::FPS}
	};

	enum class TrajectoryInterpolation
	{
		None,
		Spline
	};

	static std::unordered_map<std::string, planning::TrajectoryInterpolation> trajectory_interpolation_map = 
	{
		{ "None", planning::TrajectoryInterpolation::None },
		{ "Spline", planning::TrajectoryInterpolation::Spline}
	};

	std::ostream &operator<<(std::ostream &os, const planning::PlannerType &type);
	std::ostream &operator<<(std::ostream &os, const planning::RealTimeScheduling &type);
	std::ostream &operator<<(std::ostream &os, const planning::TrajectoryInterpolation &type);
}

#endif //RPMPL_PLANNINGTYPES_H