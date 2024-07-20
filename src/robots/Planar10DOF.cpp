#include "Planar10DOF.h"

robots::Planar10DOF::~Planar10DOF() {}

robots::Planar10DOF::Planar10DOF(const std::string &robot_desc) : Planar2DOF(robot_desc, 10) {}

bool robots::Planar10DOF::checkSelfCollision([[maybe_unused]] const std::shared_ptr<base::State> q1, 
											 [[maybe_unused]] std::shared_ptr<base::State> &q2)
{
    // TODO (if needed)
	return false;
}

bool robots::Planar10DOF::checkSelfCollision([[maybe_unused]] const std::shared_ptr<base::State> q)
{
	// TODO (if needed)
	return false;
}
