#include "Spatial14DOF.h"

robots::Spatial14DOF::~Spatial14DOF() {}

robots::Spatial14DOF::Spatial14DOF(const std::string &robot_desc, size_t ground_included_) : 
	Spatial10DOF(robot_desc, ground_included_, 14) {}
