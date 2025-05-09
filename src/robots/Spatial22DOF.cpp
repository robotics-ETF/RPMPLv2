#include "Spatial22DOF.h"

robots::Spatial22DOF::~Spatial22DOF() {}

robots::Spatial22DOF::Spatial22DOF(const std::string &robot_desc, size_t ground_included_) : 
	Spatial10DOF(robot_desc, ground_included_, 22) {}
