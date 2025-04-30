#include "Spatial18DOF.h"

robots::Spatial18DOF::~Spatial18DOF() {}

robots::Spatial18DOF::Spatial18DOF(const std::string &robot_desc, size_t ground_included_) : 
	Spatial10DOF(robot_desc, ground_included_, 18) {}
