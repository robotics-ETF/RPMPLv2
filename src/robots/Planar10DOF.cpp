//
// Created by nermin on 05.09.22.
//

#include "Planar10DOF.h"

robots::Planar10DOF::~Planar10DOF() {}

robots::Planar10DOF::Planar10DOF(std::string robot_desc) : Planar2DOF(robot_desc, 10) {}
