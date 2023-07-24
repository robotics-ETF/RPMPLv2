//
// Created by nermin on 05.09.22.
//

#include <iostream>
#include <vector>
#include "Planar10DOF.h"
#include "urdf/model.h"
#include "RealVectorSpaceState.h"
#include <Eigen/Dense>

#include <glog/logging.h>

robots::Planar10DOF::~Planar10DOF() {}

robots::Planar10DOF::Planar10DOF(std::string robot_desc) : Planar2DOF(robot_desc, 10) {}

