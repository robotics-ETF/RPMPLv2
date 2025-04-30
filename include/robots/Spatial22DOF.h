//
// Created by nermin on 25.04.25.
//

#ifndef RPMPL_SPATIAL22DOF_H
#define RPMPL_SPATIAL22DOF_H

#include "Spatial10DOF.h"

namespace robots
{
    class Spatial22DOF : public Spatial10DOF
    {
    public:
        Spatial22DOF(const std::string &robot_desc, size_t ground_included_);
        ~Spatial22DOF();

    };
}

#endif //RPMPL_SPATIAL22DOF_H