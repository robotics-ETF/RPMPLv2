//
// Created by nermin on 25.04.25.
//

#ifndef RPMPL_SPATIAL14DOF_H
#define RPMPL_SPATIAL14DOF_H

#include "Spatial10DOF.h"

namespace robots
{
    class Spatial14DOF : public Spatial10DOF
    {
    public:
        Spatial14DOF(const std::string &robot_desc, size_t ground_included_);
        ~Spatial14DOF();
        
    };
}

#endif //RPMPL_SPATIAL14DOF_H