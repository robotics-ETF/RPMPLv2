//
// Created by nermin on 05.09.22.
//

#ifndef RPMPL_PLANAR10DOF_H
#define RPMPL_PLANAR10DOF_H

#include "Planar2DOF.h"

namespace robots
{
    class Planar10DOF : public Planar2DOF
    {
    public:
        Planar10DOF(const std::string &robot_desc);
        ~Planar10DOF();
        
    };
}
#endif //RPMPL_ABSTRACTPLANNER_H