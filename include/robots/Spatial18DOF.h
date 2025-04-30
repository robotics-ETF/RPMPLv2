//
// Created by nermin on 25.04.25.
//

#ifndef RPMPL_SPATIAL18DOF_H
#define RPMPL_SPATIAL18DOF_H

#include "Spatial10DOF.h"

namespace robots
{
    class Spatial18DOF : public Spatial10DOF
    {
    public:
        Spatial18DOF(const std::string &robot_desc, size_t ground_included_);
        ~Spatial18DOF();
        
    };
}

#endif //RPMPL_SPATIAL18DOF_H