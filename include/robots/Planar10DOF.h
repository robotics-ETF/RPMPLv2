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

		bool checkSelfCollision(const std::shared_ptr<base::State> q1, std::shared_ptr<base::State> &q2) override;
		bool checkSelfCollision(const std::shared_ptr<base::State> q) override;
    };
}

#endif //RPMPL_ABSTRACTPLANNER_H