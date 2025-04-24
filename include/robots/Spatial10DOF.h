//
// Created by nermin on 05.09.22.
//

#ifndef RPMPL_SPATIAL10DOF_H
#define RPMPL_SPATIAL10DOF_H

#include "Planar2DOF.h"

namespace robots
{
    class Spatial10DOF : public Planar2DOF
    {
    public:
        Spatial10DOF(const std::string &robot_desc, size_t ground_included_);
        ~Spatial10DOF();

        std::shared_ptr<Eigen::MatrixXf> computeEnclosingRadii(const std::shared_ptr<base::State> &q) override;
		bool checkSelfCollision(const std::shared_ptr<base::State> &q1, std::shared_ptr<base::State> &q2) override;
		bool checkSelfCollision(const std::shared_ptr<base::State> &q) override;
    };
}

#endif //RPMPL_SPATIAL10DOF_H