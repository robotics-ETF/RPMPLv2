//
// Created by nermin on 05.06.24.
//
#ifndef RPMPL_SPLINE4_H
#define RPMPL_SPLINE4_H

#include "Spline.h"

namespace planning
{
    namespace trajectory
    {
        class Spline4 : public Spline
        {
        public:
            Spline4();
            ~Spline4() {}
        };
    } 
}

#endif //RPMPL_SPLINE4_H