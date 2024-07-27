//
// Created by nermin on 13.04.22.
//

#ifndef RPMPL_HORIZONSTATE_H
#define RPMPL_HORIZONSTATE_H

#include "State.h"
#include "DRGBTConfig.h"
#include "RealVectorSpaceConfig.h"

namespace planning::drbt
{
    class HorizonState
    {
    public:
        HorizonState() {}
        HorizonState(const std::shared_ptr<base::State> state_, int index_);
        HorizonState(std::shared_ptr<base::State> state_, int index_, std::shared_ptr<base::State> state_reached_);
        ~HorizonState() {}

        enum class Status {Good, Bad, Critical, Goal};

        inline std::shared_ptr<base::State> getState() const { return state; }
        inline std::shared_ptr<base::State> getStateReached() const { return state_reached; }
        inline Status getStatus() const { return status; }
        inline int getIndex() const { return index; }
        inline float getDistance() const { return d_c; }
        inline float getDistancePrevious() const { return d_c_previous; }
        inline float getWeight() const { return weight; }
        inline const Eigen::VectorXf &getCoord() const { return state->getCoord(); }
        inline float getCoord(size_t idx) const { return state->getCoord(idx); }
        inline const Eigen::VectorXf &getCoordReached() const { return state_reached->getCoord(); }
        inline float getCoordReached(size_t idx) const { return state_reached->getCoord(idx); }
        inline bool getIsReached() const { return is_reached; }

        inline void setStateReached(const std::shared_ptr<base::State> state_reached_) { state_reached = state_reached_; }
        inline void setStatus(Status status_) { status = status_; }
        inline void setIndex(int index_) { index = index_; }
        void setDistance(float d_c_);
        inline void setDistancePrevious(float d_c_previous_) { d_c_previous = d_c_previous_; }
        void setWeight(float weight_);
        inline void setIsReached(bool is_reached_) { is_reached = is_reached_; }

        friend std::ostream &operator<<(std::ostream &os, const std::shared_ptr<planning::drbt::HorizonState> q);

    private:
        std::shared_ptr<base::State> state;
        std::shared_ptr<base::State> state_reached;     // Reached state when generating spine from 'q_current' towards 'state'
        Status status;                                  // Status of 'state_reached': 'Good', 'Bad', 'Critical' or 'Goal'
        int index;                                      // Index of 'state' in the predefined path. It is -1 if 'state' does not belong to the path
        float d_c;                                      // Underestimation of distance-to-obstacles for 'state_reached'
        float d_c_previous;                             // 'd_c' from previous iteration
        float weight;                                   // Weight in range [0, 1] for 'state_reached'
        bool is_reached;                                // Whether 'state_reached' == 'state'
    };
}

#endif //RPMPL_HORIZONSTATE_H