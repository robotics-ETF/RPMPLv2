#include "HorizonState.h"

planning::drbt::HorizonState::HorizonState(std::shared_ptr<base::State> state_, int index_)
{
    state = state_;
    index = index_;
    state_reached = nullptr;
    status = planning::drbt::HorizonState::Status::Good;
    d_c = -1;
    d_c_previous = -1;
    weight = -1;
    is_reached = false;
}

planning::drbt::HorizonState::HorizonState(std::shared_ptr<base::State> state_, int index_, std::shared_ptr<base::State> state_reached_)
{
    state = state_;
    index = index_;
    state_reached = state_reached_;
    status = planning::drbt::HorizonState::Status::Good;
    d_c = state_reached->getDistance();
    d_c_previous = state_reached->getDistance();
    weight = -1;
    is_reached = (state->getCoord() - state_reached->getCoord()).norm() < RealVectorSpaceConfig::EQUALITY_THRESHOLD ? true : false;
}

void planning::drbt::HorizonState::setDistance(float d_c_)
{
    d_c = d_c_;
    if (d_c < DRGBTConfig::D_CRIT)
    {
        setStatus(planning::drbt::HorizonState::Status::Critical);
        weight = 0;
    }
}

void planning::drbt::HorizonState::setWeight(float weight_) 
{ 
    weight = weight_;
    if (weight == 0)
        setStatus(planning::drbt::HorizonState::Status::Bad);
}

namespace planning::drbt 
{
    std::ostream &operator<<(std::ostream &os, const std::shared_ptr<planning::drbt::HorizonState> q)
    {
        os << "q:         (" << q->getCoord().transpose() << ")" << std::endl;
        if (q->getStateReached() == nullptr)
            os << "q_reached: NONE " << std::endl;
        else
            os << "q_reached: (" << q->getStateReached()->getCoord().transpose() << ")" << std::endl;
            
        if (q->getStatus() == planning::drbt::HorizonState::Status::Good)
            os << "status:     " << "Good " << std::endl;
        else if (q->getStatus() == planning::drbt::HorizonState::Status::Bad)
            os << "status:     " << "Bad " << std::endl;
        else if (q->getStatus() == planning::drbt::HorizonState::Status::Critical)
            os << "status:     " << "Critical " << std::endl;
        else if (q->getStatus() == planning::drbt::HorizonState::Status::Goal)
            os << "status:     " << "Goal " << std::endl;

        os << "idx path:   " << q->getIndex() << std::endl;
        os << "d_c:        " << q->getDistance() << std::endl;
        os << "weight:     " << q->getWeight() << std::endl;
        return os;
    }
}