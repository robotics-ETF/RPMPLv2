#include "HorizonState.h"

planning::drbt::HorizonState::HorizonState(std::shared_ptr<base::State> state_, int index_)
{
    state = state_;
    index = index_;
    state_reached = nullptr;
    status = HorizonState::Status::Good;
    d_c = -1;
    d_c_previous = -1;
    weight = -1;
    is_reached = false;
}

void planning::drbt::HorizonState::setDistance(float d_c_)
{
    d_c = d_c_;
    if (d_c < DRGBTConnectConfig::D_CRIT)
        setStatus(HorizonState::Status::Critical);
}

void planning::drbt::HorizonState::setWeight(float weight_) 
{ 
    weight = weight_;
    if (weight == 0)
        setStatus(HorizonState::Status::Bad);
}

std::ostream &planning::drbt::operator<<(std::ostream &os, const planning::drbt::HorizonState *q)
{
    os << "q:         (" << q->getCoord().transpose() << ")" << std::endl;
    if (q->getStateReached() == nullptr)
        os << "q_reached: NONE " << std::endl;
    else
        os << "q_reached: (" << q->getStateReached()->getCoord().transpose() << ")" << std::endl;
        
    if (q->getStatus() == HorizonState::Status::Good)
        os << "status:     " << "Good " << std::endl;
    else if (q->getStatus() == HorizonState::Status::Bad)
        os << "status:     " << "Bad " << std::endl;
    else if (q->getStatus() == HorizonState::Status::Critical)
        os << "status:     " << "Critical " << std::endl;
    else if (q->getStatus() == HorizonState::Status::Goal)
        os << "status:     " << "Goal " << std::endl;

    os << "idx path:   " << q->getIndex() << std::endl;
    os << "d_c:        " << q->getDistance() << std::endl;
    os << "weight:     " << q->getWeight() << std::endl;
    return os;
}