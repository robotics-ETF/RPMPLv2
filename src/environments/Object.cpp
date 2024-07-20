#include "Object.h"

env::Object::~Object() {}

namespace env 
{
    std::ostream &operator<<(std::ostream &os, const std::shared_ptr<env::Object> obj)
    {
        obj->coll_object->computeAABB();

        os << "Object " << obj->getLabel() << "\t";
        os << "Range: (" << obj->getCollObject()->getAABB().min_.transpose() << ") to (" 
                        << obj->getCollObject()->getAABB().max_.transpose() << ")\t ";
        os << "Max. vel: " << obj->getMaxVel() << "\t";
        os << "Max. acc: " << obj->getMaxAcc() << "\n";

        return os;
    }
}

void env::Object::setPosition(const fcl::Vector3f &position_) 
{ 
    position = position_;
    coll_object->setTranslation(position);
    coll_object->computeAABB();
}