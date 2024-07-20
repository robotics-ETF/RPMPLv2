#include "Box.h"

env::Box::Box(const fcl::Vector3f &dim, const fcl::Vector3f &pos, const fcl::Quaternionf &rot, const std::string &label_)
{
    std::shared_ptr<fcl::CollisionGeometry<float>> fcl_box { std::make_shared<fcl::Box<float>>(dim) };
    coll_object = std::make_shared<fcl::CollisionObject<float>>(fcl_box, rot.matrix(), pos);
    coll_object->computeAABB();
    position = pos;
    velocity = fcl::Vector3f::Zero();
    acceleration = fcl::Vector3f::Zero();
    max_vel = 0;
    max_acc = 0;
    min_dist_tol = INFINITY;
    label = label_;
}
