#include <iostream>
#include <memory>

#include <glog/logging.h>

#include "fcl/fcl.h"

typedef std::shared_ptr <fcl::CollisionGeometryf> CollisionGeometryPtr;

int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int)time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

	Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");

	std::shared_ptr<fcl::BroadPhaseCollisionManagerf> manager1 = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
	std::shared_ptr<fcl::BroadPhaseCollisionManagerf> manager2 = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();


	CollisionGeometryPtr obs1(new fcl::Boxf(1.0005, 0.1, 0.1));
	fcl::Vector3f tr1(0.5, 0, 0);
    fcl::Quaternionf quat1(0, 0, 0, 0);
    fcl::Transform3f tf1(fcl::Transform3f::Identity()); 
	tf1.translation() = tr1;

	std::shared_ptr<fcl::CollisionObjectf> ob1(new fcl::CollisionObjectf(obs1, tf1));
	ob1->computeAABB();


	CollisionGeometryPtr obs2(new fcl::Boxf(0.2, 0.2, 0.2));
	fcl::Vector3f tr2(1.5, 0, 0);
    fcl::Quaternionf quat2(0, 0, 0, 0);
    fcl::Transform3f tf2(fcl::Transform3f::Identity()); tf2.translation() = tr2; tf2.linear() = quat2.normalized().matrix();

	std::shared_ptr<fcl::CollisionObjectf> ob2(new fcl::CollisionObjectf(obs2, tf2));
	ob2->computeAABB();
	LOG(INFO) << ob2->getAABB().min_.transpose().format(fmt) << "\t" << ob2->getAABB().max_.transpose().format(fmt);

	manager1->registerObject(ob1.get());
	manager2->registerObject(ob2.get());
	manager1->setup();
	manager2->setup();

	fcl::DefaultDistanceData<float> distance_data;
	distance_data.request.enable_nearest_points = true;
	//distance_data.request.gjk_solver_type = fcl::GST_INDEP;
	distance_data.result.clear();
	manager1->distance(manager2.get(), &distance_data, fcl::DefaultDistanceFunction);

	fcl::DefaultCollisionData<float> collision_data;
	collision_data.result.clear();
	manager1->collide(manager2.get(), &collision_data, fcl::DefaultCollisionFunction);


	LOG(INFO) << "distance query : " << distance_data.result.min_distance << " p1: " << distance_data.result.nearest_points[0].transpose().format(fmt)
				  << "\t p2: " << distance_data.result.nearest_points[1].transpose().format(fmt);

	LOG(INFO) << "collision query : " << collision_data.result.isCollision();

	google::ShutDownCommandLineFlags();
	return 0;
}