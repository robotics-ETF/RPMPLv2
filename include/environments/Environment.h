//
// Created by dinko on 14.02.22.
//

#ifndef RPMPL_ENVIRONMENT_H
#define RPMPL_ENVIRONMENT_H

#include <vector> 
#include <memory>
#include <fcl/fcl.h>

namespace env
{
	typedef std::pair<fcl::Box<float>, fcl::Transform3<float>> Obstacle;
	
	class Environment
	{
	public:
		Environment(const std::string &filename); // filename with description
        Environment(const fcl::Box<float> &box, const fcl::Transform3<float> &tf);
		Environment(const std::vector<Obstacle> obs);
		~Environment();

		const std::vector<std::shared_ptr<fcl::CollisionObject<float>>> &getParts() const { return parts; }
		void setMaxVel(float max_vel_) { max_vel = max_vel_; }
		void setParts(const std::vector<std::shared_ptr<fcl::CollisionObject<float>>> &parts_);
		void addCollisionObject(const std::shared_ptr<fcl::CollisionObject<float>> ob);
		void removeCollisionObjects(int start_idx);
		void updateEnvironment(float step);

	private:
		std::vector<std::shared_ptr<fcl::CollisionObject<float>>> parts;
		std::vector<fcl::Vector3f> velocities; 		// Velocity vector for each obstacle
		float max_vel; 								// Maximal velocity for each obstacle
        const Eigen::Vector3f WS_center = Eigen::Vector3f(0, 0, 0.267);
        const float WS_radius = 1.0;
	};
}
#endif //RPMPL_ABSTRACTPLANNER_H