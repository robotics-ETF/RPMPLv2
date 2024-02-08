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
		void setMaxAcc(float max_acc_) { max_acc = max_acc_; }
		void setParts(const std::vector<std::shared_ptr<fcl::CollisionObject<float>>> &parts_);
		void setVelocities(const std::vector<fcl::Vector3f> &velocities_) { velocities = velocities_; }
		void setRobotMaxVel(float robot_max_vel_) { robot_max_vel = robot_max_vel_; }
		void setBaseRadius(float base_radius_) { base_radius = base_radius_; }
		void addCollisionObject(const std::shared_ptr<fcl::CollisionObject<float>> ob, 
								const fcl::Vector3f &velocity = fcl::Vector3f::Zero(), float acc_sign = 0);
		void removeCollisionObjects(int start_idx);
		bool isValid(const Eigen::Vector3f &pos, float obs_vel);
		void updateEnvironment(float delta_time);

		const fcl::Vector3f &getWSCenter() { return WS_center; }
		const float getWSRadius() { return WS_radius; }

	private:
		std::vector<std::shared_ptr<fcl::CollisionObject<float>>> parts;	// All parts of the environment
		std::vector<fcl::Vector3f> velocities; 								// Velocity vector for each obstacle in [m/s]
		float max_vel; 														// Maximal velocity for each obstacle in [m/s]
		float max_acc; 														// Maximal acceleration for each obstacle in [m/s²]
        const fcl::Vector3f WS_center = fcl::Vector3f(0, 0, 0.267);			// Workspace center point in [m]
        const float WS_radius = 1.0; 										// Workspace radius in [m]
		std::vector<int> acc_signs;		// Sign of acceleration for each obstacle. If 1, obstacle is accelerating, and if -1, obstacle is decelerating
		float robot_max_vel;
		float base_radius;
	};
}
#endif //RPMPL_ABSTRACTPLANNER_H