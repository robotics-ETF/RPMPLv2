//
// Created by dinko on 14.02.22.
// Modified by nermin on 09.02.24.
//

#ifndef RPMPL_ENVIRONMENT_H
#define RPMPL_ENVIRONMENT_H

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/parser.h>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>

#include "Box.h"

namespace env
{	
	class Environment
	{
	public:
		Environment(const std::string &config_file_path, const std::string &root_path = "");
		~Environment();

		inline void setBaseRadius(float base_radius_) { base_radius = base_radius_; }
		inline void setRobotMaxVel(float robot_max_vel_) { robot_max_vel = robot_max_vel_; }
		inline void setGroundIncluded(size_t ground_included_) { ground_included = ground_included_; }

		inline const std::vector<std::shared_ptr<env::Object>> &getObjects() const { return objects; }
		inline std::shared_ptr<env::Object> getObject(size_t idx) const { return objects[idx]; }
		inline std::shared_ptr<fcl::CollisionObjectf> getCollObject(size_t idx) const { return objects[idx]->getCollObject(); }
		inline size_t getNumObjects() const { return objects.size(); }
		inline const fcl::Vector3f &getWSCenter() const { return WS_center; }
		inline float getWSRadius() const { return WS_radius; }

		void addObject(const std::shared_ptr<env::Object> object, const fcl::Vector3f &velocity = fcl::Vector3f::Zero(), 
			const fcl::Vector3f &acceleration = fcl::Vector3f::Zero());
		void removeObject(size_t idx);
		void removeObjects(int start_idx, int end_idx = -1);
		void removeObjects(const std::string &label, bool with_label = true);
		void removeAllObjects();
		bool isValid(const Eigen::Vector3f &pos, float vel);
		void updateEnvironment(float delta_time);

	private:
		std::vector<std::shared_ptr<env::Object>> objects;		// All objects/parts of the environment
        fcl::Vector3f WS_center;								// Workspace center point in [m]
        float WS_radius; 										// Workspace radius in [m]
		float base_radius;
		float robot_max_vel;
		size_t ground_included;
	};
}

#endif //RPMPL_ENVIRONMENT_H