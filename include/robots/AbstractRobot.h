//
// Created by dinko on 07.02.22.
// Modified by nermin on 05.09.22.
//

#ifndef RPMPL_ABSTRACTROBOT_H
#define RPMPL_ABSTRACTROBOT_H

#include <vector>
#include <string>
#include <memory>
#include <Eigen/Dense>
#include <fcl/fcl.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

#include "State.h"

namespace robots
{
	class AbstractRobot
	{
	public:
		explicit AbstractRobot() { configuration = nullptr; }
		virtual ~AbstractRobot() = 0;
		
		inline const std::string &getType() const { return type; }
		inline size_t getNumDOFs() const { return num_DOFs; }
		inline size_t getNumLinks() const { return links.size(); }
		inline const std::vector<std::unique_ptr<fcl::CollisionObjectf>> &getLinks() const { return links; }
		inline const std::vector<std::pair<float, float>> &getLimits() const { return limits; }
		inline std::shared_ptr<base::State> getConfiguration() const { return configuration; }
		inline float getCapsuleRadius(size_t idx) const { return capsules_radius[idx]; }
		inline float getMaxVel(size_t idx) const { return max_vel(idx); }
		inline float getMaxAcc(size_t idx) const { return max_acc(idx); }
		inline float getMaxJerk(size_t idx) const { return max_jerk(idx); }
		inline const Eigen::VectorXf &getMaxVel() const { return max_vel; }
		inline const Eigen::VectorXf &getMaxAcc() const { return max_acc; }
		inline const Eigen::VectorXf &getMaxJerk() const { return max_jerk; }
		inline bool getSelfCollisionChecking() const { return self_collision_checking; }
		inline float getGripperLength() const { return gripper_length; }
		inline size_t getGroundIncluded() const { return ground_included; }

		inline void setConfiguration(const std::shared_ptr<base::State> configuration_) { configuration = configuration_; }
		inline virtual void setCapsulesRadius(const std::vector<float> &capsules_radius_) { capsules_radius = capsules_radius_; }
		inline void setMaxVel(const Eigen::VectorXf &max_vel_) { max_vel = max_vel_; }
		inline void setMaxAcc(const Eigen::VectorXf &max_acc_) { max_acc = max_acc_; }
		inline void setMaxJerk(const Eigen::VectorXf &max_jerk_) { max_jerk = max_jerk_; }
		inline void setSelfCollisionChecking(bool self_collision_checking_) { self_collision_checking = self_collision_checking_; }
		inline void setGripperLength(float gripper_length_) { gripper_length = gripper_length_; }
		inline void setGroundIncluded(size_t ground_included_) { ground_included = ground_included_; }

		virtual void setState(const std::shared_ptr<base::State> q) = 0;
		virtual std::shared_ptr<std::vector<KDL::Frame>> computeForwardKinematics(const std::shared_ptr<base::State> q) = 0;
		virtual std::shared_ptr<base::State> computeInverseKinematics(const KDL::Rotation &R, const KDL::Vector &p,
																	  const std::shared_ptr<base::State> q_init = nullptr) = 0;
		virtual std::shared_ptr<Eigen::MatrixXf> computeSkeleton(const std::shared_ptr<base::State> q) = 0;
		virtual std::shared_ptr<Eigen::MatrixXf> computeEnclosingRadii(const std::shared_ptr<base::State> q) = 0;
		virtual bool checkSelfCollision(const std::shared_ptr<base::State> q1, std::shared_ptr<base::State> &q2) = 0;
		virtual bool checkSelfCollision(const std::shared_ptr<base::State> q) = 0;

	protected:
		std::string type;
		size_t num_DOFs;
		std::vector<std::unique_ptr<fcl::CollisionObjectf>> links;
		std::vector<std::pair<float, float>> limits;
		std::shared_ptr<base::State> configuration;
		std::vector<float> capsules_radius;
		Eigen::VectorXf max_vel;
		Eigen::VectorXf max_acc;
		Eigen::VectorXf max_jerk;
		bool self_collision_checking;
		float gripper_length;
		size_t ground_included;
	};
}

#endif //RPMPL_ABSTRACTROBOT_H