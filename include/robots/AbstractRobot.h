//
// Created by dinko on 07.02.22.
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
		inline int getNumDOFs() const { return num_DOFs; }
		inline int getNumLinks() const { return links.size(); }
		inline const std::vector<std::unique_ptr<fcl::CollisionObjectf>> &getLinks() const { return links; }
		inline const std::vector<std::pair<float, float>> &getLimits() const { return limits; }
		inline std::shared_ptr<base::State> getConfiguration() const { return configuration; }
		inline float getCapsuleRadius(int num) const { return capsules_radius[num]; }
		inline float getMaxVel(int num) const { return max_vel[num]; }
		inline float getMaxAcc(int num) const { return max_acc[num]; }
		inline float getMaxJerk(int num) const { return max_jerk[num]; }

		inline void setConfiguration(const std::shared_ptr<base::State> configuration_) { configuration = configuration_; }
		inline void setCapsulesRadius(const std::vector<float> &capsules_radius_) { capsules_radius = capsules_radius_; }
		inline void setMaxVel(const std::vector<float> &max_vel_) { max_vel = max_vel_; }
		inline void setMaxAcc(const std::vector<float> &max_acc_) { max_acc = max_acc_; }
		inline void setMaxJerk(const std::vector<float> &max_jerk_) { max_jerk = max_jerk_; }

		virtual void setState(const std::shared_ptr<base::State> q) = 0;
		virtual std::shared_ptr<std::vector<KDL::Frame>> computeForwardKinematics(const std::shared_ptr<base::State> q) = 0;
		virtual std::shared_ptr<base::State> computeInverseKinematics(const KDL::Rotation &R, const KDL::Vector &p,
																	  const std::shared_ptr<base::State> q_init = nullptr) = 0;
		virtual std::shared_ptr<Eigen::MatrixXf> computeSkeleton(const std::shared_ptr<base::State> q) = 0;
		virtual float computeStep(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, float d_c, 
			float rho, const std::shared_ptr<Eigen::MatrixXf> skeleton) = 0;
		virtual float computeStep2(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, 
			const std::vector<float> &d_c_profile, const std::vector<float> &rho_profile, const std::shared_ptr<Eigen::MatrixXf> skeleton) = 0;

	protected:
		std::string type;
		int num_DOFs;
		std::vector<std::unique_ptr<fcl::CollisionObjectf>> links;
		std::vector<std::pair<float, float>> limits;
		std::shared_ptr<base::State> configuration;
		std::vector<float> capsules_radius;
		std::vector<float> max_vel;
		std::vector<float> max_acc;
		std::vector<float> max_jerk;
	};
}
#endif //RPMPL_ABSTRACTROBOT_H
