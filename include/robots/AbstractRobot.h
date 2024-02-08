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
		
		const std::string &getType() { return type; }
		const int getNumDOFs() { return num_DOFs; }
		const std::vector<std::unique_ptr<fcl::CollisionObjectf>> &getParts() const { return parts; }
		const std::vector<std::vector<float>> &getLimits() const { return limits; }
		const std::shared_ptr<base::State> getConfiguration() { return configuration; }
		const float getCapsuleRadius(int num) { return capsules_radius[num]; }
		const float getMaxVel(int num) { return max_vel[num]; }
		const float getMaxAcc(int num) { return max_acc[num]; }
		const float getMaxJerk(int num) { return max_jerk[num]; }

		void setConfiguration(std::shared_ptr<base::State> configuration_) { configuration = configuration_; }
		void setCapsulesRadius(const std::vector<float> &capsules_radius_) { capsules_radius = capsules_radius_; }
		void setMaxVel(const std::vector<float> &max_vel_) { max_vel = max_vel_; }
		void setMaxAcc(const std::vector<float> &max_acc_) { max_acc = max_acc_; }
		void setMaxJerk(const std::vector<float> &max_jerk_) { max_jerk = max_jerk_; }

		virtual void setState(std::shared_ptr<base::State> q) = 0;
		virtual std::shared_ptr<std::vector<KDL::Frame>> computeForwardKinematics(std::shared_ptr<base::State> q) = 0;
		virtual std::shared_ptr<base::State> computeInverseKinematics(const KDL::Rotation &R, const KDL::Vector &p,
																	  std::shared_ptr<base::State> q_init = nullptr) = 0;
		virtual std::shared_ptr<Eigen::MatrixXf> computeSkeleton(std::shared_ptr<base::State> q) = 0;
		virtual float computeStep(std::shared_ptr<base::State> q1, std::shared_ptr<base::State> q2, float d_c, float rho,
								  std::shared_ptr<Eigen::MatrixXf> skeleton) = 0;
		virtual float computeStep2(std::shared_ptr<base::State> q1, std::shared_ptr<base::State> q2, const std::vector<float> &d_c_profile,
						   		   const std::vector<float> &rho_profile, std::shared_ptr<Eigen::MatrixXf> skeleton) = 0;

	protected:
		std::string type;
		int num_DOFs;
		std::vector<std::unique_ptr<fcl::CollisionObjectf>> parts;
		std::vector<std::vector<float>> limits;
		std::shared_ptr<base::State> configuration;
		std::vector<float> capsules_radius;
		std::vector<float> max_vel;
		std::vector<float> max_acc;
		std::vector<float> max_jerk;
	};
}
#endif //RPMPL_ABSTRACTPLANNER_H
