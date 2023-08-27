//
// Created by dinko on 07.02.22.
//

#ifndef RPMPL_ABSTRACTROBOT_H
#define RPMPL_ABSTRACTROBOT_H

#include <vector> 
#include <memory>
#include "State.h"
#include "fcl/fcl.h"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

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

		void setConfiguration(std::shared_ptr<base::State> configuration_) { configuration = configuration_; }

		virtual void setState(std::shared_ptr<base::State> q) = 0;
		virtual std::shared_ptr<std::vector<KDL::Frame>> computeForwardKinematics(std::shared_ptr<base::State> q) = 0;
		virtual std::shared_ptr<base::State> computeInverseKinematics(const KDL::Rotation &R, const KDL::Vector &p,
																	  std::shared_ptr<base::State> q_init = nullptr) = 0;
		virtual std::shared_ptr<Eigen::MatrixXf> computeSkeleton(std::shared_ptr<base::State> q) = 0;
		virtual float computeStep(std::shared_ptr<base::State> q1, std::shared_ptr<base::State> q2, float fi, 
								  std::shared_ptr<Eigen::MatrixXf> skeleton) = 0;

	protected:
		std::string type;
		int num_DOFs;
		std::vector<std::unique_ptr<fcl::CollisionObjectf>> parts;
		std::vector<std::vector<float>> limits;
		std::shared_ptr<base::State> configuration;
		std::vector<float> capsules_radius;
	};
}
#endif //RPMPL_ABSTRACTPLANNER_H
