//
// Created by dinko on 07.02.22.
//

#ifndef RPMPL_XARM6_H
#define RPMPL_XARM6_H

#include "AbstractRobot.h"
#include <memory>
#include <vector>
#include <string>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

namespace robots
{
	class xARM6 : public AbstractRobot
	{
	public:
		xARM6(std::string robot_desc);
		~xARM6();

		const KDL::Tree &getRobotTree() const { return robot_tree; }
		const std::vector<std::unique_ptr<fcl::CollisionObject<float>>> &getParts() const override { return parts; }
		const std::vector<std::vector<float>> &getLimits() const override { return limits; }
		const int getDimensions() override { return dim; }
		const float getRadius(int num) override { return radii[num]; }

		void setState(std::shared_ptr<base::State> q_) override;

		std::shared_ptr<std::vector<KDL::Frame>> computeForwardKinematics(std::shared_ptr<base::State> q) override;
		std::shared_ptr<base::State> computeInverseKinematics(const KDL::Rotation &R, const KDL::Vector &p, 
			std::shared_ptr<base::State> q_init = nullptr) override;
		std::shared_ptr<Eigen::MatrixXf> computeSkeleton(std::shared_ptr<base::State> q) override;
		float computeStep(std::shared_ptr<base::State> q1, std::shared_ptr<base::State> q2, float fi,
						  std::shared_ptr<Eigen::MatrixXf> skeleton) override;
		void test();

	private:
		fcl::Transform3f KDL2fcl(const KDL::Frame &in);
		KDL::Frame fcl2KDL(const fcl::Transform3f &in);
		float getEnclosingRadius(std::shared_ptr<Eigen::MatrixXf> skeleton, int j_start, int j_proj);
	
		std::vector<std::unique_ptr<fcl::CollisionObject<float>>> parts;
		std::vector<KDL::Frame> init_poses;
		KDL::Tree robot_tree;
		KDL::Chain robot_chain;
		std::vector<std::vector<float>> limits;
		const int dim = 3;
		// const std::vector<float> radii = {0.047, 0.12, 0.11, 0.09, 0.05, 0.0380}; 	// Radii of all enclosing cylinders
		const std::vector<float> radii = {0.047, 0.12, 0.11, 0.09, 0.05, 0.075}; 	// Radii of all enclosing cylinders. Gripper is attached.
		const bool gripper_attached = true;
		const float gripper_length = 0.15; 		// Gripper length is 0.15 [m]
	};

}
#endif //RPMPL_ABSTRACTPLANNER_H
