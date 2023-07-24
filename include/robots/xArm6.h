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
		// const std::vector<float> radii = {0.0776, 0.0940, 0.0812, 0.0812, 0.0530, 0.0380}; 	// Radii of all enclosing cylinders
		const std::vector<float> radii = {0.08, 0.13, 0.12, 0.11, 0.05, 0.1}; 	// Radii of all enclosing cylinders. Gripper is attached.
		const int dim = 3;
	};

}
#endif //RPMPL_ABSTRACTPLANNER_H
