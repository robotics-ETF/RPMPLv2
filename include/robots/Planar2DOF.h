//
// Created by dinko on 07.02.22.
//

#ifndef RPMPL_PLANAR2DOF_H
#define RPMPL_PLANAR2DOF_H

#include "AbstractRobot.h"
#include "Environment.h"
#include <memory>
#include <vector>
#include <string>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

namespace robots
{
	class Planar2DOF : public AbstractRobot
	{
	public:
		Planar2DOF(std::string robot_desc, int num_DOFs = 2);
		~Planar2DOF();

		const KDL::Tree& getRobotTree() const { return robot_tree; }
		const std::vector<std::unique_ptr<fcl::CollisionObject<float>>> &getParts() const override { return parts; }
		const std::vector<std::vector<float>> &getLimits() const override { return limits; }
		const int getDimensions() override { return dim; }
		const float getRadius(int num) override { return radii[num]; }

		void setState(std::shared_ptr<base::State> q) override;

		std::shared_ptr<std::vector<KDL::Frame>> computeForwardKinematics(std::shared_ptr<base::State> q) override;
		std::shared_ptr<Eigen::MatrixXf> computeSkeleton(std::shared_ptr<base::State> q) override;
		float computeStep(std::shared_ptr<base::State> q1, std::shared_ptr<base::State> q2, float fi, 
						  std::shared_ptr<Eigen::MatrixXf> skeleton) override;

		void test(std::shared_ptr<env::Environment> env, std::shared_ptr<base::State> q);

	protected:
		fcl::Transform3f KDL2fcl(const KDL::Frame &in);
		KDL::Frame fcl2KDL(const fcl::Transform3f &in);
		fcl::Vector3f transformPoint(fcl::Vector3f& v, fcl::Transform3f t);
		
		std::vector<std::unique_ptr<fcl::CollisionObject<float>>> parts;
		std::vector<KDL::Frame> init_poses;
		KDL::Tree robot_tree;
		KDL::Chain robot_chain;
		std::vector<std::vector<float>> limits;
		std::vector<float> radii;		// Radii of all enclosing cylinders
		const int dim = 2;
	};

}
#endif //RPMPL_ABSTRACTPLANNER_H
