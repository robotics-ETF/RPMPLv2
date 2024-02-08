//
// Created by dinko on 07.02.22.
//

#ifndef RPMPL_XARM6_H
#define RPMPL_XARM6_H

#include "AbstractRobot.h"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

namespace robots
{
	class xArm6 : public AbstractRobot
	{
	public:
		xArm6(std::string robot_desc, float gripper_length_ = 0, bool table_included_ = true);
		~xArm6();

		const KDL::Tree &getRobotTree() const { return robot_tree; }

		void setState(std::shared_ptr<base::State> q) override;
		std::shared_ptr<std::vector<KDL::Frame>> computeForwardKinematics(std::shared_ptr<base::State> q) override;
		std::shared_ptr<base::State> computeInverseKinematics(const KDL::Rotation &R, const KDL::Vector &p, 
															  std::shared_ptr<base::State> q_init = nullptr) override;
		std::shared_ptr<Eigen::MatrixXf> computeSkeleton(std::shared_ptr<base::State> q) override;
		float computeStep(std::shared_ptr<base::State> q1, std::shared_ptr<base::State> q2, float d_c, float rho, 
						  std::shared_ptr<Eigen::MatrixXf> skeleton) override;
		float computeStep2(std::shared_ptr<base::State> q1, std::shared_ptr<base::State> q2, const std::vector<float> &d_c_profile,
						   const std::vector<float> &rho_profile, std::shared_ptr<Eigen::MatrixXf> skeleton) override;

	private:
		fcl::Transform3f KDL2fcl(const KDL::Frame &in);
		KDL::Frame fcl2KDL(const fcl::Transform3f &in);
		float getEnclosingRadius(std::shared_ptr<Eigen::MatrixXf> skeleton, int j_start, int j_proj);
		void test();
	
		std::vector<KDL::Frame> init_poses;
		KDL::Tree robot_tree;
		KDL::Chain robot_chain;
		float gripper_length;
		bool table_included;
	};

}
#endif //RPMPL_ABSTRACTPLANNER_H
