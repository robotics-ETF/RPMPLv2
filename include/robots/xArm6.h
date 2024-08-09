//
// Created by dinko on 07.02.22.
// Modified by nermin on 05.09.22.
//

#ifndef RPMPL_XARM6_H
#define RPMPL_XARM6_H

#include "AbstractRobot.h"
#include "RealVectorSpace.h"
#include "RRTConnectConfig.h"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <urdf/model.h>
#include <glog/logging.h>
#include <stl_reader.h>

namespace robots
{
	class xArm6 : public AbstractRobot
	{
	public:
		xArm6(const std::string &robot_desc, float gripper_length_ = 0, size_t ground_included_ = 0);
		~xArm6();

		const KDL::Tree &getRobotTree() const { return robot_tree; }

		void setState(std::shared_ptr<base::State> q) override;
		void setCapsulesRadius(const std::vector<float> &capsules_radius_) override;
		std::shared_ptr<std::vector<KDL::Frame>> computeForwardKinematics(std::shared_ptr<base::State> q) override;
		std::shared_ptr<base::State> computeInverseKinematics(const KDL::Rotation &R, const KDL::Vector &p, 
															  std::shared_ptr<base::State> q_init = nullptr) override;
		std::shared_ptr<Eigen::MatrixXf> computeSkeleton(std::shared_ptr<base::State> q) override;
		std::shared_ptr<Eigen::MatrixXf> computeEnclosingRadii(const std::shared_ptr<base::State> q) override;
		bool checkSelfCollision(const std::shared_ptr<base::State> q1, std::shared_ptr<base::State> &q2) override;
		bool checkSelfCollision(const std::shared_ptr<base::State> q) override;

	private:
		bool checkSelfCollision(const std::shared_ptr<base::State> q, std::vector<bool> &skip_checking);
		float computeCapsulesDistance(const std::shared_ptr<base::State> q, size_t link1_idx, size_t link2_idx);
		bool checkRealSelfCollision(const std::shared_ptr<base::State> q, size_t link1_idx, size_t link2_idx);
		bool checkCollisionFCL(const std::unique_ptr<fcl::CollisionObjectf> &obj1, const std::unique_ptr<fcl::CollisionObjectf> &obj2);
		fcl::Transform3f KDL2fcl(const KDL::Frame &in);
		KDL::Frame fcl2KDL(const fcl::Transform3f &in);
		void test();
	
		std::vector<KDL::Frame> init_poses;
		KDL::Tree robot_tree;
		KDL::Chain robot_chain;
		std::vector<float> capsules_radius_new;
	};
}

#endif //RPMPL_ABSTRACTPLANNER_H