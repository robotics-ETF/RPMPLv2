//
// Created by dinko on 07.02.22.
//

#ifndef RPMPL_PLANAR2DOF_H
#define RPMPL_PLANAR2DOF_H

#include "AbstractRobot.h"
#include "Environment.h"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

namespace robots
{
	class Planar2DOF : public AbstractRobot
	{
	public:
		Planar2DOF(std::string robot_desc, int num_DOFs_ = 2);
		~Planar2DOF();

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
		fcl::Vector3f transformPoint(fcl::Vector3f& v, fcl::Transform3f t);
		void test(std::shared_ptr<env::Environment> env, std::shared_ptr<base::State> q);
		
		std::vector<KDL::Frame> init_poses;
		KDL::Tree robot_tree;
		KDL::Chain robot_chain;
	};

}
#endif //RPMPL_ABSTRACTPLANNER_H
