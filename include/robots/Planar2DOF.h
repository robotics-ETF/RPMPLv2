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
		Planar2DOF(const std::string &robot_desc, size_t num_DOFs_ = 2);
		~Planar2DOF();

		const KDL::Tree &getRobotTree() const { return robot_tree; }

		void setState(const std::shared_ptr<base::State> q) override;
		std::shared_ptr<std::vector<KDL::Frame>> computeForwardKinematics(const std::shared_ptr<base::State> q) override;
		std::shared_ptr<base::State> computeInverseKinematics(const KDL::Rotation &R, const KDL::Vector &p, 
															  const std::shared_ptr<base::State> q_init = nullptr) override;
		std::shared_ptr<Eigen::MatrixXf> computeSkeleton(const std::shared_ptr<base::State> q) override;
		float computeStep(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, float d_c, 
			float rho, const std::shared_ptr<Eigen::MatrixXf> skeleton) override;
		float computeStep2(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, 
			const std::vector<float> &d_c_profile, const std::vector<float> &rho_profile, const std::shared_ptr<Eigen::MatrixXf> skeleton) override;

	private:
		fcl::Transform3f KDL2fcl(const KDL::Frame &in);
		KDL::Frame fcl2KDL(const fcl::Transform3f &in);
		fcl::Vector3f transformPoint(fcl::Vector3f& v, fcl::Transform3f t);
		void test(const std::shared_ptr<env::Environment> env, const std::shared_ptr<base::State> q);
		
		std::vector<KDL::Frame> init_poses;
		KDL::Tree robot_tree;
		KDL::Chain robot_chain;
	};

}
#endif //RPMPL_ABSTRACTPLANNER_H
