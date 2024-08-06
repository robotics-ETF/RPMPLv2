//
// Created by dinko on 14.2.22.
//

#ifndef RPMPL_REALVECTORSPACEFCL_H
#define RPMPL_REALVECTORSPACEFCL_H

#include "RealVectorSpace.h"
#include "RealVectorSpaceConfig.h"
#include "xArm6.h"

namespace base
{
	class RealVectorSpaceFCL : public base::RealVectorSpace
	{
	public:
		std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager_robot;
		std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager_env;

		RealVectorSpaceFCL(size_t num_dimensions_, const std::shared_ptr<robots::AbstractRobot> robot_, 
						   const std::shared_ptr<env::Environment> env_);
		~RealVectorSpaceFCL();

		std::shared_ptr<fcl::BroadPhaseCollisionManagerf> getCollisionManagerRobot() const { return collision_manager_robot; }
		std::shared_ptr<fcl::BroadPhaseCollisionManagerf> getCollisionManagerEnv() const { return collision_manager_env; }
		
		bool isValid(const std::shared_ptr<base::State> q) override;
		float computeDistance(const std::shared_ptr<base::State> q, bool compute_again) override;
	};
}

#endif //RPMPL_REALVECTORSPACE_H