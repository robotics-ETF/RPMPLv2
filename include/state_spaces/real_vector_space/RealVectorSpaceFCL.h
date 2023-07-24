//
// Created by dinko on 14.2.22.
//

#ifndef RPMPL_REALVECTORSPACEFCL_H
#define RPMPL_REALVECTORSPACEFCL_H

#include <ostream>
#include <memory>
#include <vector>
#include "StateSpace.h"
#include "RealVectorSpaceState.h"
#include "RealVectorSpace.h"

namespace base
{
	class RealVectorSpaceFCL : public base::RealVectorSpace
	{
	public:
		std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager_robot;
		std::shared_ptr<fcl::BroadPhaseCollisionManagerf> collision_manager_env;

		RealVectorSpaceFCL(int dimensions_, const std::shared_ptr<robots::AbstractRobot> robot_, const std::shared_ptr<env::Environment> env_);
		~RealVectorSpaceFCL();

		std::shared_ptr<fcl::BroadPhaseCollisionManagerf> getCollisionManagerRobot() { return collision_manager_robot; }
		std::shared_ptr<fcl::BroadPhaseCollisionManagerf> getCollisionManagerEnv() { return collision_manager_env; }
		
		bool isValid(const std::shared_ptr<base::State> q) override;
		float computeDistance(const std::shared_ptr<base::State> q) override;
		std::tuple<float, std::shared_ptr<std::vector<Eigen::MatrixXf>>> computeDistanceAndPlanes(const std::shared_ptr<base::State> q) override;
		void prepareCollisionManager(const std::shared_ptr<base::State> q);
	};
}
#endif //RPMPL_REALVECTORSPACE_H
