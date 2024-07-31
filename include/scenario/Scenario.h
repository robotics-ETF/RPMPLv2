//
// Created by dinko on 01.02.22.
// Modified by nermin on 09.02.24.
//

#ifndef RPMPL_SCENARIO_H
#define RPMPL_SCENARIO_H

#include <vector> 
#include <memory>
#include <string>
#include <fcl/fcl.h>

#include "Environment.h"
#include "RealVectorSpaceFCL.h"
#include "RealVectorSpaceState.h"
#include "Planar2DOF.h"
#include "Planar10DOF.h"
#include "xArm6.h"
#include <yaml-cpp/yaml.h>

namespace scenario
{
	class Scenario
	{
	public:
		Scenario(const std::string &config_file_path, const std::string &root_path = "");
        Scenario(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> q_start_, std::shared_ptr<base::State> q_goal_);
        ~Scenario() {}
        
        inline std::shared_ptr<robots::AbstractRobot> getRobot() const { return ss->robot; }
        inline std::shared_ptr<env::Environment> getEnvironment() const { return ss->env; }
        inline std::shared_ptr<base::StateSpace> getStateSpace() const { return ss; }
        inline std::shared_ptr<base::State> getStart() const { return q_start; }
        inline std::shared_ptr<base::State> getGoal() const { return q_goal; }
        inline base::StateSpaceType getStateSpaceType() const { return state_space_type; }
        inline size_t getNumDimensions() const { return num_dimensions; }

        inline void setEnvironment(const std::shared_ptr<env::Environment> env_) { ss->env = env_; }
        inline void setStart(const std::shared_ptr<base::State> q_start_) { q_start = q_start_; }
        inline void setGoal(const std::shared_ptr<base::State> q_goal_) { q_goal = q_goal_; }

	private:
        std::shared_ptr<base::StateSpace> ss;
        std::shared_ptr<base::State> q_start;
        std::shared_ptr<base::State> q_goal;
        base::StateSpaceType state_space_type;
        size_t num_dimensions;         // Number of dimensions of the state-space (also 'num_DOFs' of the used robot)
	};
}

#endif //RPMPL_SCENARIO_H