//
// Created by nermin on 25.09.25.
//

#ifndef RPMPL_RTRGBT_H
#define RPMPL_RTRGBT_H

#include "RT_RGBTConfig.h"
#include "RGBTConnect.h"
#include "UpdatingState.h"
#include "MotionValidity.h"
#include "Trajectory.h"
#include "TrajectoryRuckig.h"

// #include <glog/log_severity.h>
// #include <glog/logging.h>
// WARNING: You need to be very careful with using LOG(INFO) for console output, due to a possible "stack smashing detected" error.
// If you get this error, just use std::cout for console output.

namespace planning::drbt
{
	class RT_RGBT : public planning::rbt::RGBTConnect
	{
	public:
		RT_RGBT(const std::shared_ptr<base::StateSpace> ss_);
		RT_RGBT(const std::shared_ptr<base::StateSpace> ss_, 
				const std::shared_ptr<base::State> q_start_, const std::shared_ptr<base::State> q_goal_);
		~RT_RGBT();
		
		bool solve() override;
		void computeTargetState();
		bool checkTerminatingCondition(base::State::Status status) override;
		void outputPlannerData(const std::string &filename, bool output_states_and_paths = true, bool append_output = false) const override;
		
	protected:
		std::shared_ptr<base::State> q_current;            								// Current robot configuration
		std::shared_ptr<base::State> q_target;          		 						// Target (next) robot configuration
        std::shared_ptr<planning::trajectory::AbstractTrajectory> traj;             	// Trajectory which is generated from 'q_current' towards 'q_target'
        std::shared_ptr<planning::trajectory::UpdatingState> updating_state;        	// Class for updating current state
        std::shared_ptr<planning::trajectory::MotionValidity> motion_validity;      	// Class for checking validity of motion
        float max_edge_length;															// Distance between 'q_current' and 'q_target'
		bool compute_new_target_state;													// Whether to compute 'q_target'
	};
}

#endif //RPMPL_RTRGBT_H