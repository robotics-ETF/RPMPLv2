#include "RT_RGBT.h"

planning::drbt::RT_RGBT::RT_RGBT(const std::shared_ptr<base::StateSpace> ss_) : RGBTConnect(ss_) 
{
    planner_type = planning::PlannerType::RT_RGBT;
}

planning::drbt::RT_RGBT::RT_RGBT(const std::shared_ptr<base::StateSpace> ss_, const std::shared_ptr<base::State> q_start_,
                                 const std::shared_ptr<base::State> q_goal_) : RGBTConnect(ss_)
{
	// std::cout << "Initializing RT_RGBT planner... \n";
    q_start = q_start_;
    q_goal = q_goal_;
    compute_new_target_state = true;

    planner_type = planning::PlannerType::RT_RGBT;
    if (!ss->isValid(q_start) || ss->robot->checkSelfCollision(q_start))
		throw std::domain_error("Start position is invalid!");
    
    q_current = q_start;
    q_target = nullptr;

    planner_info->setNumStates(1);
	planner_info->setNumIterations(0);
    path.emplace_back(q_start);     // State 'q_start' is added to the realized path
    max_edge_length = ss->robot->getMaxVel().norm() * RT_RGBTConfig::MAX_ITER_TIME;

    switch (RT_RGBTConfig::TRAJECTORY_INTERPOLATION)
    {
    case planning::TrajectoryInterpolation::None:
        traj = nullptr;
        break;

    case planning::TrajectoryInterpolation::Spline:
        traj = std::make_shared<planning::trajectory::Trajectory>
        (
            ss, 
            planning::trajectory::State(q_current->getCoord()), 
            RT_RGBTConfig::MAX_ITER_TIME
        );
        break;
    
    case planning::TrajectoryInterpolation::Ruckig:
        traj = std::make_shared<planning::trajectory::TrajectoryRuckig>
        (
            ss, 
            planning::trajectory::State(q_current->getCoord()), 
            RT_RGBTConfig::MAX_ITER_TIME
        );
        break;
    }

    updating_state = std::make_shared<planning::trajectory::UpdatingState>
                     (ss, RT_RGBTConfig::TRAJECTORY_INTERPOLATION, RT_RGBTConfig::MAX_ITER_TIME);
    updating_state->setTrajectory(traj);

    motion_validity = std::make_shared<planning::trajectory::MotionValidity>
                      (ss, RT_RGBTConfig::RESOLUTION_COLL_CHECK, RT_RGBTConfig::MAX_ITER_TIME, &path);

	// std::cout << "RT_RGBT planner initialized! \n";
}

planning::drbt::RT_RGBT::~RT_RGBT()
{
	path.clear();
}

bool planning::drbt::RT_RGBT::solve()
{
    time_alg_start = std::chrono::steady_clock::now();     // Start the algorithm clock
    time_iter_start = time_alg_start;

    // Initial iteration: Obtaining an inital path using specified static planner
    // std::cout << "Iteration: " << planner_info->getNumIterations() << "\n";
    planner_info->setNumIterations(planner_info->getNumIterations() + 1);
    planner_info->addIterationTime(getElapsedTime(time_iter_start));
    // std::cout << "----------------------------------------------------------------------------------------\n";

	base::State::Status status { base::State::Status::None };
    bool is_valid { true };

    while (true)
    {
        // std::cout << "Iteration: " << planner_info->getNumIterations() << "\n";
        time_iter_start = std::chrono::steady_clock::now();     // Start the iteration clock

        // ------------------------------------------------------------------------------- //
        // Possibly compute a new target state
        if (compute_new_target_state)
        {
            computeTargetState();
            // std::cout << "q_target: " << q_target << "\n";
        }
        // else std::cout << "Using previous q_target \n";

        // ------------------------------------------------------------------------------- //
        // Compute a trajectory and update current state
        auto time_updateCurrentState { std::chrono::steady_clock::now() };
        updating_state->setNonZeroFinalVel(!ss->isEqual(q_target, q_goal));
        updating_state->setTimeIterStart(time_iter_start);
        updating_state->update(q_current, q_current, q_target, status);
        planner_info->addRoutineTime(getElapsedTime(time_updateCurrentState, planning::TimeUnit::us), 0);

        // ------------------------------------------------------------------------------- //
        // Update environment and check if the collision occurs
        switch (RT_RGBTConfig::TRAJECTORY_INTERPOLATION)
        {
        case planning::TrajectoryInterpolation::None:
            is_valid = motion_validity->check(q_current, q_target);
            break;

        default:
            is_valid = motion_validity->check(traj->getTrajPointCurrentIter());
            break;
        }

        if (!is_valid)
        {
            std::cout << "*************** Collision has been occurred!!! *************** \n";
            planner_info->setSuccessState(false);
            planner_info->setPlanningTime(planner_info->getIterationTimes().back());
            return false;
        }

        // ------------------------------------------------------------------------------- //
        // Planner info and terminating condition
        planner_info->setNumIterations(planner_info->getNumIterations() + 1);
        planner_info->addIterationTime(getElapsedTime(time_alg_start));
        if (checkTerminatingCondition(status))
            return planner_info->getSuccessState();

        // std::cout << "----------------------------------------------------------------------------------------\n";
    }
}

// Get a random state 'q_target' with uniform distribution, which is centered around 'q_current'.
// The edge from 'q_current' to 'q_target' is collision-free.
void planning::drbt::RT_RGBT::computeTargetState()
{
    auto time_compute_target = std::chrono::steady_clock::now();
	std::shared_ptr<base::State> q_rand { nullptr };
    base::State::Status status { base::State::Status::None };

    while (getElapsedTime(time_compute_target) < RT_RGBTConfig::MAX_ITER_TIME / 2)
    {
        q_rand = (float(rand()) / RAND_MAX < RT_RGBTConfig::GOAL_PROBABILITY) ?
                 q_goal :
	             ss->getRandomState();

		q_rand = std::get<1>(ss->interpolateEdge2(q_current, q_rand, float(rand()) / RAND_MAX * max_edge_length));
		q_rand = ss->pruneEdge(q_current, q_rand);
        ss->computeDistance(q_current, true);
        tie(status, q_rand) = extendGenSpine(q_current, q_rand, true);
        // std::cout << "d_c: " << q_current->getDistance() << "\n";
        // std::cout << "q_rand: " << q_rand << "\n";

        if (status != base::State::Status::Trapped)
        {
            q_target = q_rand;
            return;
        }
    }

    std::cout << "TIME'S UP when finding a new target state!!! Retaining the existing one.\n";
}

bool planning::drbt::RT_RGBT::checkTerminatingCondition([[maybe_unused]] base::State::Status status)
{
    float time_current { getElapsedTime(time_alg_start) };
    // std::cout << "Time elapsed: " << time_current * 1e3 << " [ms] \n";

    if (ss->isEqual(q_current, q_goal))
    {
        std::cout << "Goal configuration has been successfully reached! \n";
		planner_info->setSuccessState(true);
        planner_info->setPlanningTime(time_current);
        return true;
    }
	
    if (time_current >= RT_RGBTConfig::MAX_PLANNING_TIME)
	{
        std::cout << "Maximal planning time has been reached! \n";
		planner_info->setSuccessState(false);
        planner_info->setPlanningTime(time_current);
		return true;
	}
    
    if (planner_info->getNumIterations() >= RT_RGBTConfig::MAX_NUM_ITER)
	{
        std::cout << "Maximal number of iterations has been reached! \n";
		planner_info->setSuccessState(false);
        planner_info->setPlanningTime(time_current);
		return true;
	}

	return false;
}

void planning::drbt::RT_RGBT::outputPlannerData(const std::string &filename, bool output_states_and_paths, bool append_output) const
{
	std::ofstream output_file {};
	std::ios_base::openmode mode { std::ofstream::out };
	if (append_output)
		mode = std::ofstream::app;

	output_file.open(filename, mode);
	if (output_file.is_open())
	{
		output_file << "Space Type:      " << ss->getStateSpaceType() << std::endl;
		output_file << "Dimensionality:  " << ss->num_dimensions << std::endl;
		output_file << "Planner type:    " << planner_type << std::endl;
		output_file << "Planner info:\n";
		output_file << "\t Succesfull:           " << (planner_info->getSuccessState() ? "yes" : "no") << std::endl;
		output_file << "\t Number of iterations: " << planner_info->getNumIterations() << std::endl;
		output_file << "\t Planning time [s]:    " << planner_info->getPlanningTime() << std::endl;
		if (output_states_and_paths)
		{
			if (path.size() > 0)
			{
				output_file << "Path:" << std::endl;
				for (size_t i = 0; i < path.size(); i++)
					output_file << path.at(i) << std::endl;
			}
		}
		output_file << std::string(25, '-') << std::endl;		
		output_file.close();
	}
	else
		throw "Cannot open file"; // std::something exception perhaps?
}
