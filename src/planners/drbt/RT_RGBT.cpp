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

    motion_validity = std::make_shared<planning::trajectory::MotionValidity>
                      (ss, RT_RGBTConfig::RESOLUTION_COLL_CHECK, RT_RGBTConfig::MAX_ITER_TIME, &path);

    switch (RT_RGBTConfig::TRAJECTORY_INTERPOLATION)
    {
    case planning::TrajectoryInterpolation::None:
        traj = nullptr;
        break;

    case planning::TrajectoryInterpolation::Spline:
        traj = std::make_shared<planning::trajectory::Trajectory>(ss, q_current, RT_RGBTConfig::MAX_ITER_TIME);
        break;
    }
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
        // Update environment and check if the collision occurs
        switch (RT_RGBTConfig::TRAJECTORY_INTERPOLATION)
        {
        case planning::TrajectoryInterpolation::None:
            is_valid = motion_validity->check(q_current, q_target);
            q_current = q_target;
            break;

        case planning::TrajectoryInterpolation::Spline:
            update();
            is_valid = motion_validity->check(traj->traj_points_current_iter);
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

    std::cout << "TIME'S UP when finding q_target!!! \n";
}

/// @brief Update a current state of the robot using 'traj->spline_current'.
/// Compute a new spline 'traj->spline_next'.
/// Move 'q_current' towards 'q_target' while following 'traj->spline_next'.
/// 'q_current' will be updated to a robot position from the end of current iteration.
/// @note The new trajectory will be computed in a way that all constraints on robot's maximal velocity, 
/// acceleration and jerk are surely always satisfied.
void planning::drbt::RT_RGBT::update()
{
    auto time_update { std::chrono::steady_clock::now() };     // Start the algorithm clock
    
    traj->spline_current = traj->spline_next;
    float t_traj_max { TrajectoryConfig::MAX_TIME_COMPUTE_REGULAR };
    float t_iter { getElapsedTime(time_iter_start) };
    if (RT_RGBTConfig::MAX_ITER_TIME - t_iter < t_traj_max)
        t_traj_max = RT_RGBTConfig::MAX_ITER_TIME - t_iter;
    
    float t_iter_remain { RT_RGBTConfig::MAX_ITER_TIME - t_iter - t_traj_max };
    float t_traj_current { traj->spline_current->getTimeEnd() + t_iter + t_traj_max };

    traj->spline_current->setTimeBegin(traj->spline_current->getTimeEnd());
    traj->spline_current->setTimeCurrent(t_traj_current);
    // ----------------------------------------------------------------------------------------- //
    
    bool traj_computed { false };
    float t_traj_remain { t_traj_max - (getElapsedTime(time_iter_start) - t_iter) };
    // std::cout << "t_traj_remain: " << t_traj_remain * 1e3 << " [ms] \n";
    
    if (t_traj_remain > 0)
    {
        Eigen::VectorXf current_pos { traj->spline_current->getPosition(t_traj_current) };
        Eigen::VectorXf current_vel { traj->spline_current->getVelocity(t_traj_current) };
        Eigen::VectorXf current_acc { traj->spline_current->getAcceleration(t_traj_current) };

        // std::cout << "Curr. pos: " << current_pos.transpose() << "\n";
        // std::cout << "Curr. vel: " << current_vel.transpose() << "\n";
        // std::cout << "Curr. acc: " << current_acc.transpose() << "\n";

        traj->setCurrentState(q_current);
        traj->setTargetState(q_target);
        // std::cout << "q_target:       " << q_target << "\n";

        traj_computed = traj->computeRegular(current_pos, current_vel, current_acc, t_iter_remain, t_traj_remain, true);
        
        // Try to compute again, but with ZERO final velocity
        t_traj_remain = t_traj_max - (getElapsedTime(time_iter_start) - t_iter);
        if (traj_computed && traj->spline_next->getTimeFinal() < t_iter_remain && !traj->spline_next->getIsZeroFinalVel() && t_traj_remain > 0)
            traj_computed = traj->computeRegular(current_pos, current_vel, current_acc, t_iter_remain, t_traj_remain, false);
    }
    
    if (traj_computed)
    {
        // std::cout << "New trajectory is computed! \n";
        traj->spline_current->setTimeEnd(t_traj_current);
        traj->spline_next->setTimeEnd(t_iter_remain);
    }
    else
    {
        // std::cout << "Continuing with the previous trajectory! \n";
        traj->spline_next = traj->spline_current;
        traj->spline_next->setTimeEnd(t_traj_current + t_iter_remain);
    }

    q_current = ss->getNewState(traj->spline_next->getPosition(traj->spline_next->getTimeEnd()));   // Current robot position at the end of iteration
    // std::cout << "q_current: " << q_current << "\n";

    // If 'q_current' is far away from 'q_target', do not change 'q_target'
    compute_new_target_state = (ss->getNorm(q_current, q_target) > 
        traj->spline_next->getVelocity(traj->spline_next->getTimeEnd()).norm() / ss->robot->getMaxVel().norm() * TrajectoryConfig::MAX_RADIUS) ?
            false : true;

    planner_info->addRoutineTime(getElapsedTime(time_update, planning::TimeUnit::ms), 0);
    // std::cout << "Elapsed time for trajectory computing: " << (getElapsedTime(time_update) - t_iter) * 1e6 << " [us] \n";

    // ----------------------------------------------------------------------------------------- //
    // Store trajectory points from the current iteration to be validated later within 'MotionValidity'
    traj->traj_points_current_iter.clear();
    size_t num_checks1 = std::ceil((traj->spline_current->getTimeCurrent() - traj->spline_current->getTimeBegin()) * TrajectoryConfig::NUM_VALIDITY_POINTS_CHECK / RT_RGBTConfig::MAX_ITER_TIME);
    float delta_time1 { (traj->spline_current->getTimeCurrent() - traj->spline_current->getTimeBegin()) / num_checks1 };
    float t { 0 };
    for (size_t num_check = 1; num_check <= num_checks1; num_check++)
    {
        t = traj->spline_current->getTimeBegin() + num_check * delta_time1;
        traj->traj_points_current_iter.emplace_back(traj->spline_current->getPosition(t));
    }
    
    size_t num_checks2 { TrajectoryConfig::NUM_VALIDITY_POINTS_CHECK - num_checks1 };
    float delta_time2 { (traj->spline_next->getTimeEnd() - traj->spline_next->getTimeCurrent()) / num_checks2 };
    for (size_t num_check = 1; num_check <= num_checks2; num_check++)
    {
        t = traj->spline_next->getTimeCurrent() + num_check * delta_time2;
        traj->traj_points_current_iter.emplace_back(traj->spline_next->getPosition(t));
    }
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
