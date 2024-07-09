//
// Created by nermin on 13.04.22.
//

#include "DRGBT.h"

// #include <glog/log_severity.h>
// #include <glog/logging.h>
// WARNING: You need to be very careful with LOG(INFO) for console output, due to a possible "stack smashing detected" error.
// If you get this error, just use std::cout for console output.

planning::drbt::DRGBT::DRGBT(const std::shared_ptr<base::StateSpace> ss_) : RGBTConnect(ss_) 
{
    planner_type = planning::PlannerType::DRGBT;
}

planning::drbt::DRGBT::DRGBT(const std::shared_ptr<base::StateSpace> ss_, const std::shared_ptr<base::State> q_start_,
                             const std::shared_ptr<base::State> q_goal_) : RGBTConnect(ss_)
{
	// std::cout << "Initializing DRGBT planner... \n";
    planner_type = planning::PlannerType::DRGBT;
    q_start = q_start_;
    q_goal = q_goal_;
	if (!ss->isValid(q_start) || ss->robot->checkSelfCollision(q_start))
		throw std::domain_error("Start position is invalid!");
    
    q_current = q_start;
    q_previous = q_current;
    q_target = q_current;
    q_next = std::make_shared<planning::drbt::HorizonState>(q_current, 0, q_current);
    q_next_previous = q_next;

    d_c = INFINITY;
    d_max_mean = 0;
    num_lateral_states = 2 * ss->num_dimensions - 2;
    horizon_size = DRGBTConfig::INIT_HORIZON_SIZE + num_lateral_states;
    replanning = false;
    status = base::State::Status::Reached;
    planner_info->setNumStates(1);
	planner_info->setNumIterations(0);
    path.emplace_back(q_start);     // State 'q_start' is added to the realized path
    max_edge_length = ss->robot->getMaxVel().norm() * DRGBTConfig::MAX_ITER_TIME;

    all_robot_vel_same = true;
    for (size_t i = 1; i < ss->num_dimensions; i++)
    {
        if (std::abs(ss->robot->getMaxVel(i) - ss->robot->getMaxVel(i-1)) > RealVectorSpaceConfig::EQUALITY_THRESHOLD)
        {
            all_robot_vel_same = false;
            break;
        }
    }

    max_obs_vel = 0;
    for (size_t i = 0; i < ss->env->getNumObjects(); i++)
    {
        if (ss->env->getObject(i)->getMaxVel() > max_obs_vel)
            max_obs_vel = ss->env->getObject(i)->getMaxVel();
    }

    if (DRGBTConfig::TRAJECTORY_INTERPOLATION == planning::TrajectoryInterpolation::Spline)
    {
        spline_current = std::make_shared<planning::trajectory::Spline5>(ss->robot, q_current->getCoord());
        spline_next = spline_current;
        max_num_iter_spline_next = all_robot_vel_same ? 
            std::ceil(std::log2(2 * ss->robot->getMaxVel(0) / Spline5Config::FINAL_VELOCITY_STEP)) :
            std::ceil(std::log2(2 * ss->robot->getMaxVel().maxCoeff() / Spline5Config::FINAL_VELOCITY_STEP));
    }

	// std::cout << "DRGBT planner initialized! \n";
}

planning::drbt::DRGBT::~DRGBT()
{
	path.clear();
    horizon.clear();
    predefined_path.clear();
}

bool planning::drbt::DRGBT::solve()
{
    time_alg_start = std::chrono::steady_clock::now();     // Start the algorithm clock
    time_iter_start = time_alg_start;

    // Initial iteration: Obtaining an inital path using specified static planner
    // std::cout << "Iteration: " << planner_info->getNumIterations() << "\n";
    // std::cout << "Obtaining an inital path... \n";
    replan(DRGBTConfig::MAX_ITER_TIME);
    planner_info->setNumIterations(planner_info->getNumIterations() + 1);
    planner_info->addIterationTime(getElapsedTime(time_iter_start));
    // std::cout << "----------------------------------------------------------------------------------------\n";

    while (true)
    {
        // std::cout << "Iteration: " << planner_info->getNumIterations() << "\n";
        // std::cout << "TASK 1: Computing next configuration... \n";
        time_iter_start = std::chrono::steady_clock::now();     // Start the iteration clock
        
        // ------------------------------------------------------------------------------- //
        // Since the environment may change, a new distance is required!
        auto time_computeDistance { std::chrono::steady_clock::now() };
        d_c = ss->computeDistance(q_current, true);     // ~ 1 [ms]
        planner_info->addRoutineTime(getElapsedTime(time_computeDistance, planning::TimeUnit::us), 1);
        // std::cout << "d_c: " << d_c << "\n";

        // ------------------------------------------------------------------------------- //
        if (status != base::State::Status::Advanced)
            generateHorizon();          // ~ 2 [us]
            
        updateHorizon();                // ~ 10 [us]
        generateGBur();                 // ~ 10 [ms] Time consuming routine... 
        computeNextState();             // ~ 1 [us]
        
        switch (DRGBTConfig::TRAJECTORY_INTERPOLATION)
        {
        case planning::TrajectoryInterpolation::Spline:
            updateCurrentState(false);  // ~ 1 [ms]
            break;
        
        case planning::TrajectoryInterpolation::None:
            updateCurrentState();       // ~ 1 [us]
            break;
        }

        // std::cout << "Time elapsed: " << getElapsedTime(time_iter_start, planning::TimeUnit::ms) << " [ms] \n";

        // ------------------------------------------------------------------------------- //
        // Replanning procedure assessment
        if (whetherToReplan())
        {
            // std::cout << "TASK 2: Replanning... \n";
            replan(DRGBTConfig::MAX_ITER_TIME - getElapsedTime(time_iter_start));
            // std::cout << "Time elapsed: " << getElapsedTime(time_iter_start, planning::TimeUnit::ms) << " [ms] \n";
        }
        // else
        //     std::cout << "Replanning is not required! \n";

        // ------------------------------------------------------------------------------- //
        // Checking the real-time execution
        // float time_iter_remain { DRGBTConfig::MAX_ITER_TIME * 1e3 - getElapsedTime(time_iter_start, planning::TimeUnit::ms) };
        // std::cout << "Remaining iteration time is " << time_iter_remain << " [ms] \n";
        // if (time_iter_remain < 0)
        //     std::cout << "*************** Real-time is broken. " << -time_iter_remain << " [ms] exceeded!!! *************** \n";

        // ------------------------------------------------------------------------------- //
        // Update environment and check if the collision occurs
        if (!checkMotionValidity())
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

// Generate a horizon using predefined path (or random nodes).
// Only states from predefined path that come after 'q_next' are remained in the horizon. Other states are deleted.
void planning::drbt::DRGBT::generateHorizon()
{
    // std::cout << "Generating horizon... \n";
    auto time_generateHorizon { std::chrono::steady_clock::now() };

    // Deleting states which are left behind 'q_next', and do not belong to the predefined path
    int q_next_idx { q_next->getIndex() };
    if (!horizon.empty())   // 'q_next' is reached. Predefined path was not replanned nor 'status' is trapped
    {
        for (int i = horizon.size() - 1; i >= 0; i--)
        {
            if (horizon[i]->getIndex() <= q_next_idx)
            {
                horizon.erase(horizon.begin() + i);
                // std::cout << "Deleting state " << i << ". Horizon size is " << horizon.size() << "\n";
            }
        }
    }

    // Generating horizon
    size_t num_states { horizon_size - (horizon.size() + num_lateral_states) };
    if (status == base::State::Status::Reached && !predefined_path.empty())
    {
        size_t idx { 0 };    // Designates from which state in predefined path new states are added to the horizon
        if (!horizon.empty())
            idx = horizon.back()->getIndex() + 1;
        else
            idx = q_next_idx + 1;
        
        // std::cout << "Adding states from the predefined path starting from " << idx << ". state... \n";
        if (idx + num_states <= predefined_path.size())
        {
            for (size_t i = idx; i < idx + num_states; i++)
                horizon.emplace_back(std::make_shared<planning::drbt::HorizonState>(predefined_path[i], i));
        }
        else if (idx < predefined_path.size())
        {
            for (size_t i = idx; i < predefined_path.size(); i++)
                horizon.emplace_back(std::make_shared<planning::drbt::HorizonState>(predefined_path[i], i));
            
            horizon.back()->setStatus(planning::drbt::HorizonState::Status::Goal);
        }

        if (q_next->getStatus() == planning::drbt::HorizonState::Status::Goal)
            horizon.emplace_back(q_next);
    }
    else    // status == base::State::Status::Trapped || predefined_path.empty()
    {
        replanning = true;
        addRandomStates(num_states);
    }
    
    // Set the next state just for obtaining lateral spines later, since 'q_next' is not set
    if (!horizon.empty())
        q_next = horizon.front();

    // std::cout << "Initial horizon consists of " << horizon.size() << " states: \n";
    // for (size_t i = 0; i < horizon.size(); i++)
    //     std::cout << i << ". state:\n" << horizon[i] << "\n";
    
    planner_info->addRoutineTime(getElapsedTime(time_generateHorizon, planning::TimeUnit::us), 3);
}

// Update the horizon size, and add lateral spines.
void planning::drbt::DRGBT::updateHorizon()
{
    // std::cout << "Robot current state: " << q_current->getCoord().transpose() << " with d_c: " << d_c << "\n";
    auto time_updateHorizon { std::chrono::steady_clock::now() };

    if ((ss->num_dimensions-1) * d_c < DRGBTConfig::D_CRIT)
        horizon_size = DRGBTConfig::INIT_HORIZON_SIZE * ss->num_dimensions;
    else
        horizon_size = DRGBTConfig::INIT_HORIZON_SIZE * (1 + DRGBTConfig::D_CRIT / d_c);
    
    // Modified formula (does not show better performance):
    // if (d_c < DRGBTConfig::D_CRIT)
    //     horizon_size = ss->num_dimensions * DRGBTConfig::INIT_HORIZON_SIZE;
    // else
    //     horizon_size = DRGBTConfig::INIT_HORIZON_SIZE * (1 + (ss->num_dimensions-1) * DRGBTConfig::D_CRIT / d_c);
    
    // std::cout << "Modifying horizon size from " << horizon.size() << " to " << horizon_size << "\n";
    if (horizon_size < horizon.size())
        shortenHorizon(horizon.size() - horizon_size);
    else if (horizon_size > horizon.size())    // If 'horizon_size' has increased, or not enough states exist, then random states are added
        addRandomStates(horizon_size - horizon.size());

    // Lateral states are added
    // std::cout << "Adding " << num_lateral_states << " lateral states... \n";
    addLateralStates();
    horizon_size = horizon.size();
    planner_info->addRoutineTime(getElapsedTime(time_updateHorizon, planning::TimeUnit::us), 4);
}

// Generate a generalized bur from 'q_current', i.e., compute horizon spines.
// Bad and critical states will be replaced with "better" states, such that the horizon contains possibly better states.
void planning::drbt::DRGBT::generateGBur()
{
    // std::cout << "Generating gbur by computing reached states... \n";
    auto time_generateGBur { std::chrono::steady_clock::now() };
    size_t max_num_attempts {};
    float time_elapsed {};
    float max_time { DRGBTConfig::TRAJECTORY_INTERPOLATION == planning::TrajectoryInterpolation::Spline ? 
                     DRGBTConfig::MAX_TIME_TASK1 - Spline5Config::MAX_TIME_COMPUTE : DRGBTConfig::MAX_TIME_TASK1 };
    planner_info->setTask1Interrupted(false);

    for (size_t idx = 0; idx < horizon.size(); idx++)
    {
        computeReachedState(horizon[idx]);
        // std::cout << idx << ". state:\n" << horizon[idx] << "\n";

        if (DRGBTConfig::REAL_TIME_SCHEDULING != planning::RealTimeScheduling::None)   // Some scheduling is chosen
        {
            // Check whether the elapsed time for Task 1 is exceeded
            time_elapsed = getElapsedTime(time_iter_start);
            if (time_elapsed >= max_time && idx < horizon.size() - 1)
            {
                // Delete horizon states for which there is no enough remaining time to be processed
                // This is OK since better states are usually located at the beginning of horizon
                for (size_t i = horizon.size() - 1; i > idx; i--)
                {
                    if (q_next == horizon[i])   // 'q_next' will be deleted
                        q_next = horizon.front();

                    horizon.erase(horizon.begin() + i);
                }
                
                // std::cout << "Deleting " << horizon_size - horizon.size() << " of " << horizon_size << " horizon states...\n";
                planner_info->setTask1Interrupted(true);
                planner_info->addRoutineTime(getElapsedTime(time_generateGBur, planning::TimeUnit::ms), 2);
                return;
            }
            max_num_attempts = std::ceil((1 - time_elapsed / max_time) * DRGBTConfig::MAX_NUM_MODIFY_ATTEMPTS);
        }
        else
            max_num_attempts = DRGBTConfig::MAX_NUM_MODIFY_ATTEMPTS;

        // Bad and critical states are modified if there is enough remaining time for Task 1
        if (horizon[idx]->getStatus() == planning::drbt::HorizonState::Status::Bad || 
            horizon[idx]->getStatus() == planning::drbt::HorizonState::Status::Critical)
            modifyState(horizon[idx], max_num_attempts);
    }
    planner_info->addRoutineTime(getElapsedTime(time_generateGBur, planning::TimeUnit::ms), 2);
}

// Shorten the horizon by removing 'num' states. Excess states are deleted, and best states holds priority.
void planning::drbt::DRGBT::shortenHorizon(size_t num)
{
    size_t num_deleted { 0 };
    for (int i = horizon.size() - 1; i >= 0; i--)
    {
        if (horizon[i]->getStatus() == planning::drbt::HorizonState::Status::Bad)
        {
            horizon.erase(horizon.begin() + i);
            num_deleted++;
            // std::cout << "Deleting state " << i << " in stage 1 \n";
        }
        if (num_deleted == num)
            return;
    }

    for (int i = horizon.size() - 1; i >= 0; i--)
    {
        if (horizon[i]->getIndex() == -1)
        {
            horizon.erase(horizon.begin() + i);
            num_deleted++;
            // std::cout << "Deleting state " << i << " in stage 2 \n";
        }
        if (num_deleted == num)
            return;
    }

    for (int i = horizon.size() - 1; i >= 0; i--)
    {
        horizon.erase(horizon.begin() + i);
        num_deleted++;
        // std::cout << "Deleting state " << i << " in stage 3 \n";
        if (num_deleted == num)
            return;
    }
}

void planning::drbt::DRGBT::addRandomStates(size_t num)
{
    std::shared_ptr<base::State> q_rand { nullptr };
    for (size_t i = 0; i < num; i++)
    {
        q_rand = getRandomState(q_current);
        horizon.emplace_back(std::make_shared<planning::drbt::HorizonState>(q_rand, -1));
        // std::cout << "Adding random state: " << horizon.back()->getCoord().transpose() << "\n";
    }
}

void planning::drbt::DRGBT::addLateralStates()
{
    size_t num_added { 0 };
    if (ss->num_dimensions == 2)   // In 2D C-space only two possible lateral spines exist
    {
        std::shared_ptr<base::State> q_new;
        Eigen::Vector2f new_vec;
        for (int coord = -1; coord <= 1; coord += 2)
        {
            new_vec(0) = -coord;
            new_vec(1) = coord * (q_next->getCoord(0) - q_current->getCoord(0)) / (q_next->getCoord(1) - q_current->getCoord(1));
            q_new = ss->getNewState(q_current->getCoord() + new_vec);
            q_new = ss->interpolateEdge(q_current, q_new, RBTConnectConfig::DELTA);
            q_new = ss->pruneEdge(q_current, q_new);
            if (!ss->isEqual(q_current, q_new))
            {
                horizon.emplace_back(std::make_shared<planning::drbt::HorizonState>(q_new, -1));
                num_added++;
                // std::cout << "Adding lateral state: " << horizon.back()->getCoord().transpose() << "\n";
            }
        }
    }
    else
    {
        size_t idx { 0 };
        for (idx = 0; idx < ss->num_dimensions; idx++)
        {
            if (std::abs(q_next->getCoord(idx) - q_current->getCoord(idx)) > RealVectorSpaceConfig::EQUALITY_THRESHOLD)
                break;
        }
            
        if (idx < ss->num_dimensions)
        {
            std::shared_ptr<base::State> q_new { nullptr };
            float coord { 0 };
            for (size_t i = 0; i < num_lateral_states; i++)
            {
                q_new = ss->getRandomState(q_current);
                coord = q_current->getCoord(idx) + q_new->getCoord(idx) -
                        (q_next->getCoord() - q_current->getCoord()).dot(q_new->getCoord()) /
                        (q_next->getCoord(idx) - q_current->getCoord(idx));
                q_new->setCoord(coord, idx);
                q_new = ss->interpolateEdge(q_current, q_new, RBTConnectConfig::DELTA);
                q_new = ss->pruneEdge(q_current, q_new);
                
                if (!ss->isEqual(q_current, q_new))
                {
                    horizon.emplace_back(std::make_shared<planning::drbt::HorizonState>(q_new, -1));
                    num_added++;
                    // std::cout << "Adding lateral state: " << horizon.back()->getCoord().transpose() << "\n";
                }
            }
        }
    }
    addRandomStates(num_lateral_states - num_added);
}

// Modify state 'q' by replacing it with a random state, which is generated using biased distribution,
// i.e., oriented weight around 'q' ('q->getStatus()' == Bad), or around '-q' ('q->getStatus()' == Critical).
// Return whether the modification is successful
bool planning::drbt::DRGBT::modifyState(std::shared_ptr<planning::drbt::HorizonState> &q, size_t max_num_attempts)
{
    std::shared_ptr<planning::drbt::HorizonState> q_new_horizon_state { nullptr };
    std::shared_ptr<base::State> q_new { nullptr };
    std::shared_ptr<base::State> q_reached { q->getStateReached() };
    float norm { ss->getNorm(q_current, q_reached) };
    float coeff { 0 };
    
    for (size_t num = 0; num < max_num_attempts; num++)
    {
        Eigen::VectorXf vec = Eigen::VectorXf::Random(ss->num_dimensions) * norm / std::sqrt(ss->num_dimensions - 1);
        vec(0) = (vec(0) > 0) ? 1 : -1;
        vec(0) *= std::sqrt(norm * norm - vec.tail(ss->num_dimensions - 1).squaredNorm());
        if (q->getStatus() == planning::drbt::HorizonState::Status::Bad)
            q_new = ss->getNewState(q_reached->getCoord() + vec);
        else if (q->getStatus() == planning::drbt::HorizonState::Status::Critical)
        {
            q_new = ss->getNewState(2 * q_current->getCoord() - q_reached->getCoord() + coeff * vec);
            coeff = 1;
        }
        
        q_new = ss->interpolateEdge(q_current, q_new, RBTConnectConfig::DELTA);
        q_new = ss->pruneEdge(q_current, q_new);
        if (!ss->isEqual(q_current, q_new))
        {
            q_new_horizon_state = std::make_shared<planning::drbt::HorizonState>(q_new, -1);
            computeReachedState(q_new_horizon_state);
            if (q_new_horizon_state->getDistance() > q->getDistance())
            {
                // std::cout << "Modifying this state with the following state:\n" << q_new_horizon_state << "\n";
                if (q_next == q)
                    q_next = q_new_horizon_state;
                
                q = q_new_horizon_state;
                return true;
            }
            // else
            //     std::cout << "Modifying this state is not successfull (" << num << ". attempt)! \n";
        }
    }
    return false;
}

// Compute a reached state when generating a generalized spine from 'q_current' towards 'q'.
void planning::drbt::DRGBT::computeReachedState(const std::shared_ptr<planning::drbt::HorizonState> q)
{
    base::State::Status status { base::State::Status::None };
    std::shared_ptr<base::State> q_reached { nullptr };
    tie(status, q_reached) = extendGenSpine(q_current, q->getState());
    q->setStateReached(q_reached);
    q->setIsReached(status == base::State::Status::Reached ? true : false);

    // TODO: If there is enough remaining time, compute real distance-to-obstacles 
    float d_c_underest { ss->computeDistanceUnderestimation(q_reached, q_current->getNearestPoints()) };
    if (q->getDistance() != -1)
        q->setDistancePrevious(q->getDistance());
    else
        q->setDistancePrevious(d_c_underest);
    
    q->setDistance(d_c_underest);
    
    // Check whether the goal is reached
    if (q->getIndex() != -1 && ss->isEqual(q_reached, q_goal))
        q->setStatus(planning::drbt::HorizonState::Status::Goal);
}

// Compute weight for each state from the horizon, and then obtain the next state
void planning::drbt::DRGBT::computeNextState()
{
    std::vector<float> dist_to_goal(horizon.size());
    float d_goal_min { INFINITY };
    size_t d_goal_min_idx { 0 };
    float d_c_max { 0 };

    for (size_t i = 0; i < horizon.size(); i++)
    {
        dist_to_goal[i] = ss->getNorm(horizon[i]->getStateReached(), q_goal);
        if (dist_to_goal[i] < d_goal_min)
        {
            d_goal_min = dist_to_goal[i];
            d_goal_min_idx = i;
        }
        if (horizon[i]->getDistance() > d_c_max)
            d_c_max = horizon[i]->getDistance();
    }
    
    if (d_goal_min < RealVectorSpaceConfig::EQUALITY_THRESHOLD) // 'q_goal' lies in the horizon
    {
        d_goal_min = RealVectorSpaceConfig::EQUALITY_THRESHOLD; // Only to avoid "0/0" when 'dist_to_goal[i] < RealVectorSpaceConfig::EQUALITY_THRESHOLD'
        dist_to_goal[d_goal_min_idx] = d_goal_min;
    }

    std::vector<float> weights_dist(horizon.size());
    for (size_t i = 0; i < horizon.size(); i++)
        weights_dist[i] = d_goal_min / dist_to_goal[i];        
    
    d_max_mean = (planner_info->getNumIterations() * d_max_mean + d_c_max) / (planner_info->getNumIterations() + 1);
    float weights_dist_mean = std::accumulate(weights_dist.begin(), weights_dist.end(), 0.0) / horizon.size();
    float max_weight { 0 };
    float weight { 0 };
    int idx_best { -1 };

    for (size_t i = 0; i < horizon.size(); i++)
    {
        if (horizon[i]->getStatus() == planning::drbt::HorizonState::Status::Critical)  // 'weight' is already set to zero
            continue;
        
        // Computing 'weight' according to the following heuristic:
        weight = horizon[i]->getDistance() / d_max_mean
                 + (horizon[i]->getDistance() - horizon[i]->getDistancePrevious()) / d_max_mean 
                 + weights_dist[i] - weights_dist_mean;
        
        // Saturate 'weight' since it must be between 0 and 1
        if (weight > 1)
            weight = 1;
        else if (weight < 0)
            weight = 0;

        // If state does not belong to the predefined path, 'weight' is decreased within the range [0, DRGBTConfig::TRESHOLD_WEIGHT]
        // If such state becomes the best one, the replanning will surely be triggered
        if (horizon[i]->getIndex() == -1)   
            weight *= DRGBTConfig::TRESHOLD_WEIGHT;
        
        horizon[i]->setWeight(weight);

        if (weight > max_weight)
        {
            max_weight = weight;
            idx_best = i;
        }
    }

    if (idx_best != -1)
    {
        q_next = horizon[idx_best];
        float d_min { dist_to_goal[idx_best] };

        // Do the following only if 'q_next' belongs to the predefined path
        if (q_next->getIndex() != -1)
        {
            float hysteresis = 0.1 * DRGBTConfig::TRESHOLD_WEIGHT;  // Hysteresis size when choosing the next state

            // "The best" state nearest to the goal is chosen as the next state
            for (size_t i = 0; i < horizon.size(); i++)
            {
                if (std::abs(q_next->getWeight() - horizon[i]->getWeight()) < hysteresis && dist_to_goal[i] < d_min)
                {
                    d_min = dist_to_goal[i];
                    q_next = horizon[i];
                }
            }

            // If weights of 'q_next_previous' and 'q_next' are close, 'q_next_previous' remains the next state
            if (q_next != q_next_previous &&
                std::abs(q_next->getWeight() - q_next_previous->getWeight()) < hysteresis &&
                q_next->getStatus() != planning::drbt::HorizonState::Status::Goal &&
                getIndexInHorizon(q_next_previous) != -1)
                    q_next = q_next_previous;
        }
    }
    else if (predefined_path.empty())   // All states are critical, and 'q_next' cannot be updated!
    {
        std::cout << "All states are critical, and q_next cannot be updated! \n";
        horizon.clear();
        status = base::State::Status::Trapped;
        replanning = true;
        q_next = std::make_shared<planning::drbt::HorizonState>(q_current, -1, q_current);
    }

    q_next_previous = q_next;
    // std::cout << "Setting the robot next state to: " << q_next->getCoord().transpose() << "\n";
    
    // std::cout << "Horizon consists of " << horizon.size() << " states: \n";
    // for (size_t i = 0; i < horizon.size(); i++)
    //     std::cout << i << ". state:\n" << horizon[i] << "\n";
}

// Return index in the horizon of state 'q'. If 'q' does not belong to the horizon, -1 is returned.
int planning::drbt::DRGBT::getIndexInHorizon(const std::shared_ptr<planning::drbt::HorizonState> q)
{
    for (size_t idx = 0; idx < horizon.size(); idx++)
    {
        if (q == horizon[idx])
            return idx;
    }
    return -1;   
}

bool planning::drbt::DRGBT::checkTerminatingCondition([[maybe_unused]] base::State::Status status)
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
	
    if (time_current >= DRGBTConfig::MAX_PLANNING_TIME)
	{
        std::cout << "Maximal planning time has been reached! \n";
		planner_info->setSuccessState(false);
        planner_info->setPlanningTime(time_current);
		return true;
	}
    
    if (planner_info->getNumIterations() >= DRGBTConfig::MAX_NUM_ITER)
	{
        std::cout << "Maximal number of iterations has been reached! \n";
		planner_info->setSuccessState(false);
        planner_info->setPlanningTime(time_current);
		return true;
	}

	return false;
}

void planning::drbt::DRGBT::outputPlannerData(const std::string &filename, bool output_states_and_paths, bool append_output) const
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
