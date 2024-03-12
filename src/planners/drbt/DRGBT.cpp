//
// Created by nermin on 13.04.22.
//

#include "DRGBT.h"

// #include <glog/log_severity.h>
// #include <glog/logging.h>
// WARNING: You need to be very careful with LOG(INFO) for console output, due to a possible "stack smashing detected" error.
// If you get this error, just use std::cout for console output.

planning::drbt::DRGBT::DRGBT(const std::shared_ptr<base::StateSpace> ss_) : RGBTConnect(ss_) {}

planning::drbt::DRGBT::DRGBT(const std::shared_ptr<base::StateSpace> ss_, const std::shared_ptr<base::State> q_start_,
                             const std::shared_ptr<base::State> q_goal_) : RGBTConnect(ss_)
{
	// std::cout << "Initializing DRGBT planner... \n";
    q_start = q_start_;
    q_goal = q_goal_;
	if (!ss->isValid(q_start))
		throw std::domain_error("Start position is invalid!");
    
    q_current = q_start;
    q_previous = q_current;
    q_target = q_current;
    q_next = std::make_shared<planning::drbt::HorizonState>(q_current, 0);
    q_next_previous = q_next;

    d_max_mean = 0;
    num_lateral_states = 2 * ss->num_dimensions - 2;
    horizon_size = DRGBTConfig::INIT_HORIZON_SIZE + num_lateral_states;
    replanning = false;
    status = base::State::Status::Reached;
    planner_info->setNumStates(1);
	planner_info->setNumIterations(0);
    path.emplace_back(q_start);                               // State 'q_start' is added to the realized path

    delta_q_max = 0;
    for (int i = 0; i < ss->num_dimensions; i++)
        delta_q_max += std::pow(ss->robot->getMaxVel(i), 2);
    delta_q_max = std::sqrt(delta_q_max) * DRGBTConfig::MAX_ITER_TIME * 1e-3;

    spline_current = std::make_shared<planning::trajectory::Spline5>(ss->robot, q_current->getCoord());
    spline_next = spline_current;
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
    time_start = std::chrono::steady_clock::now();     // Start the algorithm clock
    time_iter_start = time_start;
    float d_c;

    // Initial iteration: Obtaining the inital path using specified static planner
    // std::cout << "\nIteration num. " << planner_info->getNumIterations() << "\n";
    // std::cout << "Obtaining the inital path... \n";
    replan(DRGBTConfig::MAX_ITER_TIME);
    planner_info->setNumIterations(planner_info->getNumIterations() + 1);
    planner_info->addIterationTime(getElapsedTime(time_iter_start, std::chrono::steady_clock::now()));
    // std::cout << "----------------------------------------------------------------------------------------\n";

    while (true)
    {
        std::cout << "\nIteration num. " << planner_info->getNumIterations() << "\n";
        // std::cout << "TASK 1: Computing next configuration... \n";
        time_iter_start = std::chrono::steady_clock::now();     // Start the iteration clock
        
        // ------------------------------------------------------------------------------- //
        // Since the environment may change, a new distance is required!
        auto time_computeDistance = std::chrono::steady_clock::now();
        d_c = ss->computeDistance(q_target, true);     // ~ 1 [ms]
        if (d_c <= 0)   // The desired/target conf. is not safe, thus the robot is required to stop immediately, 
        {               // and compute the horizon again from 'q_current'
            // TODO: Urgently stopping needs to be implemented using quartic spline.
            q_target = q_current;
            d_c = ss->computeDistance(q_target, true);     // ~ 1 [ms]
            clearHorizon(base::State::Status::Trapped, true);
            q_next = std::make_shared<planning::drbt::HorizonState>(q_target, 0);
            // std::cout << "Not updating the robot current state since d_c < 0. \n";
        }
        planner_info->addRoutineTime(getElapsedTime(time_computeDistance, std::chrono::steady_clock::now(), "us"), 1);

        // ------------------------------------------------------------------------------- //
        if (status != base::State::Status::Advanced)
            generateHorizon();          // ~ 2 [us]
            
        updateHorizon(d_c);             // ~ 10 [us]
        generateGBur();                 // ~ 10 [ms] Time consuming routine... 
        computeNextState();             // ~ 1 [us]
        if (DRGBTConfig::TRAJECTORY_INTERPOLATION == "spline")
            updateCurrentState();       // ~ 1 [ms]
        else if (DRGBTConfig::TRAJECTORY_INTERPOLATION == "none")
            updateCurrentState2();      // ~ 1 [us]

        // std::cout << "Time elapsed: " << getElapsedTime(time_iter_start, std::chrono::steady_clock::now(), "us") << " [us]\n";

        // ------------------------------------------------------------------------------- //
        // Replanning procedure assessment
        if (whetherToReplan())
        {
            // std::cout << "TASK 2: Replanning... \n";
            replan(DRGBTConfig::MAX_ITER_TIME - getElapsedTime(time_iter_start, std::chrono::steady_clock::now()));   
        }
        // else
        //     std::cout << "Replanning is not required! \n";

        // ------------------------------------------------------------------------------- //
        // Checking the real-time execution
        int time_iter_remain = DRGBTConfig::MAX_ITER_TIME - getElapsedTime(time_iter_start, std::chrono::steady_clock::now());
        std::cout << "Remaining iteration time is " << time_iter_remain << " [ms]. \n";
        if (time_iter_remain < 0)
            std::cout << "*************** Real-time is broken. " << -time_iter_remain << " [ms] exceeded!!! *************** \n";

        // ------------------------------------------------------------------------------- //
        // Update environment and check if the collision occurs
        if (DRGBTConfig::TRAJECTORY_INTERPOLATION == "spline" && !checkMotionValidity() ||
            DRGBTConfig::TRAJECTORY_INTERPOLATION == "none" && !checkMotionValidity2())
        {
            std::cout << "Collision has been occured!!! \n";
            planner_info->setSuccessState(false);
            planner_info->setPlanningTime(planner_info->getIterationTimes().back());
            return false;
        }

        // ------------------------------------------------------------------------------- //
        // Planner info and terminating condition
        planner_info->setNumIterations(planner_info->getNumIterations() + 1);
        planner_info->addIterationTime(getElapsedTime(time_start, std::chrono::steady_clock::now()));
        if (checkTerminatingCondition())
            return planner_info->getSuccessState();

        std::cout << "----------------------------------------------------------------------------------------\n";
    }
}

// Generate a horizon using predefined path (or random nodes).
// Only states from predefined path that come after 'q_next' are remained in the horizon. Other states are deleted.
void planning::drbt::DRGBT::generateHorizon()
{
    // std::cout << "Generating horizon... \n";
    auto time_generateHorizon = std::chrono::steady_clock::now();

    // Deleting states which are left behind 'q_next', and do not belong to the predefined path
    int q_next_idx = q_next->getIndex();
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
    int num_states = horizon_size - (horizon.size() + num_lateral_states);
    if (status == base::State::Status::Reached && !predefined_path.empty())
    {
        int idx;    // Designates from which state in predefined path new states are added to the horizon
        if (!horizon.empty())
            idx = horizon.back()->getIndex() + 1;
        else
            idx = q_next_idx + 1;
        
        // std::cout << "Adding states from the predefined path starting from " << idx << ". state... \n";
        if (idx + num_states <= predefined_path.size())
        {
            for (int i = idx; i < idx + num_states; i++)
                horizon.emplace_back(std::make_shared<planning::drbt::HorizonState>(predefined_path[i], i));
        }
        else if (idx < predefined_path.size())
        {
            for (int i = idx; i < predefined_path.size(); i++)
                horizon.emplace_back(std::make_shared<planning::drbt::HorizonState>(predefined_path[i], i));
            
            horizon.back()->setStatus(planning::drbt::HorizonState::Status::Goal);
        }
    }
    else            // status == base::State::Status::Trapped || predefined_path.empty()
    {
        replanning = true;
        addRandomStates(num_states);
    }
    
    // Set the next state just for obtaining lateral spines later, since 'q_next' is not set
    q_next = horizon.front();

    // std::cout << "Initial horizon consists of " << horizon.size() << " states: \n";
    // for (int i = 0; i < horizon.size(); i++)
    //     std::cout << i << ". state:\n" << horizon[i] << "\n";
    
    planner_info->addRoutineTime(getElapsedTime(time_generateHorizon, std::chrono::steady_clock::now(), "us"), 3);
}

// Update the horizon size, and add lateral spines.
void planning::drbt::DRGBT::updateHorizon(float d_c)
{
    // std::cout << "Robot target state: " << q_target->getCoord().transpose() << " with d_c: " << d_c << "\n";
    auto time_updateHorizon = std::chrono::steady_clock::now();

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
    planner_info->addRoutineTime(getElapsedTime(time_updateHorizon, std::chrono::steady_clock::now(), "us"), 4);
}

// Generate the generalized bur from 'q_target', i.e., compute the horizon spines.
// Bad and critical states will be replaced with "better" states, such that the horizon contains possibly better states.
void planning::drbt::DRGBT::generateGBur()
{
    // std::cout << "Generating gbur by computing reached states... \n";
    auto time_generateGBur = std::chrono::steady_clock::now();
    int max_num_attempts;
    planner_info->setTask1Interrupted(false);

    for (int idx = 0; idx < horizon.size(); idx++)
    {
        computeReachedState(horizon[idx]);
        // std::cout << idx << ". state:\n" << horizon[idx] << "\n";

        if (!DRGBTConfig::REAL_TIME_SCHEDULING.empty())   // Some scheduling is chosen
        {
            // Check whether the elapsed time for Task 1 is exceeded
            // 1 [ms] is reserved for the routines computeNextState and updateCurrentState
            int time_elapsed = getElapsedTime(time_iter_start, std::chrono::steady_clock::now());
            if (time_elapsed >= DRGBTConfig::MAX_TIME_TASK1 - 1 && idx < horizon.size() - 1)
            {
                // Delete horizon states for which there is no enough remaining time to be processed
                // This is OK since better states are usually located at the beginning of horizon
                for (int i = horizon.size() - 1; i > idx; i--)
                {
                    if (q_next == horizon[i])   // 'q_next' will be deleted
                        q_next = horizon.front();

                    horizon.erase(horizon.begin() + i);
                }
                
                // std::cout << "Deleting " << horizon_size - horizon.size() << " of " << horizon_size << " horizon states...\n";
                planner_info->setTask1Interrupted(true);
                planner_info->addRoutineTime(getElapsedTime(time_generateGBur, std::chrono::steady_clock::now()), 2);
                return;
            }
            max_num_attempts = std::ceil((1 - float(time_elapsed) / (DRGBTConfig::MAX_TIME_TASK1 - 1)) * DRGBTConfig::MAX_NUM_MODIFY_ATTEMPTS);
        }
        else
            max_num_attempts = DRGBTConfig::MAX_NUM_MODIFY_ATTEMPTS;

        // Bad and critical states are modified if there is enough remaining time for Task 1
        if (horizon[idx]->getStatus() == planning::drbt::HorizonState::Status::Bad || 
            horizon[idx]->getStatus() == planning::drbt::HorizonState::Status::Critical)
            modifyState(horizon[idx], max_num_attempts);
    }
    planner_info->addRoutineTime(getElapsedTime(time_generateGBur, std::chrono::steady_clock::now()), 2);
}

// Shorten the horizon by removing 'num' states. Excess states are deleted, and best states holds priority.
void planning::drbt::DRGBT::shortenHorizon(int num)
{
    int num_deleted = 0;
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

void planning::drbt::DRGBT::addRandomStates(int num)
{
    std::shared_ptr<base::State> q_rand;
    for (int i = 0; i < num; i++)
    {
        q_rand = getRandomState(q_target);
        horizon.emplace_back(std::make_shared<planning::drbt::HorizonState>(q_rand, -1));
        // std::cout << "Adding random state: " << horizon.back()->getCoord().transpose() << "\n";
    }
}

void planning::drbt::DRGBT::addLateralStates()
{
    int num_added = 0;
    if (ss->num_dimensions == 2)   // In 2D C-space only two possible lateral spines exist
    {
        std::shared_ptr<base::State> q_new;
        Eigen::Vector2f new_vec;
        for (int coord = -1; coord <= 1; coord += 2)
        {
            new_vec(0) = -coord; 
            new_vec(1) = coord * (q_next->getCoord(0) - q_target->getCoord(0)) / (q_next->getCoord(1) - q_target->getCoord(1));
            q_new = ss->getNewState(q_target->getCoord() + new_vec);
            q_new = ss->interpolateEdge(q_target, q_new, RBTConnectConfig::DELTA);
            q_new = ss->pruneEdge(q_target, q_new);
            if (!ss->isEqual(q_target, q_new))
            {
                horizon.emplace_back(std::make_shared<planning::drbt::HorizonState>(q_new, -1));
                num_added++;
                // std::cout << "Adding lateral state: " << horizon.back()->getCoord().transpose() << "\n";
            }
        }
    }
    else
    {
        int idx;
        for (idx = 0; idx < ss->num_dimensions; idx++)
            if (std::abs(q_next->getCoord(idx) - q_target->getCoord(idx)) > RealVectorSpaceConfig::EQUALITY_THRESHOLD)
                break;
            
        if (idx < ss->num_dimensions)
        {
            std::shared_ptr<base::State> q_new;
            float coord;
            for (int i = 0; i < num_lateral_states; i++)
            {
                q_new = ss->getRandomState(q_target);
                coord = q_target->getCoord(idx) + q_new->getCoord(idx) -
                        (q_next->getCoord() - q_target->getCoord()).dot(q_new->getCoord()) /
                        (q_next->getCoord(idx) - q_target->getCoord(idx));
                q_new->setCoord(coord, idx);
                q_new = ss->interpolateEdge(q_target, q_new, RBTConnectConfig::DELTA);
                q_new = ss->pruneEdge(q_target, q_new);
                if (!ss->isEqual(q_target, q_new))
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
bool planning::drbt::DRGBT::modifyState(std::shared_ptr<planning::drbt::HorizonState> &q, int max_num_attempts)
{
    std::shared_ptr<planning::drbt::HorizonState> q_new_horizon_state;
    std::shared_ptr<base::State> q_new;
    std::shared_ptr<base::State> q_reached = q->getStateReached();
    float norm = ss->getNorm(q_target, q_reached);
    float coeff = 0;
    
    for (int num = 0; num < max_num_attempts; num++)
    {
        Eigen::VectorXf vec = Eigen::VectorXf::Random(ss->num_dimensions) * norm / std::sqrt(ss->num_dimensions - 1);
        vec(0) = (vec(0) > 0) ? 1 : -1;
        vec(0) *= std::sqrt(norm * norm - vec.tail(ss->num_dimensions - 1).squaredNorm());
        if (q->getStatus() == planning::drbt::HorizonState::Status::Bad)
            q_new = ss->getNewState(q_reached->getCoord() + vec);
        else if (q->getStatus() == planning::drbt::HorizonState::Status::Critical)
        {
            q_new = ss->getNewState(2 * q_target->getCoord() - q_reached->getCoord() + coeff * vec);
            coeff = 1;
        }
        q_new = ss->interpolateEdge(q_target, q_new, RBTConnectConfig::DELTA);
        q_new = ss->pruneEdge(q_target, q_new);
        if (!ss->isEqual(q_target, q_new))
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

// Compute reached state when generating a generalized spine from 'q_target' towards 'q'.
void planning::drbt::DRGBT::computeReachedState(const std::shared_ptr<planning::drbt::HorizonState> q)
{
    base::State::Status status;
    std::shared_ptr<base::State> q_reached;
    tie(status, q_reached) = extendGenSpine(q_target, q->getState());
    q->setStateReached(q_reached);
    q->setIsReached(status == base::State::Status::Reached ? true : false);

    // TODO: If there is enough remaining time, compute real distance-to-obstacles 
    float d_c = ss->computeDistanceUnderestimation(q_reached, q_target->getNearestPoints());
    if (q->getDistance() != -1)
        q->setDistancePrevious(q->getDistance());
    else
        q->setDistancePrevious(d_c);
    
    q->setDistance(d_c);
    
    // Check whether the goal is reached
    if (q->getIndex() != -1 && ss->isEqual(q_reached, q_goal))
        q->setStatus(planning::drbt::HorizonState::Status::Goal);
}

// Compute weight for each state from the horizon, and then obtain the next state
void planning::drbt::DRGBT::computeNextState()
{
    std::vector<float> dist_to_goal(horizon.size());
    float d_goal_min = INFINITY;
    int d_goal_min_idx = -1;
    float d_c_max = 0;

    for (int i = 0; i < horizon.size(); i++)
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
    for (int i = 0; i < horizon.size(); i++)
        weights_dist[i] = d_goal_min / dist_to_goal[i];        
    
    d_max_mean = (planner_info->getNumIterations() * d_max_mean + d_c_max) / (planner_info->getNumIterations() + 1);
    float weights_dist_mean = std::accumulate(weights_dist.begin(), weights_dist.end(), 0.0) / horizon.size();
    float max_weight = 0;
    float weight;
    int idx_best = -1;

    for (int i = 0; i < horizon.size(); i++)
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
        float d_min = dist_to_goal[idx_best];

        // Do the following only if 'q_next' belongs to the predefined path
        if (q_next->getIndex() != -1)
        {
            float hysteresis = 0.1 * DRGBTConfig::TRESHOLD_WEIGHT;  // Hysteresis size when choosing the next state

            // "The best" state nearest to the goal is chosen as the next state
            for (int i = 0; i < horizon.size(); i++)
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
    else    // All states are critical, and q_next cannot be updated! 'status' will surely become Trapped
    {
        // std::cout << "All states are critical, and q_next cannot be updated! \n";
        q_next = std::make_shared<planning::drbt::HorizonState>(q_target, 0);
        q_next->setStateReached(q_target);
    }

    q_next_previous = q_next;
    // std::cout << "Setting the robot next state to: " << q_next->getCoord().transpose() << "\n";
    
    // std::cout << "Horizon consists of " << horizon.size() << " states: \n";
    // for (int i = 0; i < horizon.size(); i++)
    //     std::cout << i << ". state:\n" << horizon[i] << "\n";
}

// Return index in the horizon of state 'q'. If 'q' does not belong to the horizon, -1 is returned.
int planning::drbt::DRGBT::getIndexInHorizon(const std::shared_ptr<planning::drbt::HorizonState> q)
{
    for (int idx = 0; idx < horizon.size(); idx++)
    {
        if (q == horizon[idx])
            return idx;
    }
    return -1;   
}

/// @brief Update the current state 'q_current' to become 'q_target'.
/// Determine a new target state 'q_target' (a new desired current state) of the robot 
/// by moving from 'q_current' towards 'q_next' for an advancing step size determined as 
/// ss->robot->getMaxVel(i) * DRGBTConfig::MAX_ITER_TIME * 1e-3, i.d., using the maximal robot's velocity.
void planning::drbt::DRGBT::updateCurrentState2()
{
    q_previous = q_current;
    q_current = q_target;
    if (ss->isEqual(q_current, q_goal))
    {
        status = base::State::Status::Reached;
        return;
    }

    q_target = ss->getNewState(q_next->getStateReached()->getCoord());

    // If all velocities are not the same, the following can be used:
    std::vector<std::pair<float, float>> limits;
    for (int i = 0; i < ss->num_dimensions; i++)
    {
        limits.emplace_back(std::pair<float, float>
            (q_current->getCoord(i) - ss->robot->getMaxVel(i) * DRGBTConfig::MAX_ITER_TIME * 1e-3, 
             q_current->getCoord(i) + ss->robot->getMaxVel(i) * DRGBTConfig::MAX_ITER_TIME * 1e-3));
    }
    q_target = ss->pruneEdge(q_current, q_target, limits);  // Check whether 'q_target' can be reached considering robot max. velocity

    // If all velocities are the same, the following can be used:
    // float delta_q_max1 = ss->robot->getMaxVel(0) * DRGBTConfig::MAX_ITER_TIME * 1e-3;   // Time conversion from [ms] to [s]
    // if (ss->getNorm(q_current, q_target) > delta_q_max1)    // Check whether 'q_target' can be reached considering robot max. velocity
    //     q_target = ss->pruneEdge2(q_current, q_target, delta_q_max1);

    if (!ss->isEqual(q_current, q_target))
    {
        if (ss->isEqual(q_current, q_next->getState()))
            status = base::State::Status::Reached;      // 'q_next' must be reached, and not only 'q_next->getStateReached()'
        else
            status = base::State::Status::Advanced;
        
        q_target->setParent(q_current);
    }
    else
        clearHorizon(base::State::Status::Trapped, true);

    // std::cout << "Status: " << (status == base::State::Status::Advanced ? "Advanced" : "")
    //                         << (status == base::State::Status::Trapped  ? "Trapped"  : "")
    //                         << (status == base::State::Status::Reached  ? "Reached"  : "") << "\n";
}

/// @brief Update the current state of the robot using 'spline_current'.
/// Compute a new spline 'spline_next', or remain 'spline_current'.
/// Determine a new target state 'q_target' (a new desired current state) of the robot 
/// by moving from 'q_current' towards 'q_next' while following 'spline_next'.
/// @return Remaining time in [s] after which 'spline_next' will become active.
/// @note The new spline will be computed in a way that all constraints on robot's maximal velocity, 
/// acceleration and jerk are surely satisfied.
float planning::drbt::DRGBT::updateCurrentState()
{
    spline_current = spline_next;

    float t_spline_max = 3e-3;     // Maximal allowable time in [s] for a spline computation
    float t_iter = getElapsedTime(time_iter_start, std::chrono::steady_clock::now(), "us") * 1e-6;
    if (DRGBTConfig::MAX_TIME_TASK1 * 1e-3 - t_iter < t_spline_max)
        t_spline_max = DRGBTConfig::MAX_TIME_TASK1 * 1e-3 - t_iter;
    
    float t_iter_remain = DRGBTConfig::MAX_ITER_TIME * 1e-3 - t_iter - t_spline_max;
    float t_spline_current = spline_current->getTimeEnd() + t_iter + t_spline_max;

    spline_current->setTimeBegin(spline_next->getTimeEnd());
    spline_current->setTimeCurrent(t_spline_current);

    q_current = ss->getNewState(spline_current->getPosition(t_spline_current));

    std::cout << "Iter. time:        " << t_iter * 1000 << " [ms] \n";
    std::cout << "Max. spline time:  " << t_spline_max * 1000 << " [ms] \n";
    std::cout << "Remain. time:      " << t_iter_remain * 1000 << " [ms] \n";
    std::cout << "Begin spline time: " << spline_current->getTimeBegin() * 1000 << " [ms] \n";
    std::cout << "Curr. spline time: " << t_spline_current * 1000 << " [ms] \n";
    std::cout << "q_current: " << q_current << "\n";
    std::cout << "q_target:  " << q_target << "\n";
    std::cout << "q_next:    " << q_next << "\n";

    if (ss->isEqual(q_current, q_goal))
    {
        spline_next->setTimeEnd(t_spline_current + t_iter_remain);
        status = base::State::Status::Reached;
        return t_spline_max;
    }

    bool found = false;
    if ((q_next->getStateReached()->getCoord() - spline_current->getPosition(INFINITY)).norm() 
        < RealVectorSpaceConfig::EQUALITY_THRESHOLD)  // Coordinates of q_next_reached did not change
    {
        std::cout << "Not computing a new spline! \n";
        found = false;
    }
    else if (ss->isEqual(q_next->getState(), q_target))
    {
        std::cout << "Robot is urgently stopping! \n";
        found = false;
        // TODO: Urgently stopping needs to be implemented using quartic spline.
    }
    else
    {
        std::chrono::steady_clock::time_point time_start_ = std::chrono::steady_clock::now();
        std::vector<std::shared_ptr<planning::drbt::HorizonState>> visited_states = { q_next };
        spline_next = std::make_shared<planning::trajectory::Spline5>
        (
            ss->robot, 
            q_current->getCoord(),
            spline_current->getVelocity(t_spline_current),
            spline_current->getAcceleration(t_spline_current)
        );

        while (getElapsedTime(time_start_, std::chrono::steady_clock::now(), "us") * 1e-6 < t_spline_max - 1e-3)
        {
            if (spline_next->compute(q_next->getStateReached()->getCoord()))
            {
                std::cout << "New spline is computed! \n";
                found = true;                
                break;
            }
            else if (!changeNextState(visited_states))
            {
                std::cout << "Spline could not be changed! Continuing with the previous spline! \n";
                found = false;                    
                break;
            }
        }
        std::cout << "Elapsed time for spline computing: " << getElapsedTime(time_start_, std::chrono::steady_clock::now(), "us") << " [us] \n";
    }

    if (found)
    {
        spline_current->setTimeEnd(t_spline_current);
        spline_next->setTimeEnd(t_iter_remain);
        std::cout << "q_target time: " << t_iter_remain * 1000 + DRGBTConfig::MAX_TIME_TASK1 << " [ms] \n";
    }
    else
    {
        spline_next = spline_current;
        spline_next->setTimeEnd(t_spline_current + t_iter_remain);
        std::cout << "q_target time: " << spline_next->getTimeEnd() * 1000 + DRGBTConfig::MAX_TIME_TASK1 << " [ms] \n";
    }

    // TODO: New spline needs to be validated on collision, at least during the current iteration!
    
    q_target = ss->getNewState(spline_next->getPosition(spline_next->getTimeEnd() + DRGBTConfig::MAX_TIME_TASK1 * 1e-3));
    std::cout << "q_target:      " << q_target << "\n";
    std::cout << "Spline next: \n" << spline_next << "\n";
    
    if (!ss->isEqual(q_current, q_target))
    {
        if (ss->isEqual(q_current, q_next->getState()))
            status = base::State::Status::Reached;      // 'q_next' must be reached, and not only 'q_next->getStateReached()'
        else
            status = base::State::Status::Advanced;
        
        q_target->setParent(q_current);
    }
    else
        clearHorizon(base::State::Status::Trapped, true);

    // std::cout << "Status: " << (status == base::State::Status::Advanced ? "Advanced" : "")
    //                         << (status == base::State::Status::Trapped  ? "Trapped"  : "")
    //                         << (status == base::State::Status::Reached  ? "Reached"  : "") << "\n";

    return t_spline_max - (getElapsedTime(time_iter_start, std::chrono::steady_clock::now(), "us") * 1e-6 - t_iter);
}

bool planning::drbt::DRGBT::changeNextState(std::vector<std::shared_ptr<planning::drbt::HorizonState>> &visited_states)
{
    std::cout << "Change of q_next is required! \n";
    std::shared_ptr<planning::drbt::HorizonState> q_new = nullptr;
    float weight_max = 0;
    bool visited;

    for (std::shared_ptr<planning::drbt::HorizonState> q : horizon)
    {
        if (q->getWeight() < DRGBTConfig::TRESHOLD_WEIGHT)
            continue;

        visited = false;
        for (std::shared_ptr<planning::drbt::HorizonState> q_visited : visited_states)
        {
            if (q == q_visited)
            {
                visited = true;
                break;
            }
        }

        if (!visited && q->getWeight() > weight_max)
        {
            q_new = q;
            weight_max = q->getWeight();
        }
    }

    if (q_new != nullptr)
    {
        visited_states.emplace_back(q_new);
        q_next = q_new;

        std::cout << "Horizon size: " << horizon.size() << "\t Visited idx: ";
        for (std::shared_ptr<planning::drbt::HorizonState> q_visited : visited_states)
            std::cout << getIndexInHorizon(q_visited) << " ";
        std::cout << "\n" << "New q_next: " << q_next << "\n";

        return true;
    }

    return false;
}

void planning::drbt::DRGBT::clearHorizon(base::State::Status status_, bool replanning_)
{
    horizon.clear();
    status = status_;
    replanning = replanning_;
}

bool planning::drbt::DRGBT::whetherToReplan()
{
    if (replanning)
        return true;
    
    float weight_max = 0;
    float weight_sum = 0;
    for (int i = 0; i < horizon.size(); i++)
    {
        weight_max = std::max(weight_max, horizon[i]->getWeight());
        weight_sum += horizon[i]->getWeight();
    }
    return (weight_max <= DRGBTConfig::TRESHOLD_WEIGHT && 
            weight_sum / horizon.size() <= DRGBTConfig::TRESHOLD_WEIGHT) 
            ? true : false;
}

// Initialize static planner, to plan the path from 'q_target' to 'q_goal' in 'max_planning_time' 
std::unique_ptr<planning::AbstractPlanner> planning::drbt::DRGBT::initStaticPlanner(int max_planning_time)
{
    // std::cout << "Static planner (for replanning): " << DRGBTConfig::STATIC_PLANNER_NAME << "\n";
    if (DRGBTConfig::STATIC_PLANNER_NAME == "RGBMT*")
    {
        RGBMTStarConfig::MAX_PLANNING_TIME = max_planning_time;
        return std::make_unique<planning::rbt_star::RGBMTStar>(ss, q_target, q_goal);
    }
    else if (DRGBTConfig::STATIC_PLANNER_NAME == "RGBTConnect")
    {
        RGBTConnectConfig::MAX_PLANNING_TIME = max_planning_time;
        return std::make_unique<planning::rbt::RGBTConnect>(ss, q_target, q_goal);
    }
    else if (DRGBTConfig::STATIC_PLANNER_NAME == "RBTConnect")
    {
        RBTConnectConfig::MAX_PLANNING_TIME = max_planning_time;
        return std::make_unique<planning::rbt::RBTConnect>(ss, q_target, q_goal);
    }
    else if (DRGBTConfig::STATIC_PLANNER_NAME == "RRTConnect")
    {
        RRTConnectConfig::MAX_PLANNING_TIME = max_planning_time;
        return std::make_unique<planning::rrt::RRTConnect>(ss, q_target, q_goal);
    }
    else
        throw std::domain_error("The requested static planner " + DRGBTConfig::STATIC_PLANNER_NAME + " is not found! ");
}

// Try to replan the predefined path from the target to the goal configuration within the specified time
void planning::drbt::DRGBT::replan(int max_planning_time)
{
    std::unique_ptr<planning::AbstractPlanner> planner;
    bool result = false;

    try
    {
        if (max_planning_time <= 0)
            throw std::runtime_error("Not enough time for replanning! ");

        if (DRGBTConfig::REAL_TIME_SCHEDULING == "FPS")         // Fixed Priority Scheduling
        {
            // std::cout << "Replanning with Fixed Priority Scheduling \n";
            // std::cout << "Trying to replan in " << max_planning_time << " [ms]... \n";
            planner = initStaticPlanner(max_planning_time);
            result = planner->solve();
        }
        else                                                    // No real-time scheduling
        {
            // std::cout << "Replanning without real-time scheduling \n";
            // std::cout << "Trying to replan in " << max_planning_time << " [ms]... \n";
            planner = initStaticPlanner(max_planning_time);
            result = planner->solve();
        }

        // New path is found within the specified time limit, thus update the predefined path to the goal
        if (result && planner->getPlannerInfo()->getPlanningTime() <= max_planning_time)
        {
            // std::cout << "The path has been replanned in " << planner->getPlannerInfo()->getPlanningTime() << " [ms]. \n";
            acquirePredefinedPath(planner->getPath());
            clearHorizon(base::State::Status::Reached, false);
            q_next = std::make_shared<planning::drbt::HorizonState>(q_target, 0);
            planner_info->addRoutineTime(planner->getPlannerInfo()->getPlanningTime(), 0);  // replan
        }
        else    // New path is not found, and just continue with the previous motion. We can also impose the robot to stop.
            throw std::runtime_error("New path is not found! ");
    }
    catch (std::exception &e)
    {
        // std::cout << "Replanning is required. " << e.what() << "\n";
        replanning = true;
    }
}

void planning::drbt::DRGBT::acquirePredefinedPath(const std::vector<std::shared_ptr<base::State>> &path_)
{
    predefined_path.clear();
    predefined_path.emplace_back(path_.front());
    base::State::Status status_temp;
    std::shared_ptr<base::State> q_new;

    for (int i = 1; i < path_.size(); i++)
    {
        status_temp = base::State::Status::Advanced;
        q_new = path_[i-1];
        while (status_temp == base::State::Status::Advanced)
        {
            std::tie(status_temp, q_new) = ss->interpolateEdge2(q_new, path_[i], delta_q_max);
            predefined_path.emplace_back(q_new);
        }
    }

    // std::cout << "Predefined path is: \n";
    // for (int i = 0; i < predefined_path.size(); i++)
    //     std::cout << predefined_path.at(i) << "\n";
    // std::cout << std::endl;
}

/// @brief Discretely check the validity of motion when robot moves from 'q_previous' to 'q_current', 
/// while the obstacles are moving simultaneously. Finally, the environment is updated within this function.
/// @param num_checks Number of checks of motion validity, which depends on maximal velocity of obstacles.
/// @return The validity of motion.
/// @note In reality, this motion would happen continuously during the execution of the current algorithm iteration.
bool planning::drbt::DRGBT::checkMotionValidity2(int num_checks)
{
    // std::cout << "Checking the validity of motion while updating environment... \n";
    std::shared_ptr<base::State> q_temp;
    float dist = ss->getNorm(q_previous, q_current);
    float delta_time = DRGBTConfig::MAX_ITER_TIME * 1e-3 / num_checks;
    bool is_valid = true;

    for (int num_check = 1; num_check <= num_checks; num_check++)
    {
        ss->env->updateEnvironment(delta_time);
        q_temp = ss->interpolateEdge(q_previous, q_current, dist * num_check / num_checks, dist);
        path.emplace_back(q_temp);
        is_valid = ss->isValid(q_temp);
        if (!is_valid)
            break;
    }

    // for (int i = 0; i < ss->env->getNumObjects(); i++)
    //     std::cout << "i = " << i << " : " << ss->env->getObject(i)->getPosition().transpose() << "\n";

    return is_valid;
}

/// @brief Discretely check the validity of motion when the robot ('q_current') moves over splines 'spline_curr' and 'spline_next'
/// during the current iteration, while the obstacles are moving simultaneously.
/// Finally, the environment is updated within this function.
/// @param num_checks Number of checks of motion validity, which depends on maximal velocity of obstacles.
/// @return The validity of motion.
/// @note In reality, this motion would happen continuously during the execution of the current algorithm iteration.
bool planning::drbt::DRGBT::checkMotionValidity(int num_checks)
{
    // std::cout << "Checking the validity of motion while updating environment... \n";
    int num_checks1 = std::ceil((spline_current->getTimeCurrent() - spline_current->getTimeBegin()) * num_checks 
                                / (DRGBTConfig::MAX_ITER_TIME * 1e-3));
    int num_checks2 = num_checks - num_checks1;
    float delta_time1 = (spline_current->getTimeCurrent() - spline_current->getTimeBegin()) / num_checks1;
    float delta_time2 = (spline_next->getTimeEnd() - spline_next->getTimeCurrent()) / num_checks2;
    float t;
    bool is_valid = true;
    
    // std::cout << "Current spline times:   " << spline_current->getTimeBegin() * 1000 << " [ms] \t"
    //                                         << spline_current->getTimeCurrent() * 1000 << " [ms] \t"
    //                                         << spline_current->getTimeEnd() * 1000 << " [ms] \n";
    // std::cout << "Next spline times:      " << spline_next->getTimeBegin() * 1000 << " [ms] \t"
    //                                         << spline_next->getTimeCurrent() * 1000 << " [ms] \t"
    //                                         << spline_next->getTimeEnd() * 1000 << " [ms] \n";

    for (int num_check = 1; num_check <= num_checks; num_check++)
    {
        if (num_check <= num_checks1)
        {
            t = spline_current->getTimeBegin() + num_check * delta_time1;
            q_current = ss->getNewState(spline_current->getPosition(t));
            std::cout << "t: " << t * 1000 << " [ms]\t from curr. spline \t" << q_current << "\n";
            ss->env->updateEnvironment(delta_time1);
        }
        else
        {
            t = spline_next->getTimeCurrent() + (num_check - num_checks1) * delta_time2;
            q_current = ss->getNewState(spline_next->getPosition(t));
            std::cout << "t: " << t * 1000 << " [ms]\t from next  spline \t" << q_current << "\n";
            ss->env->updateEnvironment(delta_time2);
        }

        path.emplace_back(q_current);
        is_valid = ss->isValid(q_current);
        if (!is_valid || ss->isEqual(q_current, q_goal))
            break;
    }
    
    // for (int i = 0; i < ss->env->getNumObjects(); i++)
    //     std::cout << "i = " << i << " : " << ss->env->getObject(i)->getPosition().transpose() << "\n";

    return is_valid;
}

bool planning::drbt::DRGBT::checkTerminatingCondition()
{
    int t_spline_current = getElapsedTime(time_start, std::chrono::steady_clock::now());    
    // std::cout << "Time elapsed: " << t_spline_current << " [ms] \n";
    if (ss->isEqual(q_current, q_goal))
    {
        std::cout << "Goal configuration has been successfully reached! \n";
		planner_info->setSuccessState(true);
        planner_info->setPlanningTime(t_spline_current);
        return true;
    }
	
    if (t_spline_current >= DRGBTConfig::MAX_PLANNING_TIME)
	{
        std::cout << "Maximal planning time has been reached! \n";
		planner_info->setSuccessState(false);
        planner_info->setPlanningTime(t_spline_current);
		return true;
	}
    
    if (planner_info->getNumIterations() >= DRGBTConfig::MAX_NUM_ITER)
	{
        std::cout << "Maximal number of iterations has been reached! \n";
		planner_info->setSuccessState(false);
        planner_info->setPlanningTime(t_spline_current);
		return true;
	}

	return false;
}

void planning::drbt::DRGBT::outputPlannerData(const std::string &filename, bool output_states_and_paths, bool append_output) const
{
	std::ofstream output_file;
	std::ios_base::openmode mode = std::ofstream::out;
	if (append_output)
		mode = std::ofstream::app;

	output_file.open(filename, mode);
	if (output_file.is_open())
	{
		output_file << "Space Type:      " << ss->getStateSpaceType() << std::endl;
		output_file << "Dimensionality:  " << ss->num_dimensions << std::endl;
		output_file << "Planner type:    " << "DRGBT" << std::endl;
		output_file << "Planner info:\n";
		output_file << "\t Succesfull:           " << (planner_info->getSuccessState() ? "yes" : "no") << std::endl;
		output_file << "\t Number of iterations: " << planner_info->getNumIterations() << std::endl;
		output_file << "\t Planning time [ms]:   " << planner_info->getPlanningTime() << std::endl;
		if (output_states_and_paths)
		{
			if (path.size() > 0)
			{
				output_file << "Path:" << std::endl;
				for (int i = 0; i < path.size(); i++)
					output_file << path.at(i) << std::endl;
			}
		}
		output_file << std::string(25, '-') << std::endl;		
		output_file.close();
	}
	else
		throw "Cannot open file"; // std::something exception perhaps?
}
