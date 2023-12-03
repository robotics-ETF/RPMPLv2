//
// Created by nermin on 13.04.22.
//

#include "DRGBT.h"

#include <glog/log_severity.h>
#include <glog/logging.h>
// WARNING: You need to be very careful with LOG(INFO) for console output, due to a possible "stack smashing detected" error.
// If you get this error, just use std::cout for console output.

planning::drbt::DRGBT::DRGBT(std::shared_ptr<base::StateSpace> ss_) : RGBTConnect(ss_) {}

planning::drbt::DRGBT::DRGBT(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_,
                             std::shared_ptr<base::State> goal_, const std::string &planner_name_) :
                             RGBTConnect(ss_)
{
	// std::cout << "Initializing planner... \n";
    start = start_;
    goal = goal_;
	if (!ss->isValid(start))
		throw std::domain_error("Start position is invalid!");
    
    q_current = start;
    q_prev = q_current;
    q_next = std::make_shared<HorizonState>(q_current, 0);
    q_next_previous = q_next;
    d_max_mean = 0;
    horizon_size = DRGBTConfig::INIT_HORIZON_SIZE + num_lateral_states;
    replanning = false;
    status = base::State::Status::Reached;
    planner_info->setNumStates(1);
	planner_info->setNumIterations(0);
    planner_name = planner_name_;
    path.emplace_back(start);                                   // State 'start' is added to the realized path
	// std::cout << "Planner initialized! \n";
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

    // Initial iteration: Obtaining the inital path using specified static planner
    // std::cout << "\n\nIteration num. " << planner_info->getNumIterations() << "\n";
    // std::cout << "Obtaining the inital path... \n";
    replan(DRGBTConfig::MAX_ITER_TIME);
    replanning_cnt++;
    planner_info->setNumIterations(planner_info->getNumIterations() + 1);
    planner_info->addIterationTime(getElapsedTime(time_iter_start, std::chrono::steady_clock::now()));
    // std::cout << "----------------------------------------------------------------------------------------\n";

    while (true)
    {
        // std::cout << "TASK 1: Computing next configuration... \n";
        // std::cout << "\n\nIteration num. " << planner_info->getNumIterations() << "\n";
        time_iter_start = std::chrono::steady_clock::now();     // Start the iteration clock

        // Update environment and check if the collision occurs
        bool is_valid = checkMotionValidity();              // < 10 [us]

        // Since the environment has changed, a new distance is required!
        // auto time_computeDistance = std::chrono::steady_clock::now();
        float d_c = ss->computeDistance(q_current, true);   // < 1 [ms]
        // planner_info->addRoutineTime(getElapsedTime(time_computeDistance, std::chrono::steady_clock::now(), "microseconds"), 2);

        if (!is_valid || d_c <= 0)
        {
            std::cout << "Collision has been occured!!! \n";
            planner_info->setSuccessState(false);
            planner_info->setPlanningTime(planner_info->getIterationTimes().back());
            return false;
        }

        // ------------------------------------------------------------------------------- //
        if (status != base::State::Status::Advanced)
            generateHorizon();          // < 1 [us]
        
        updateHorizon(d_c);             // < 1 [us]
        generateGBur();                 // Time consuming routine...
        computeNextState();             // < 1 [us]
        updateCurrentState();           // < 1 [us]
        
        // ------------------------------------------------------------------------------- //
        // Replanning procedure assessment
        if (replanning_cnt > 0 && (replanning || whetherToReplan()))
        {
            // std::cout << "TASK 2: Replanning... \n";
            float time_remaining = DRGBTConfig::MAX_ITER_TIME - getElapsedTime(time_iter_start, std::chrono::steady_clock::now());
            replan(time_remaining);
        }
        // else
        //     std::cout << "Replanning is not required! \n";

        // ------------------------------------------------------------------------------- //
        // Checking the real-time execution
        auto time_current = std::chrono::steady_clock::now();
        // float time_iter_remain = DRGBTConfig::MAX_ITER_TIME - getElapsedTime(time_iter_start, time_current);
        // // std::cout << "Remaining iteration time is " << time_iter_remain << " [ms]. \n";
        // if (time_iter_remain < 0)
        //     std::cout << "*************** Real-time is broken. " << -time_iter_remain << " [ms] exceeded!!! *************** \n";

        // ------------------------------------------------------------------------------- //
        // Planner info and terminating condition
        planner_info->setNumIterations(planner_info->getNumIterations() + 1);
        planner_info->addIterationTime(getElapsedTime(time_start, time_current));
        if (checkTerminatingCondition())
            return planner_info->getSuccessState();

        replanning_cnt++;
        // std::cout << "----------------------------------------------------------------------------------------\n";
    }
}

// Generate a horizon using predefined path (or random nodes).
// Only states from predefined path that come after 'q_next' are remained in the horizon. Other states are deleted.
void planning::drbt::DRGBT::generateHorizon()
{
    // std::cout << "Generating horizon... \n";
    // auto time_generateHorizon = std::chrono::steady_clock::now();

    if (!horizon.empty())   // 'q_next' is reached. No replanning nor 'status' is trapped
    {
        for (int i = horizon_size - 1; i >= 0; i--)
        {
            if (horizon[i]->getIndex() <= q_next->getIndex())
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
            idx = q_next->getIndex() + 1;
        
        // std::cout << "Adding states from the predefined path starting from " << idx << ". state... \n";
        if (idx + num_states <= predefined_path.size())
        {
            for (int i = idx; i < idx + num_states; i++)
                horizon.emplace_back(std::make_shared<HorizonState>(predefined_path[i], i));
        }
        else if (idx < predefined_path.size())
        {
            for (int i = idx; i < predefined_path.size(); i++)
                horizon.emplace_back(std::make_shared<HorizonState>(predefined_path[i], i));
            horizon.back()->setStatus(HorizonState::Status::Goal);
            replanning = false;
        }
    }
    else            // status == base::State::Status::Trapped || predefined_path.empty()
    {
        replanning = true;
        addRandomStates(num_states);
    }
    
    // Set the next state just for obtaining lateral spines
    q_next = horizon.front();

    // std::cout << "Initial horizon consists of " << horizon.size() << " states: \n";
    // for (int i = 0; i < horizon.size(); i++)
    //     std::cout << i << ". state:\n" << horizon[i] << "\n";
    
    // planner_info->addRoutineTime(getElapsedTime(time_generateHorizon, std::chrono::steady_clock::now(), "microseconds"), 4);
}

// Update the horizon size, and add lateral spines.
void planning::drbt::DRGBT::updateHorizon(float d_c)
{
    // std::cout << "Robot current state: " << q_current->getCoord().transpose() << " with d_c: " << d_c << "\n";
    // auto time_updateHorizon = std::chrono::steady_clock::now();
    horizon_size = std::min(int(DRGBTConfig::INIT_HORIZON_SIZE * (1 + DRGBTConfig::D_CRIT / d_c)),
                            ss->getNumDimensions() * DRGBTConfig::INIT_HORIZON_SIZE);
    
    // std::cout << "Modifying horizon size from " << horizon.size() << " to " << horizon_size << "\n";
    if (horizon_size < horizon.size())
        shortenHorizon(horizon.size() - horizon_size);
    else if (horizon_size > horizon.size())    // If 'horizon_size' has increased, or not enough states exist, then random states are added
        addRandomStates(horizon_size - horizon.size());

    // Lateral states are added
    // std::cout << "Adding " << num_lateral_states << " lateral states... \n";
    addLateralStates();
    horizon_size = horizon.size();
    // planner_info->addRoutineTime(getElapsedTime(time_updateHorizon, std::chrono::steady_clock::now(), "microseconds"), 5);
}

// Generate the generalized bur from 'q_current', i.e., compute the horizon spines.
// Bad and critical states will be replaced with "better" states, such that the horizon contains possibly better states.
void planning::drbt::DRGBT::generateGBur()
{
    // std::cout << "Generating gbur by computing reached states... \n";
    // auto time_generateGBur = std::chrono::steady_clock::now();
    int idx;
    for (idx = 0; idx < horizon.size(); idx++)
    {
        computeReachedState(q_current, horizon[idx]);
        // std::cout << idx << ". state:\n" << horizon[idx] << "\n";

        // Bad and critical states are modified
        if (horizon[idx]->getStatus() == HorizonState::Status::Bad || 
            horizon[idx]->getStatus() == HorizonState::Status::Critical)
                modifyState(horizon[idx]);
        
        if (!DRGBTConfig::REAL_TIME_SCHEDULING.empty() &&   // Some scheduling is chosen
            getElapsedTime(time_iter_start, std::chrono::steady_clock::now()) > DRGBTConfig::TASK1_UTILITY * DRGBTConfig::MAX_ITER_TIME)
        {
            // Delete horizon states for which there is no enough remaining time to be processed
            for (int i = horizon.size() - 1; i > idx; i--)  
                horizon.erase(horizon.begin() + i);
            
            // if (horizon_size > horizon.size())
            //     std::cout << "Deleted " << horizon_size - horizon.size() << " horizon states. \n";

            horizon_size = horizon.size();
            break;
        }
    }
    // planner_info->addRoutineTime(getElapsedTime(time_generateGBur, std::chrono::steady_clock::now()), 3);
}

// Shorten the horizon by removing 'num' states. Excess states are deleted, and best states holds priority.
void planning::drbt::DRGBT::shortenHorizon(int num)
{
    int num_deleted = 0;
    for (int i = horizon.size() - 1; i >= 0; i--)
    {
        if (horizon[i]->getStatus() == HorizonState::Status::Bad)
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
    for (int i = 0; i < num; i++)
    {
        std::shared_ptr<base::State> q_rand = getRandomState(q_current);
        horizon.emplace_back(std::make_shared<HorizonState>(q_rand, -1));
        // std::cout << "Adding random state: " << horizon.back()->getCoord().transpose() << "\n";
    }
}

void planning::drbt::DRGBT::addLateralStates()
{
    int num_added = 0;
    if (ss->getNumDimensions() == 2)   // In 2D C-space only two possible lateral spines exist
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
                horizon.emplace_back(std::make_shared<HorizonState>(q_new, -1));
                num_added++;
                // std::cout << "Adding lateral state: " << horizon.back()->getCoord().transpose() << "\n";
            }
        }
    }
    else
    {
        int idx = -1;
        for (int k = 0; k < ss->getNumDimensions(); k++)
            if (std::abs(q_next->getCoord(k) - q_current->getCoord(k)) > 1e-3)
                break;
            
        if (idx != -1)
        {
            std::shared_ptr<base::State> q_new;
            float coord;
            for (int i = 0; i < num_lateral_states; i++)
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
                    horizon.emplace_back(std::make_shared<HorizonState>(q_new, -1));
                    num_added++;
                    // std::cout << "Adding lateral state: " << horizon.back()->getCoord().transpose() << "\n";
                }
            }
        }
    }
    addRandomStates(num_lateral_states - num_added);
}

// Modify state by replacing it with a random state, which is generated using biased distribution,
// i.e., oriented weight around 'q' ('q->getStatus()' == bad), or around '-q' ('q->getStatus()' == critical).
// Return whether the modification is successful
bool planning::drbt::DRGBT::modifyState(std::shared_ptr<HorizonState> &q)
{
    std::shared_ptr<HorizonState> q_new_;
    std::shared_ptr<base::State> q_new;
    std::shared_ptr<base::State> q_reached = q->getStateReached();
    float norm = ss->getNorm(q_current, q_reached);
    float coeff = 0;
    int num = 0;
    int num_attepmts = 10;
    while (num++ < num_attepmts)
    {
        Eigen::VectorXf vec = Eigen::VectorXf::Random(ss->getNumDimensions()) * norm / std::sqrt(ss->getNumDimensions() - 1);
        vec(0) = (vec(0) > 0) ? 1 : -1;
        vec(0) *= std::sqrt(norm * norm - vec.tail(ss->getNumDimensions() - 1).squaredNorm());
        if (q->getStatus() == HorizonState::Status::Bad)
            q_new = ss->getNewState(q_reached->getCoord() + vec);
        else if (q->getStatus() == HorizonState::Status::Critical)
        {
            q_new = ss->getNewState(2 * q_current->getCoord() - q_reached->getCoord() + coeff * vec);
            coeff = 1;
        }
        q_new = ss->interpolateEdge(q_current, q_new, RBTConnectConfig::DELTA);
        q_new = ss->pruneEdge(q_current, q_new);
        if (!ss->isEqual(q_current, q_new))
        {
            q_new_ = std::make_shared<HorizonState>(q_new, -1);
            computeReachedState(q_current, q_new_);
            if (q_new_->getDistance() > q->getDistance())
            {
                q = q_new_;
                // std::cout << "Modifying this state with the following state:\n" << q << "\n";
                return true;
            }
            // else
            //     std::cout << "Modifying this state is not successfull (" << num << ". attempt)! \n";
        }
    }   
    return false;
}

// Compute reached state when generating a generalized spine from 'q_current' towards 'q'.
void planning::drbt::DRGBT::computeReachedState(std::shared_ptr<base::State> q_current, std::shared_ptr<HorizonState> q)
{
    base::State::Status status;
    std::shared_ptr<base::State> q_reached;
    tie(status, q_reached) = extendGenSpine(q_current, q->getState());
    q->setStateReached(q_reached);
    q->setIsReached(status == base::State::Status::Reached ? true : false);

    // TODO: If there is enough remaining time, compute real distance-to-obstacles 
    float d_c = ss->computeDistanceUnderestimation(q_reached, q_current->getNearestPoints());
    if (q->getDistancePrevious() == -1)
        q->setDistancePrevious(d_c);
    else
        q->setDistancePrevious(q->getDistance());
    
    q->setDistance(d_c);
    
    // Check whether the goal is reached
    if (q->getIndex() > 0 && ss->isEqual(q_reached, goal))
        q->setStatus(HorizonState::Status::Goal);
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
        dist_to_goal[i] = ss->getNorm(horizon[i]->getStateReached(), goal);
        if (dist_to_goal[i] < d_goal_min)
        {
            d_goal_min = dist_to_goal[i];
            d_goal_min_idx = i;
        }
        if (horizon[i]->getDistance() > d_c_max)
            d_c_max = horizon[i]->getDistance();
    }
    
    if (d_goal_min == 0)      // 'goal' lies in the horizon
    {
        d_goal_min = 1e-6;    // Only to avoid "0/0" when 'dist_to_goal[i] == 0'
        dist_to_goal[d_goal_min_idx] = d_goal_min;
    }

    std::vector<float> weights_dist(horizon.size());
    for (int i = 0; i < horizon.size(); i++)
        weights_dist[i] = d_goal_min / dist_to_goal[i];        
    
    d_max_mean = (planner_info->getNumIterations() * d_max_mean + d_c_max) / (planner_info->getNumIterations() + 1);
    float weights_dist_mean = std::accumulate(weights_dist.begin(), weights_dist.end(), 0.0) / horizon.size();
    float max_weight = 0;
    float weight;
    float d_min;

    for (int i = 0; i < horizon.size(); i++)
    {
        if (horizon[i]->getStatus() == HorizonState::Status::Critical)  // 'weight' is already set to zero
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

        // If state does not belong to the predefined path, 'weight' is decreased within the range [0, DRGBTConfig::WEIGHT_MIN]
        // If such state becomes the best one, the replanning will surely be triggered
        if (horizon[i]->getIndex() == -1)   
            weight *= DRGBTConfig::WEIGHT_MIN;
        
        horizon[i]->setWeight(weight);

        if (weight > max_weight)
        {
            max_weight = weight;
            d_min = dist_to_goal[i];
            q_next = horizon[i];
        }
    }

    // Do the following only if 'q_next' belongs to the predefined path
    if (q_next->getIndex() != -1)
    {
        // The best state nearest to the goal is chosen as the next state
        for (int i = 0; i < horizon.size(); i++)
        {
            if (std::abs(q_next->getWeight() - horizon[i]->getWeight()) < hysteresis && 
                dist_to_goal[i] < d_min)
            {
                d_min = dist_to_goal[i];
                q_next = horizon[i];
            }
        }

        // If weights of 'q_next_previous' and 'q_next' are close, 'q_next_previous' remains the next state
        if (q_next != q_next_previous &&
            std::abs(q_next->getWeight() - q_next_previous->getWeight()) < hysteresis &&
            q_next->getStatus() != HorizonState::Status::Goal &&
            getIndexInHorizon(q_next_previous) != -1)
                q_next = q_next_previous;
    }

    // std::cout << "Setting the robot next state to: " << q_next->getCoord().transpose() << "\n";
    q_next_previous = q_next;
    
    // std::cout << "Horizon consists of " << horizon.size() << " states: \n";
    // for (int i = 0; i < horizon.size(); i++)
    //     std::cout << i << ". state:\n" << horizon[i] << "\n";
}

// Return index in the horizon of state 'q'. If 'q' does not belong to the horizon, -1 is returned.
int planning::drbt::DRGBT::getIndexInHorizon(std::shared_ptr<HorizonState> q)
{
    for (int idx = 0; idx < horizon.size(); idx++)
    {
        if (q == horizon[idx])
            return idx;
    }
    return -1;   
}

// Update the current state of the robot by moving from 'q_current' towards 'q_next' for step size DRGBTConfig::STEP
void planning::drbt::DRGBT::updateCurrentState()
{
    q_prev = q_current;
    std::shared_ptr<base::State> q_new;
    tie(status, q_new) = ss->interpolateEdge2(q_current, q_next->getStateReached(), DRGBTConfig::STEP);
    if (!ss->isEqual(q_current, q_new))
    {
        q_new->setParent(q_current);
        q_current = q_new;
        if (!q_next->getIsReached())
            status = base::State::Status::Advanced;
        // std::cout << "Updating the robot current state to: " << q_current->getCoord().transpose() << "\n";
    }
    else
    {
        status = base::State::Status::Trapped;
        replanning = true;
        horizon.clear();
        // std::cout << "Not updating the robot current state! \n";
    }
    path.emplace_back(q_current);

    // std::cout << "Status: " << (status == base::State::Status::Advanced ? "Advanced" : "")
    //                         << (status == base::State::Status::Trapped  ? "Trapped"  : "")
    //                         << (status == base::State::Status::Reached  ? "Reached"  : "") << "\n";
}

bool planning::drbt::DRGBT::whetherToReplan()
{
    float weight_max = 0;
    float weight_sum = 0;
    for (int i = 0; i < horizon.size(); i++)
    {
        weight_max = std::max(weight_max, horizon[i]->getWeight());
        weight_sum += horizon[i]->getWeight();
    }
    return (weight_max <= DRGBTConfig::WEIGHT_MIN && 
            weight_sum / horizon.size() <= DRGBTConfig::WEIGHT_MEAN_MIN) 
            ? true : false;
}

std::unique_ptr<planning::AbstractPlanner> planning::drbt::DRGBT::initStaticPlanner(float max_planning_time)
{
    // std::cout << "Static planner (for replanning): " << planner_name << "\n";
    if (planner_name == "RRTConnect")
    {
        RRTConnectConfig::MAX_PLANNING_TIME = max_planning_time;
        return std::make_unique<planning::rrt::RRTConnect>(ss, q_current, goal);
    }
    else if (planner_name == "RBTConnect")
    {
        RBTConnectConfig::MAX_PLANNING_TIME = max_planning_time;
        return std::make_unique<planning::rbt::RBTConnect>(ss, q_current, goal);
    }
    else if (planner_name == "RGBTConnect")
    {
        RGBTConnectConfig::MAX_PLANNING_TIME = max_planning_time;
        return std::make_unique<planning::rbt::RGBTConnect>(ss, q_current, goal);
    }
    else if (planner_name == "RGBMT*")
    {
        RGBMTStarConfig::MAX_PLANNING_TIME = max_planning_time;
        return std::make_unique<planning::rbt_star::RGBMTStar>(ss, q_current, goal);
    }
    else
        throw std::domain_error("The requested static planner " + planner_name + " is not found! ");
}

// Try to replan the predefined path from current to goal configuration within the specified time
void planning::drbt::DRGBT::replan(float max_planning_time)
{
    std::unique_ptr<planning::AbstractPlanner> planner;
    bool result = false;
    replanning_cnt = 0;     // Replanning is completed within a single iteration

    try
    {
        if (max_planning_time < 0)
            throw std::runtime_error("Not enough time for replanning! ");

        if (DRGBTConfig::REAL_TIME_SCHEDULING == "DPS")         // Dynamic Priority Scheduling
        {
            // std::cout << "Replanning with Dynamic Priority Scheduling \n";
            float max_planning_time_new = max_planning_time + (1 - DRGBTConfig::TASK1_UTILITY) * DRGBTConfig::MAX_ITER_TIME;
            // std::cout << "Trying to replan in " << max_planning_time_new << " [ms]... \n";

            planner = initStaticPlanner(max_planning_time_new);
            result = planner->solve();
            if (max_planning_time_new - planner->getPlannerInfo()->getPlanningTime() 
                < (1 - DRGBTConfig::TASK1_UTILITY) * DRGBTConfig::MAX_ITER_TIME)
                    replanning_cnt = -1;    // Replanning is NOT completed within a single iteration
        }
        else if (DRGBTConfig::REAL_TIME_SCHEDULING == "FPS")    // Fixed Priority Scheduling
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

        // New path is found within the specified time limit, thus update the predefined path to goal
        if (result && planner->getPlannerInfo()->getPlanningTime() <= max_planning_time)
        {
            // std::cout << "The path has been replanned in " << planner->getPlannerInfo()->getPlanningTime() << " [ms]. \n";
            acquirePredefinedPath(planner->getPath(), std::sqrt(ss->getNumDimensions()) * DRGBTConfig::STEP);
            replanning = false;
            status = base::State::Status::Reached;
            horizon.clear();
            q_next = std::make_shared<HorizonState>(predefined_path.front(), 0);
            // planner_info->addRoutineTime(planner->getPlannerInfo()->getPlanningTime(), 0);  // replan
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

void planning::drbt::DRGBT::acquirePredefinedPath(const std::vector<std::shared_ptr<base::State>> &path_, float delta_q_max)
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

// Discretely check the validity of motion when robot moves from 'q_prev' to 'q_current', while the obstacles are moving simultaneously.
// Finally, the environment is updated within this function.
// Generally, 'num_checks' depends on maximal velocity of obstacles.
bool planning::drbt::DRGBT::checkMotionValidity(int num_checks)
{
    // std::cout << "Updating environment... \n";
    std::shared_ptr<base::State> q_temp;
    float dist = ss->getNorm(q_prev, q_current);
    float random = float(rand()) / RAND_MAX;

    for (float k = 1; k <= std::ceil(num_checks * random); k++)
    {
        ss->env->updateEnvironment(random / num_checks);
        q_temp = ss->interpolateEdge(q_prev, q_current, k / num_checks * dist, dist);

        // auto time_isValid = std::chrono::steady_clock::now();
        bool is_valid = ss->isValid(q_temp);
        // if (k == 1)     // Just to reduce the number of measurements
        //     planner_info->addRoutineTime(getElapsedTime(time_isValid, std::chrono::steady_clock::now(), "microseconds"), 1);
    
        if (!is_valid)
            return false;
    }
    return true;
}

bool planning::drbt::DRGBT::checkTerminatingCondition()
{
    auto time_current = getElapsedTime(time_start, std::chrono::steady_clock::now());    
    // std::cout << "Time elapsed: " << time_current << " [ms] \n";
    if (ss->isEqual(q_current, goal))
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

void planning::drbt::DRGBT::outputPlannerData(std::string filename, bool output_states_and_paths, bool append_output) const
{
	std::ofstream output_file;
	std::ios_base::openmode mode = std::ofstream::out;
	if (append_output)
		mode = std::ofstream::app;

	output_file.open(filename, mode);
	if (output_file.is_open())
	{
		output_file << "Space Type:      " << ss->getStateSpaceType() << std::endl;
		output_file << "Dimensionality:  " << ss->getNumDimensions() << std::endl;
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
