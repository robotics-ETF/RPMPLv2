//
// Created by nermin on 13.04.22.
//

#include "DRGBTConnect.h"

#include <glog/log_severity.h>
#include <glog/logging.h>
// WARNING: You need to be very careful with LOG(INFO) for console output, due to a possible "stack smashing detected" error.
// If you get this error, just use std::cout for console output.

planning::drbt::DRGBTConnect::DRGBTConnect(std::shared_ptr<base::StateSpace> ss_) : RGBTConnect(ss_) {}

planning::drbt::DRGBTConnect::DRGBTConnect(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_,
                                           std::shared_ptr<base::State> goal_, const std::string &planner_name_) : 
                                           RGBTConnect(ss_, start_, goal_) 
{
    // Additionally the following is required:
    q_current = start;
    q_next = std::make_shared<HorizonState>(q_current, 0);
    q_next_previous = q_next;
    d_max_mean = 0;
    horizon_size = DRGBTConnectConfig::INIT_HORIZON_SIZE + num_lateral_states;
    replanning = false;
    status = base::State::Status::Reached;
    planner_info->setNumStates(1);
    planner_name = planner_name_;
    path.emplace_back(start);                                   // State 'start' is added to the realized path
}

bool planning::drbt::DRGBTConnect::solve()
{
    time_start = std::chrono::steady_clock::now();     // Start the algorithm clock
    auto time_iter_start = time_start;

    // Initial iteration: Obtaining the inital path using specified static planner
    // LOG(INFO) << "\n\nIteration num. " << planner_info->getNumIterations();
    // LOG(INFO) << "Obtaining the inital path...";
    replan(DRGBTConnectConfig::MAX_ITER_TIME);	 
    planner_info->setNumIterations(planner_info->getNumIterations() + 1);
    planner_info->addIterationTime(getElapsedTime(time_iter_start, std::chrono::steady_clock::now()));
    // LOG(INFO) << "----------------------------------------------------------------------------------------\n";

    while (true)
    {
        // LOG(INFO) << "\n\nIteration num. " << planner_info->getNumIterations();
        time_iter_start = std::chrono::steady_clock::now();     // Start the iteration clock

        // Update environment and check if the collision occurs
        // LOG(INFO) << "Updating environment...";
        ss->env->updateEnvironment();
        if (!ss->isValid(q_current))
        {
            // LOG(INFO) << "Collision has been occured!!! ";
            std::cout << "Collision has been occured!!! \n";
            planner_info->setSuccessState(false);
            planner_info->setPlanningTime(planner_info->getIterationTimes().back());
            return false;
        }

        if (status != base::State::Status::Advanced)
        {
            // Generate a horizon
            generateHorizon();
            // LOG(INFO) << "Initial horizon consists of " << horizon.size() << " states: ";
            // for (int i = 0; i < horizon.size(); i++)
            //     LOG(INFO) << i << ". state:\n" << horizon[i]; 
        }
        
        // Moving from 'q_current' towards 'q_next' for step 'DRGBTConnectConfig::STEP', where 'q_next' may change        
        // Compute the horizon and the next state
        computeHorizon();
        computeNextState();
        // LOG(INFO) << "Horizon consists of " << horizon.size() << " states: ";
        // for (int i = 0; i < horizon.size(); i++)
        //     LOG(INFO) << i << ". state:\n" << horizon[i];

        // Update the robot current state
        updateCurrentState();
        // LOG(INFO) << "Status: " << (status == base::State::Status::Advanced ? "Advanced" : "")
        //                         << (status == base::State::Status::Trapped  ? "Trapped"  : "")
        //                         << (status == base::State::Status::Reached  ? "Reached"  : "");
        
        // Replanning procedure assessment
        if (replanning || whetherToReplan())
        {
            float time_remaining = DRGBTConnectConfig::MAX_ITER_TIME 
                                   - getElapsedTime(time_iter_start, std::chrono::steady_clock::now()) 
                                   - 1;     // 1 [ms] is reserved for the following code lines
            replan(time_remaining);
        }
        // else
        //     LOG(INFO) << "Replanning is not required! ";

        // Checking the real-time execution
        auto time_current = std::chrono::steady_clock::now();
        float time_iter_remain = DRGBTConnectConfig::MAX_ITER_TIME - getElapsedTime(time_iter_start, time_current);
        // LOG(INFO) << "Remaining iteration time is " << time_iter_remain << " [ms]. ";
        if (time_iter_remain < 0)
        {
            // LOG(INFO) << "*************** Real-time is broken. " << -time_iter_remain << " [ms] exceeded!!! ***************";
            std::cout << "*************** Real-time is broken. " << -time_iter_remain << " [ms] exceeded!!! *************** \n";
        }

        // Planner info and terminating condition
        planner_info->setNumIterations(planner_info->getNumIterations() + 1);
        planner_info->addIterationTime(getElapsedTime(time_start, time_current));
        if (checkTerminatingCondition())
            return planner_info->getSuccessState();
        
        // LOG(INFO) << "----------------------------------------------------------------------------------------\n";
    }
}

// Generate a horizon using predefined path (or random nodes).
// Only states from predefined path that come after 'q_next' are remained in the horizon. Other states are deleted.
void planning::drbt::DRGBTConnect::generateHorizon()
{
    // auto T = std::chrono::steady_clock::now();
    // LOG(INFO) << "Preparing horizon...";
    if (!horizon.empty())   // 'q_next' is reached. No replanning nor 'status' is trapped
    {
        for (int i = horizon_size - 1; i >= 0; i--)
        {
            if (horizon[i]->getIndex() <= q_next->getIndex())
            {
                horizon.erase(horizon.begin() + i);
                // LOG(INFO) << "Deleting state " << i << ". Horizon size is " << horizon.size();
            }
        }
    }

    // Generating horizon
    int num_states = horizon_size - (horizon.size() + num_lateral_states);
    if (status == base::State::Status::Reached)
    {
        int idx;    // Designates from which state in predefined path new states are added to the horizon
        if (!horizon.empty())
            idx = horizon.back()->getIndex() + 1;
        else
            idx = q_next->getIndex() + 1;
        
        // LOG(INFO) << "Adding states from the predefined path starting from " << idx << ". state... ";
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
    else            // status == base::State::Status::Trapped
    {
        replanning = true;
        addRandomStates(num_states);
    }
    
    // Set the next state just for obtaining lateral spines
    q_next = horizon.front();

    // planner_info->addRoutineTime(getElapsedTime(T, std::chrono::steady_clock::now(), "microseconds"), 0);
}

// Update and then compute the horizon.
// Bad and critical states will be replaced with "better" states, such that the horizon contains possibly better states.
void planning::drbt::DRGBTConnect::computeHorizon()
{
    q_current->setDistance(-1);                 // In order to be computed again since the environment has changed
    float d_c = ss->computeDistance(q_current);

    // auto T1 = std::chrono::steady_clock::now();
    // LOG(INFO) << "Robot current state: " << q_current->getCoord().transpose() << " with d_c: " << d_c;
    horizon_size = std::min((int) std::floor(DRGBTConnectConfig::INIT_HORIZON_SIZE * (1 + DRGBTConnectConfig::D_CRIT / d_c)),
                            5 * ss->getNumDimensions() * DRGBTConnectConfig::INIT_HORIZON_SIZE);
    
    // LOG(INFO) << "Modifying horizon size from " << horizon.size() << " to " << horizon_size;
    if (horizon_size < horizon.size())
        shortenHorizon(horizon.size() - horizon_size);
    else if (horizon_size > horizon.size())    // If 'horizon_size' has increased, or little states exist, then random states are added
        addRandomStates(horizon_size - horizon.size());

    // Lateral states are added
    // LOG(INFO) << "Adding " << num_lateral_states << " lateral states...";
    addLateralStates();
    horizon_size = horizon.size();

    // planner_info->addRoutineTime(getElapsedTime(T1, std::chrono::steady_clock::now(), "microseconds"), 1);

    // LOG(INFO) << "Generating gbur by computing reached states...";
    // auto T2 = std::chrono::steady_clock::now();
    for (int i = 0; i < horizon.size(); i++)
    {
        computeReachedState(q_current, horizon[i]);
        // LOG(INFO) << i << ". state:\n" << horizon[i];

        // Bad and critical states are modified
        if (horizon[i]->getStatus() == HorizonState::Status::Bad || 
            horizon[i]->getStatus() == HorizonState::Status::Critical)
                modifyState(horizon[i]);
    }

    // planner_info->addRoutineTime(getElapsedTime(T2, std::chrono::steady_clock::now()), 3);
}

// Shorten the horizon by removing 'num' states. Excess states are deleted, and best states holds priority.
void planning::drbt::DRGBTConnect::shortenHorizon(int num)
{
    int num_deleted = 0;
    for (int i = horizon.size() - 1; i >= 0; i--)
    {
        if (horizon[i]->getStatus() == HorizonState::Status::Bad)
        {
            horizon.erase(horizon.begin() + i);
            num_deleted++;
            // LOG(INFO) << "Deleting state " << i << " in stage 1";
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
            // LOG(INFO) << "Deleting state " << i << " in stage 2";
        }
        if (num_deleted == num)
            return;
    }
    for (int i = horizon.size() - 1; i >= 0; i--)
    {
        horizon.erase(horizon.begin() + i);
        num_deleted++;
        // LOG(INFO) << "Deleting state " << i << " in stage 3";
        if (num_deleted == num)
            return;
    }
}

void planning::drbt::DRGBTConnect::addRandomStates(int num)
{
    std::shared_ptr<base::State> q_rand;
    for (int i = 0; i < num; i++)
    {
        q_rand = getRandomState(q_current);
        horizon.emplace_back(std::make_shared<HorizonState>(q_rand, -1));
        // LOG(INFO) << "Adding random state: " << horizon.back()->getCoord().transpose();
    }
}

void planning::drbt::DRGBTConnect::addLateralStates()
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
                // LOG(INFO) << "Adding lateral state: " << horizon.back()->getCoord().transpose();
            }
        }
    }
    else
    {
        int k = 0;
        while (std::abs(q_next->getCoord(k) - q_current->getCoord(k)) < 1e-6)
            k++;
            
        if (k < ss->getNumDimensions())
        {
            std::shared_ptr<base::State> q_new;
            float coord;
            for (int i = 0; i < num_lateral_states; i++)
            {
                q_new = ss->getRandomState(q_current);
                coord = q_current->getCoord(k) + q_new->getCoord(k) -
                        (q_next->getCoord() - q_current->getCoord()).dot(q_new->getCoord()) /
                        (q_next->getCoord(k) - q_current->getCoord(k));
                q_new->setCoord(coord, k);
                q_new = ss->interpolateEdge(q_current, q_new, RBTConnectConfig::DELTA);
                q_new = ss->pruneEdge(q_current, q_new);
                if (!ss->isEqual(q_current, q_new))
                {
                    horizon.emplace_back(std::make_shared<HorizonState>(q_new, -1));
                    num_added++;
                    // LOG(INFO) << "Adding lateral state: " << horizon.back()->getCoord().transpose();
                }
            }
        }
    }
    addRandomStates(num_lateral_states - num_added); 
}

// Modify state by replacing it with a random state, which is generated using biased distribution,
// i.e., oriented weight around 'q' ('q->getStatus()' == bad), or around '-q' ('q->getStatus()' == critical).
// Return whether the modification is successful
bool planning::drbt::DRGBTConnect::modifyState(std::shared_ptr<HorizonState> &q)
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
                // LOG(INFO) << "Modifying this state with the following state:\n" << q;
                return true;
            }
            // else
            //     LOG(INFO) << "Modifying this state is not successfull (" << num << ". attempt)!";
        }
    }   
    return false;
}

// Update the current state of the robot. If possible, move towards 'q_next' for step size DRGBTConnectConfig::STEP.
void planning::drbt::DRGBTConnect::updateCurrentState()
{
    // auto T = std::chrono::steady_clock::now();
    std::shared_ptr<base::State> q_new;
    tie(status, q_new) = ss->interpolateEdge2(q_current, q_next->getStateReached(), DRGBTConnectConfig::STEP);
    if (!ss->isEqual(q_current, q_new))
    {
        q_new->setParent(q_current);
        q_current = q_new;
        // LOG(INFO) << "Updating the robot current state to: " << q_current->getCoord().transpose();
    }
    else
    {
        status = base::State::Status::Trapped;
        replanning = true;
        horizon.clear();
        // LOG(INFO) << "Not updating the robot current state!";
    }
    path.emplace_back(q_current);

    // planner_info->addRoutineTime(getElapsedTime(T, std::chrono::steady_clock::now(), "microseconds"), 2);
}

// Compute reached state when generating a generalized spine from 'q_current' towards 'q'.
void planning::drbt::DRGBTConnect::computeReachedState(std::shared_ptr<base::State> q_current, std::shared_ptr<HorizonState> q)
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

// Compute the next state from the horizon
void planning::drbt::DRGBTConnect::computeNextState()
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

        // If state does not belong to the predefined path, 'weight' is decreased within the range [0, DRGBTConnectConfig::WEIGHT_MIN]
        // If such state becomes the best one, the replanning will surely be triggered
        if (horizon[i]->getIndex() == -1)   
            weight *= DRGBTConnectConfig::WEIGHT_MIN;
        
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

    // LOG(INFO) << "Setting the robot next state to: " << q_next->getCoord().transpose();
    q_next_previous = q_next;
}

// Return index in the horizon of state 'q'. If 'q' does not belong to the horizon, -1 is returned.
int planning::drbt::DRGBTConnect::getIndexInHorizon(std::shared_ptr<HorizonState> q)
{
    for (int idx = 0; idx < horizon.size(); idx++)
    {
        if (q == horizon[idx])
            return idx;
    }
    return -1;   
}

bool planning::drbt::DRGBTConnect::whetherToReplan()
{
    float weight_max = 0;
    float weight_sum = 0;
    for (int i = 0; i < horizon.size(); i++)
    {
        weight_max = std::max(weight_max, horizon[i]->getWeight());
        weight_sum += horizon[i]->getWeight();
    }
    return (weight_max <= DRGBTConnectConfig::WEIGHT_MIN && 
            weight_sum / horizon.size() <= DRGBTConnectConfig::WEIGHT_MEAN_MIN) 
            ? true : false;
}

void planning::drbt::DRGBTConnect::replan(float replanning_time)
{
    try
    {
        // LOG(INFO) << "Trying to replan in " << replanning_time << " [ms]...";
        std::unique_ptr<planning::AbstractPlanner> planner = initPlanner(replanning_time);
        
        if (planner->solve())   // New path is found, thus update predefined path to the goal
        {
            // LOG(INFO) << "The path has been replanned in " << planner->getPlannerInfo()->getPlanningTime() << " [ms]. ";
            predefined_path = planner->getPath();
            // LOG(INFO) << "Predefined path is: ";
            // for (int i = 0; i < predefined_path.size(); i++)
            //     std::cout << predefined_path.at(i) << std::endl;
            replanning = false;
            status = base::State::Status::Reached;
            q_next = std::make_shared<HorizonState>(predefined_path.front(), 0);
            horizon.clear();
            // planner_info->addRoutineTime(planner->getPlannerInfo()->getPlanningTime(), 4);
        }
        else    // New path is not found
            throw std::runtime_error("New path is not found! ");
    }
    catch (std::exception &e)
    {
        // LOG(INFO) << "Replanning is required. " << e.what();
        replanning = true;
    }
}

std::unique_ptr<planning::AbstractPlanner> planning::drbt::DRGBTConnect::initPlanner(float max_planning_time)
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

    std::cout << "The requested planner " << planner_name << " is not found! Using RGBTConnect. \n";
    RGBTConnectConfig::MAX_PLANNING_TIME = max_planning_time;
    return std::make_unique<planning::rbt::RGBTConnect>(ss, q_current, goal);
}

bool planning::drbt::DRGBTConnect::checkTerminatingCondition()
{
    auto time_current = getElapsedTime(time_start, std::chrono::steady_clock::now());    
    if (ss->isEqual(q_current, goal))
    {
        // LOG(INFO) << "Goal configuration has been successfully reached! ";
        std::cout << "Goal configuration has been successfully reached! \n";
		planner_info->setSuccessState(true);
        planner_info->setPlanningTime(time_current);
        return true;
    }
	
    if (time_current >= DRGBTConnectConfig::MAX_PLANNING_TIME)
	{
        // LOG(INFO) << "Maximal planning time has been reached! ";
        std::cout << "Maximal planning time has been reached! \n";
		planner_info->setSuccessState(false);
        planner_info->setPlanningTime(time_current);
		return true;
	}
    
    if (planner_info->getNumIterations() >= DRGBTConnectConfig::MAX_NUM_ITER)
	{
        // LOG(INFO) << "Maximal number of iterations has been reached! ";
        std::cout << "Maximal number of iterations has been reached! \n";
		planner_info->setSuccessState(false);
        planner_info->setPlanningTime(time_current);
		return true;
	}

	return false;
}

void planning::drbt::DRGBTConnect::outputPlannerData(std::string filename, bool output_states_and_paths, bool append_output) const
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
		output_file << "Planner type:    " << "DRGBTConnect" << std::endl;
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
