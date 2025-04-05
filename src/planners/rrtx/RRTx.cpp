#include "RRTx.h"

planning::rrtx::RRTx::RRTx(const std::shared_ptr<base::StateSpace> ss_) : RRTConnect(ss_) 
{
    planner_type = planning::PlannerType::RRTx;
}

planning::rrtx::RRTx::RRTx(const std::shared_ptr<base::StateSpace> ss_, const std::shared_ptr<base::State> q_start_,
                           const std::shared_ptr<base::State> q_goal_) : RRTConnect(ss_) 
{
    planner_type = planning::PlannerType::RRTx;
    q_start = q_start_;
    q_goal = q_goal_;

	if (!ss->isValid(q_start) || ss->robot->checkSelfCollision(q_start))
		throw std::domain_error("Start position is invalid!");
    
    // Single tree for RRTx (unlike RRTConnect which uses two trees)
    tree = std::make_shared<base::Tree>("rrtx_tree", 0);
    tree->setKdTree(std::make_shared<base::KdTree>(ss->num_dimensions, *tree, nanoflann::KDTreeSingleIndexAdaptorParams(10)));
    
    // Initialize the goal state
    tree->upgradeTree(q_goal, nullptr);
    
    // Add start as target
    start_state = q_start;

    // Set current, previous, and next state to start
    q_current = start_state;
    q_previous = q_current;
    q_next = q_current;
    
    // Initialize rewire sets and queues
    orphan_set.clear();
    rewire_set.clear();
    propagation_queue = std::priority_queue<std::shared_ptr<base::State>, std::vector<std::shared_ptr<base::State>>, CostComparator>();
    
    planner_info->setNumIterations(0);
    planner_info->setNumStates(1);

    updating_state = std::make_shared<planning::trajectory::UpdatingState>
        (ss, RRTxConfig::TRAJECTORY_INTERPOLATION, RRTxConfig::MAX_ITER_TIME);

    motion_validity = std::make_shared<planning::trajectory::MotionValidity>
        (ss, RRTxConfig::TRAJECTORY_INTERPOLATION, RRTxConfig::RESOLUTION_COLL_CHECK, &path, RRTxConfig::MAX_ITER_TIME);
    
    splines = nullptr;
    if (RRTxConfig::TRAJECTORY_INTERPOLATION == planning::TrajectoryInterpolation::Spline)
    {
        splines = std::make_shared<planning::trajectory::Splines>(ss, q_current, RRTxConfig::MAX_ITER_TIME);
        updating_state->setSplines(splines);
        motion_validity->setSplines(splines);
    }

}

planning::rrtx::RRTx::~RRTx()
{
    if (tree) {
        tree->clearTree();
    }
    
    orphan_set.clear();
    rewire_set.clear();
    path_current.clear();
}

float planning::rrtx::RRTx::distance(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) const
{
    return ss->getNorm(q1, q2);
}

bool planning::rrtx::RRTx::solve()
{
    time_alg_start = std::chrono::steady_clock::now();
    time_iter_start = time_alg_start;
    
    std::shared_ptr<base::State> q_rand = nullptr;
    std::shared_ptr<base::State> q_near = nullptr;
    std::shared_ptr<base::State> q_new = nullptr;
    base::State::Status status = base::State::Status::None;
    bool first_path_found = false;

    // Initially, radius for rewiring is set to a constant value
    r_rewire = RRTxConfig::R_REWIRE;
    
    // Phase 1: Find an initial path (similar to RRT)
    std::cout << "Finding an initial path... \n";
    while (!first_path_found)
    {
        // std::cout << "Iteration: " << planner_info->getNumIterations() << "\n";
        q_rand = ss->getRandomState();
        
        if (generateRandomNumber(0, 1) < RRTxConfig::START_BIAS) {
            q_rand = start_state;
        }
        
        q_near = tree->getNearestState(q_rand);
        
        std::tie(status, q_new) = extend(q_near, q_rand);
        
        if (status != base::State::Status::Trapped)
        {
            q_new->setCost(q_near->getCost() + distance(q_near, q_new));
            tree->upgradeTree(q_new, q_near);
            
            rewireNeighbors(q_new);
            
            if (distance(q_new, start_state) < r_rewire && 
                ss->isValid(q_new, start_state) && 
                !ss->robot->checkSelfCollision(q_new, start_state))
            {
                start_state->setParent(q_new);
                start_state->setCost(q_new->getCost() + distance(q_new, start_state));
                //tree->addState(start_state);
                tree->upgradeTree(start_state, q_new);
                first_path_found = true;
                computePath();
            }
        }
        
        planner_info->setNumIterations(planner_info->getNumIterations() + 1);
        planner_info->addIterationTime(getElapsedTime(time_alg_start));
        planner_info->setNumStates(tree->getNumStates());
        
        // Check if we've exceeded time or iterations
        if (checkTerminatingCondition(status)) {
            return planner_info->getSuccessState();
        }
    }
    
    // Phase 2: Continue improving the solution
    std::cout << "Dynamic planner is starting... \n";
    while (true)
    {
        // std::cout << "Iteration: " << planner_info->getNumIterations() << "\n";

        // Start the iteration clock
        time_iter_start = std::chrono::steady_clock::now();

        // Compute the shrinking ball radius
        r_rewire = shrinkingBallRadius(tree->getNumStates());

        // Sample a random state
        q_rand = ss->getRandomState();
        
        // Find nearest neighbor
        q_near = tree->getNearestState(q_rand);
        
        // Extend towards the random state
        std::tie(status, q_new) = extend(q_near, q_rand);
        
        if (status != base::State::Status::Trapped)
        {
            // Compute initial cost
            q_new->setCost(q_near->getCost() + distance(q_near, q_new));
            
            // Find neighbors within r_rewire
            std::vector<std::shared_ptr<base::State>> neighbors = findNeighbors(q_new, r_rewire);
            
            // Choose parent that minimizes cost
            chooseParent(q_new, neighbors);
            
            // Add to tree
            tree->upgradeTree(q_new, q_new->getParent());
            
            // Rewire the tree
            rewireNeighbors(q_new, neighbors);
            
            // Update the path if needed
            if (updatePath()) {
                computePath();
            }
        }

        // Updating current state
        q_next = start_state->getParent();
        // std::cout << "q_current: " << q_current << "\n";
        // std::cout << "q_next:    " << q_next << "\n";
        
        status = base::State::Status::None;
        std::shared_ptr<base::State> q_current_new = ss->getNewState(q_current->getCoord());
        updating_state->setTimeIterStart(time_iter_start);
        updating_state->update(q_previous, q_current_new, q_next, status);

        if (status == base::State::Status::Advanced)
        {
            // Update cost
            q_current_new->setCost(q_next->getCost() + distance(q_next, q_current_new));
            
            // Find neighbors within r_rewire
            std::vector<std::shared_ptr<base::State>> neighbors = findNeighbors(q_current_new, r_rewire);
            
            // If possible, choose parent that minimizes cost. Otherwise, remain the old parent.
            if (chooseParent(q_current_new, neighbors))
                tree->upgradeTree(q_current_new, q_current_new->getParent());
            else
                tree->upgradeTree(q_current_new, q_next);
            
            // Rewire the tree
            rewireNeighbors(q_current_new, neighbors);

            q_current = q_current_new;
        }
        else if (status == base::State::Status::Reached)
            q_current = q_next;

        // Change start to the current state
        start_state = q_current;

        // Update the path if needed
        if (updatePath()) {
            computePath();
        }
        // std::cout << "q_current_new: " << q_current << "\n";

        // Checking the real-time execution
        // float time_iter_remain = RRTxConfig::MAX_ITER_TIME * 1e3 - getElapsedTime(time_iter_start, planning::TimeUnit::ms);
        // std::cout << "Remaining iteration time is " << time_iter_remain << " [ms] \n";
        // if (time_iter_remain < 0)
        //     std::cout << "*************** Real-time is broken. " << -time_iter_remain << " [ms] exceeded!!! *************** \n";

        // Update environment and check if the collision occurs
        if (!motion_validity->check(q_previous, q_current))
        {
            std::cout << "*************** Collision has been occurred!!! *************** \n";
            planner_info->setSuccessState(false);
            planner_info->setPlanningTime(planner_info->getIterationTimes().back());
            return false;
        }

        // Process any invalidated nodes if obstacles have moved
        if (planner_info->getNumIterations() % RRTxConfig::REPLANNING_THROTTLE == 0) {
            handleDynamicObstacles();
        }
        
        // Planner info and terminating condition
        planner_info->setNumIterations(planner_info->getNumIterations() + 1);
        planner_info->addIterationTime(getElapsedTime(time_alg_start));
        planner_info->setNumStates(tree->getNumStates());
        if (checkTerminatingCondition(status)) {
            return planner_info->getSuccessState();
        }

        // std::cout << "----------------------------------------------------------------------------------------\n";
    }
    
    return false;
}

void planning::rrtx::RRTx::handleDynamicObstacles()
{
    if (orphan_set.empty()) {
        return;
    }
    
    // Propagate orphan status to children
    propagateOrphanStatus();
    
    // Update nearest neighbors for all nodes affected by obstacle changes
    updateNeighbors();
    
    // Rewire the tree to accommodate changes
    rewireTree();
    
    // Clean up orphaned nodes
    removeOrphanNodes();
    
    // Update the path
    computePath();
}

void planning::rrtx::RRTx::markAsOrphan(std::shared_ptr<base::State> node)
{
    if (node && orphan_set.find(node) == orphan_set.end()) {
        orphan_set.insert(node);
        node->setStatus(base::State::Status::Orphan);
    }
}

void planning::rrtx::RRTx::propagateOrphanStatus()
{
    std::queue<std::shared_ptr<base::State>> propagation_queue;
    
    // Initialize queue with orphaned nodes
    for (const auto &orphan : orphan_set) {
        propagation_queue.push(orphan);
    }
    
    while (!propagation_queue.empty())
    {
        auto current = propagation_queue.front();
        propagation_queue.pop();
        
        // Mark all children as orphans
        for (const auto &child : *current->getChildren())
        {
            if (orphan_set.find(child) == orphan_set.end()) {
                markAsOrphan(child);
                propagation_queue.push(child);
            }
        }
    }
}

void planning::rrtx::RRTx::updateNeighbors()
{
    // Create a set of all affected nodes (orphans + neighbors of orphans)
    std::unordered_set<std::shared_ptr<base::State>> affected_nodes = orphan_set;
    
    for (const auto &orphan : orphan_set)
    {
        // Find neighbors and add them to affected nodes
        std::vector<std::shared_ptr<base::State>> neighbors = findNeighbors(orphan, r_rewire);
        for (const auto &neighbor : neighbors) {
            affected_nodes.insert(neighbor);
        }
    }
    
    // Update neighbor connections for all affected nodes
    for (const auto &node : affected_nodes)
    {
        if (orphan_set.find(node) == orphan_set.end()) {
            node->clearNeighbourStates();
            std::vector<std::shared_ptr<base::State>> neighbors = findNeighbors(node, r_rewire);
            for (const auto &neighbor : neighbors) {
                if (orphan_set.find(neighbor) == orphan_set.end()) {
                    node->addNeighbourState(neighbor);
                }
            }
        }
    }
}

void planning::rrtx::RRTx::rewireTree()
{
    // Add all non-orphaned neighbors of orphans to the rewire set
    for (const auto &orphan : orphan_set)
    {
        std::vector<std::shared_ptr<base::State>> neighbors = findNeighbors(orphan, r_rewire);
        for (const auto &neighbor : neighbors) {
            if (orphan_set.find(neighbor) == orphan_set.end()) {
                rewire_set.insert(neighbor);
            }
        }
    }
    
    // Initialize propagation queue for rewiring
    for (const auto &node : rewire_set) {
        propagation_queue.push(node);
    }
    
    // Process the propagation queue
    while (!propagation_queue.empty())
    {
        auto current = propagation_queue.top();
        propagation_queue.pop();
        
        if (orphan_set.find(current) != orphan_set.end()) {
            continue;
        }
        
        // Look for a better parent
        std::vector<std::shared_ptr<base::State>> neighbors = findNeighbors(current, r_rewire);
        bool changed = chooseParent(current, neighbors);
        
        if (changed) {
            // Propagate changes to children
            for (const auto &child : *current->getChildren()) {
                child->setCost(current->getCost() + distance(current, child));
                propagation_queue.push(child);
            }
        }
    }
}

void planning::rrtx::RRTx::removeOrphanNodes()
{
    for (const auto &orphan : orphan_set) {
        tree->removeState(orphan);
    }
    
    orphan_set.clear();
    rewire_set.clear();
}

std::vector<std::shared_ptr<base::State>> planning::rrtx::RRTx::findNeighbors(
    const std::shared_ptr<base::State> q, float radius)
{
    std::vector<std::shared_ptr<base::State>> neighbors;
    const auto all_states = tree->getStates();
    
    // Find all states within radius
    for (const auto &state : *all_states)
    {
        if (state == q) continue;
        if (orphan_set.find(state) != orphan_set.end()) continue;
        
        if (distance(q, state) <= radius) {
            neighbors.push_back(state);
        }
    }
    
    // Limit number of neighbors to RRTxConfig::MAX_NEIGHBORS if necessary
    if (neighbors.size() > RRTxConfig::MAX_NEIGHBORS) {
        // Sort by distance
        std::sort(neighbors.begin(), neighbors.end(), 
                 [this, &q](const std::shared_ptr<base::State> &a, const std::shared_ptr<base::State> &b) {
                     return distance(q, a) < distance(q, b);
                 });
        neighbors.resize(RRTxConfig::MAX_NEIGHBORS);
    }
    
    return neighbors;
}

bool planning::rrtx::RRTx::chooseParent(
    std::shared_ptr<base::State> q_new, const std::vector<std::shared_ptr<base::State>> &neighbors)
{
    std::shared_ptr<base::State> min_parent = q_new->getParent();
    float min_cost = min_parent ? min_parent->getCost() + distance(min_parent, q_new) : std::numeric_limits<float>::infinity();
    
    for (const auto &neighbor : neighbors)
    {
        if (neighbor == q_new) continue;
        
        float potential_cost = neighbor->getCost() + distance(neighbor, q_new);
        
        if (potential_cost < min_cost && 
            ss->isValid(neighbor, q_new) && 
            !ss->robot->checkSelfCollision(neighbor, q_new))
        {
            min_cost = potential_cost;
            min_parent = neighbor;
        }
    }
    
    if (min_parent && min_parent != q_new->getParent())
    {
        // Remove from old parent's children list if exists
        if (q_new->getParent()) {
            q_new->getParent()->removeChild(q_new);
        }
        
        // Update parent and cost
        q_new->setParent(min_parent);
        q_new->setCost(min_cost);
        min_parent->addChild(q_new);
        
        return true;
    }
    
    return false;
}

void planning::rrtx::RRTx::rewireNeighbors(
    std::shared_ptr<base::State> q_new, const std::vector<std::shared_ptr<base::State>> &neighbors_arg)
{
    // Use provided neighbors or find them if not provided
    std::vector<std::shared_ptr<base::State>> neighbors = 
        neighbors_arg.empty() ? findNeighbors(q_new, r_rewire) : neighbors_arg;
    
    for (auto &neighbor : neighbors)
    {
        if (neighbor == q_new || neighbor == q_new->getParent()) continue;
        
        float potential_cost = q_new->getCost() + distance(q_new, neighbor);
        
        if (potential_cost < neighbor->getCost() && 
            ss->isValid(q_new, neighbor) && 
            !ss->robot->checkSelfCollision(q_new, neighbor))
        {
            // Remove from old parent's children list
            if (neighbor->getParent()) {
                neighbor->getParent()->removeChild(neighbor);
            }
            
            // Update parent and cost
            neighbor->setParent(q_new);
            neighbor->setCost(potential_cost);
            q_new->addChild(neighbor);
            
            // Propagate cost updates to children
            propagateCostChanges(neighbor);
        }
    }
}

void planning::rrtx::RRTx::propagateCostChanges(std::shared_ptr<base::State> node)
{
    std::queue<std::shared_ptr<base::State>> cost_queue;
    cost_queue.push(node);
    
    while (!cost_queue.empty())
    {
        auto current = cost_queue.front();
        cost_queue.pop();
        
        for (const auto &child : *current->getChildren())
        {
            float new_cost = current->getCost() + distance(current, child);
            if (new_cost < child->getCost()) {
                child->setCost(new_cost);
                cost_queue.push(child);
            }
        }
    }
}

bool planning::rrtx::RRTx::updatePath()
{
    if (path_current.empty()) {
        return false;
    }
    
    // Check if the start node's parent is valid
    if (!start_state->getParent() || orphan_set.find(start_state->getParent()) != orphan_set.end()) {
        return true;  // Need to recompute path
    }
    
    // Traverse the path from goal to start to check validity
    std::shared_ptr<base::State> current = start_state;
    while (current->getParent() != nullptr)
    {
        std::shared_ptr<base::State> parent = current->getParent();
        if (!ss->isValid(current, parent) || 
            ss->robot->checkSelfCollision(current, parent))
        {
            return true;  // Path is invalid, need to recompute
        }
        current = current->getParent();
    }
    
    return false;  // Path is still valid
}

void planning::rrtx::RRTx::computePath()
{
    path_current.clear();
    
    if (!start_state->getParent()) {
        // No path to start
        return;
    }
    
    // Begin from start and trace forward to goal
    std::shared_ptr<base::State> current = start_state;
    while (current != nullptr)
    {
        path_current.push_back(current);
        current = current->getParent();
    }
}

base::Tree planning::rrtx::RRTx::getTree() const
{
    return *tree;
}

bool planning::rrtx::RRTx::checkTerminatingCondition([[maybe_unused]] base::State::Status status)
{
    float time_current = getElapsedTime(time_alg_start);
    
    if (ss->isEqual(q_current, q_goal))
    {
        std::cout << "Goal configuration has been successfully reached! \n";
		planner_info->setSuccessState(true);
        planner_info->setPlanningTime(time_current);
        return true;
    }
	
    if (time_current >= RRTxConfig::MAX_PLANNING_TIME)
	{
        std::cout << "Maximal planning time has been reached! \n";
		planner_info->setSuccessState(false);
        planner_info->setPlanningTime(time_current);
		return true;
	}
    
    if (planner_info->getNumIterations() >= RRTxConfig::MAX_NUM_ITER)
	{
        std::cout << "Maximal number of iterations has been reached! \n";
		planner_info->setSuccessState(false);
        planner_info->setPlanningTime(time_current);
		return true;
	}

	return false;
}

float planning::rrtx::RRTx::generateRandomNumber(float min, float max)
{
    std::uniform_real_distribution<float> distribution(min, max);
    static std::random_device rd;
    static std::mt19937 generator(rd());
    return distribution(generator);
}

float planning::rrtx::RRTx::shrinkingBallRadius(size_t num_states) 
{
    float eta = std::sqrt(ss->num_dimensions);
    float mi = std::pow(2 * M_PI, ss->num_dimensions);
    float zeta = std::pow(M_PI, ss->num_dimensions / 2.0f) / std::tgamma(ss->num_dimensions / 2.0f + 1);
    float gamma_rrt = 2 * std::pow((1 + 1.0 / ss->num_dimensions) * (mi / zeta), 1.0f / ss->num_dimensions);
    return std::min(gamma_rrt * float(std::pow(std::log(num_states) / num_states, 1.0f / ss->num_dimensions)), eta);
}

void planning::rrtx::RRTx::outputPlannerData(const std::string &filename, bool output_states_and_paths, bool append_output) const
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
        output_file << "\t Successful:            " << (planner_info->getSuccessState() ? "yes" : "no") << std::endl;
        output_file << "\t Number of iterations:  " << planner_info->getNumIterations() << std::endl;
        output_file << "\t Number of states:      " << planner_info->getNumStates() << std::endl;
        output_file << "\t Planning time [s]:     " << planner_info->getPlanningTime() << std::endl;
        output_file << "\t Rewire radius:         " << r_rewire << std::endl;
        output_file << "\t Collision radius:      " << RRTxConfig::R_COLLISION << std::endl;
        
        if (output_states_and_paths)
        {
            output_file << *tree;
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
        throw std::runtime_error("Cannot open file");
}