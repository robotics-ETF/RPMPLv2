#include "RRTx.h"

planning::rrtx::RRTx::RRTx(const std::shared_ptr<base::StateSpace> ss_) : AbstractPlanner(ss_) 
{
    planner_type = planning::PlannerType::RRTx;
    initializeParameters();
}

planning::rrtx::RRTx::RRTx(const std::shared_ptr<base::StateSpace> ss_, const std::shared_ptr<base::State> q_start_,
                           const std::shared_ptr<base::State> q_goal_) : AbstractPlanner(ss_, q_start_, q_goal_)
{
    planner_type = planning::PlannerType::RRTx;
    initializeParameters();
    
    if (!ss->isValid(q_start) || ss->robot->checkSelfCollision(q_start))
        throw std::domain_error("Start position is invalid!");
    if (!ss->isValid(q_goal) || ss->robot->checkSelfCollision(q_goal))
        throw std::domain_error("Goal position is invalid!");
        
    // Single tree for RRTx (unlike RRTConnect which uses two trees)
    tree = std::make_shared<base::Tree>("rrtx_tree", 0);
    tree->setKdTree(std::make_shared<base::KdTree>(ss->num_dimensions, *tree, nanoflann::KDTreeSingleIndexAdaptorParams(10)));
    
    // Initialize the goal state
    tree->upgradeTree(q_goal, nullptr);
    
    // Add start as target
    start_state = q_start;

    // Set current, previous, and next state to start
    q_current = q_start;
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
    path.clear();
}

void planning::rrtx::RRTx::initializeParameters()
{
    // RRTx specific parameters
    r_rewire = RRTxConfig::R_REWIRE;
    r_collision = RRTxConfig::R_COLLISION;
    r_nearest = RRTxConfig::R_NEAREST;
    eps_step = RRTxConfig::EPS_STEP;
    max_neighbors = RRTxConfig::MAX_NEIGHBORS;
    replanning_throttle = RRTxConfig::REPLANNING_THROTTLE;
    rewire_factor = RRTxConfig::REWIRE_FACTOR;
}

double planning::rrtx::RRTx::distance(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) const
{
    return static_cast<double>(ss->getNorm(q1, q2));
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
    planner_info->setSuccessState(true);
    std::cout << "Dynamic planner is starting... \n";
    while (true)
    {
        std::cout << "Iteration: " << planner_info->getNumIterations() << "\n";

        // Start the iteration clock
        time_iter_start = std::chrono::steady_clock::now();

        // Change start to the current state
        q_start = q_current;

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
            double cost = q_near->getCost() + distance(q_near, q_new);
            q_new->setCost(cost);
            
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

            // Updating current state
            markAsOrphan(q_start);
            q_next = q_start->getParent();
            std::cout << "q_current: " << q_current << "\n";
            std::cout << "q_next:    " << q_next << "\n";
            
            updating_state->setTimeIterStart(time_iter_start);
            updating_state->update(q_previous, q_current, q_next, status);
            std::cout << "q_current_new: " << q_current << "\n";
        }

        // Checking the real-time execution
        float time_iter_remain = RRTxConfig::MAX_ITER_TIME * 1e3 - getElapsedTime(time_iter_start, planning::TimeUnit::ms);
        std::cout << "Remaining iteration time is " << time_iter_remain << " [ms] \n";
        if (time_iter_remain < 0)
            std::cout << "*************** Real-time is broken. " << -time_iter_remain << " [ms] exceeded!!! *************** \n";

        // Update environment and check if the collision occurs
        if (!motion_validity->check(q_previous, q_current))
        {
            std::cout << "*************** Collision has been occurred!!! *************** \n";
            planner_info->setSuccessState(false);
            planner_info->setPlanningTime(planner_info->getIterationTimes().back());
            return false;
        }
        
        // Process any invalidated nodes if obstacles have moved
        if (planner_info->getNumIterations() % replanning_throttle == 0) {
            handleDynamicObstacles();
        }
        
        // Planner info and terminating condition
        planner_info->setNumIterations(planner_info->getNumIterations() + 1);
        planner_info->addIterationTime(getElapsedTime(time_alg_start));
        planner_info->setNumStates(tree->getNumStates());
        if (checkTerminatingCondition(status)) {
            return planner_info->getSuccessState();
        }

        std::cout << "----------------------------------------------------------------------------------------\n";
    }
    
    return false;
}

bool planning::rrtx::RRTx::updateObstacles(const std::vector<std::tuple<std::shared_ptr<base::State>, std::shared_ptr<base::State>>> &changed_regions)
{
    // Mark affected nodes
    for (const auto &region : changed_regions)
    {
        std::shared_ptr<base::State> q_min = std::get<0>(region);
        std::shared_ptr<base::State> q_max = std::get<1>(region);
        
        // Find all states in the tree
        const auto all_states = tree->getStates();
        
        // Check which states are in the affected region
        for (auto &state : *all_states)
        {
            // Check if the state is within the bounding box defined by q_min and q_max
            bool in_region = true;
            for (size_t i = 0; i < ss->num_dimensions; ++i)
            {
                if (state->getCoord()(i) < q_min->getCoord()(i) || 
                    state->getCoord()(i) > q_max->getCoord()(i)) {
                    in_region = false;
                    break;
                }
            }
            
            if (in_region)
            {
                // Check if the node is now in collision
                if (!ss->isValid(state) || ss->robot->checkSelfCollision(state))
                {
                    markAsOrphan(state);
                }
                else
                {
                    // Check if the edges to/from this node are now in collision
                    std::shared_ptr<base::State> parent = state->getParent();
                    if (parent && (!ss->isValid(parent, state) || ss->robot->checkSelfCollision(parent, state)))
                    {
                        markAsOrphan(state);
                    }
                    
                    // Check children's edges
                    for (auto &child : *state->getChildren())
                    {
                        if (!ss->isValid(state, child) || ss->robot->checkSelfCollision(state, child))
                        {
                            markAsOrphan(child);
                        }
                    }
                }
            }
        }
    }
    
    // If there are orphans, we need to replan
    return !orphan_set.empty();
}

bool planning::rrtx::RRTx::updateObstacles(const std::shared_ptr<base::State> region_min, 
                                        const std::shared_ptr<base::State> region_max)
{
    std::vector<std::tuple<std::shared_ptr<base::State>, std::shared_ptr<base::State>>> regions;
    regions.push_back(std::make_tuple(region_min, region_max));
    return updateObstacles(regions);
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

std::tuple<base::State::Status, std::shared_ptr<base::State>> planning::rrtx::RRTx::extend(
    const std::shared_ptr<base::State> q, const std::shared_ptr<base::State> q_e)
{
    base::State::Status status { base::State::Status::None };
    std::shared_ptr<base::State> q_new { nullptr };
    std::tie(status, q_new) = ss->interpolateEdge2(q, q_e, static_cast<float>(eps_step));

    if (ss->isValid(q, q_new) && !ss->robot->checkSelfCollision(q, q_new))
        return {status, q_new};
    else
        return {base::State::Status::Trapped, q};
}

std::vector<std::shared_ptr<base::State>> planning::rrtx::RRTx::findNeighbors(
    const std::shared_ptr<base::State> q, double radius)
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
    
    // Limit number of neighbors to max_neighbors if necessary
    if (neighbors.size() > max_neighbors) {
        // Sort by distance
        std::sort(neighbors.begin(), neighbors.end(), 
                 [this, &q](const std::shared_ptr<base::State> &a, const std::shared_ptr<base::State> &b) {
                     return distance(q, a) < distance(q, b);
                 });
        neighbors.resize(max_neighbors);
    }
    
    return neighbors;
}

bool planning::rrtx::RRTx::chooseParent(
    std::shared_ptr<base::State> q_new, const std::vector<std::shared_ptr<base::State>> &neighbors)
{
    std::shared_ptr<base::State> min_parent = q_new->getParent();
    double min_cost = min_parent ? min_parent->getCost() + distance(min_parent, q_new) : std::numeric_limits<double>::infinity();
    
    for (const auto &neighbor : neighbors)
    {
        if (neighbor == q_new) continue;
        
        double potential_cost = neighbor->getCost() + distance(neighbor, q_new);
        
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
        
        double potential_cost = q_new->getCost() + distance(q_new, neighbor);
        
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
            double new_cost = current->getCost() + distance(current, child);
            if (new_cost < child->getCost()) {
                child->setCost(new_cost);
                cost_queue.push(child);
            }
        }
    }
}

bool planning::rrtx::RRTx::updatePath()
{
    if (!planner_info->getSuccessState()) {
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
    path.clear();
    
    if (!start_state->getParent()) {
        // No path to start
        planner_info->setSuccessState(false);
        return;
    }
    
    // Begin from start and trace forward to goal
    std::shared_ptr<base::State> current = start_state;
    while (current != nullptr)
    {
        path.push_back(current);
        current = current->getParent();
    }
    
    planner_info->setSuccessState(true);
}

const std::vector<std::shared_ptr<base::State>> &planning::rrtx::RRTx::getPath() const
{
    return path;
}

base::Tree planning::rrtx::RRTx::getTree() const
{
    return *tree;
}

bool planning::rrtx::RRTx::checkTerminatingCondition([[maybe_unused]] base::State::Status status)
{
    float time_current = getElapsedTime(time_alg_start);
    
    if (time_current >= RRTxConfig::MAX_PLANNING_TIME ||
        planner_info->getNumStates() >= RRTxConfig::MAX_NUM_STATES || 
        planner_info->getNumIterations() >= RRTxConfig::MAX_NUM_ITER)
    {
        planner_info->setPlanningTime(time_current);
        return true;
    }
    
    return false;
}

double planning::rrtx::RRTx::generateRandomNumber(double min, double max)
{
    std::uniform_real_distribution<double> distribution(min, max);
    static std::random_device rd;
    static std::mt19937 generator(rd());
    return distribution(generator);
}

double planning::rrtx::RRTx::shrinkingBallRadius(size_t num_states) 
{
    double eta = std::sqrt(ss->num_dimensions);
    double mi = std::pow(2 * M_PI, ss->num_dimensions);
    double zeta = std::pow(M_PI, ss->num_dimensions / 2.0f) / std::tgamma(ss->num_dimensions / 2.0f + 1);
    double gamma_rrt = 2 * std::pow((1 + 1.0 / ss->num_dimensions) * (mi / zeta), 1.0f / ss->num_dimensions);
    return std::min(gamma_rrt * double(std::pow(std::log(num_states) / num_states, 1.0f / ss->num_dimensions)), eta);
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
        output_file << "\t Collision radius:      " << r_collision << std::endl;
        
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