#include "RGBMTStar.h"

planning::rbt_star::RGBMTStar::RGBMTStar(const std::shared_ptr<base::StateSpace> ss_) : RGBTConnect(ss_) 
{
    planner_type = planning::PlannerType::RGBMTStar;
}

planning::rbt_star::RGBMTStar::RGBMTStar(const std::shared_ptr<base::StateSpace> ss_, const std::shared_ptr<base::State> q_start_,
                                         const std::shared_ptr<base::State> q_goal_) : RGBTConnect(ss_, q_start_, q_goal_)
{
    // Additionally the following is required:
    planner_type = planning::PlannerType::RGBMTStar;
	q_start->setCost(0);
    q_goal->setCost(0);
    num_states = {1, 1};
    cost_opt = INFINITY;
    q_con_opt = nullptr;
    planner_info->addCostConvergence({INFINITY, INFINITY});
    planner_info->addStateTimes({0, 0});
}

bool planning::rbt_star::RGBMTStar::solve()
{
	time_alg_start = std::chrono::steady_clock::now();  // Start the clock
	size_t tree_idx { 0 };                              // Determines a tree index, i.e., which tree is chosen, 0: from q_init; 1: from q_goal; >1: local trees
    size_t tree_new_idx { 2 };                          // Index of a new tree
    std::shared_ptr<base::State> q_rand { nullptr };
    std::shared_ptr<base::State> q_near { nullptr };
    std::shared_ptr<base::State> q_new { nullptr };
	base::State::Status status { base::State::Status::None };
    std::vector<size_t> trees_exist {};                            // List of trees for which a new tree is extended to
    std::vector<size_t> trees_reached {};                          // List of reached trees
    std::vector<size_t> trees_connected {};                        // List of connected trees
    std::vector<std::shared_ptr<base::State>> states_reached {};   // Reached states from other trees

    while (true)
    {
		// std::cout << "Iteration: " << planner_info->getNumIterations() << "\n";
		// std::cout << "Num. states: " << planner_info->getNumStates() << "\n";
        // std::cout << "Num. main: " << num_states[0] + num_states[1] << "\t "
        //           << "Num. local: " << planner_info->getNumStates() - num_states[0] - num_states[1] << "\n";
        
		q_rand = getRandomState();
        q_rand->setCost(0);
        
        // Adding a new local tree rooted in 'q_rand'
        trees.emplace_back(std::make_shared<base::Tree>(base::Tree("local", tree_new_idx)));
        trees[tree_new_idx]->setKdTree(std::make_shared<base::KdTree>(ss->num_dimensions, *trees[tree_new_idx], 
                                                                      nanoflann::KDTreeSingleIndexAdaptorParams(10)));
        trees[tree_new_idx]->upgradeTree(q_rand, nullptr);
        trees_exist.clear();
        trees_reached.clear();
        trees_connected.clear();
        states_reached.clear();
        states_reached = std::vector<std::shared_ptr<base::State>>(tree_new_idx, nullptr);

        // Considering all previous trees
        for (size_t idx = 0; idx < tree_new_idx; idx++)
        {
            // If the connection with 'q_near' is not possible, attempt to connect with 'parent(q_near)', etc.
            q_near = trees[idx]->getNearestState(q_rand);
            // q_near = trees[idx]->getNearestState2(q_rand);
            std::shared_ptr<base::State> q_near_new { q_near };
            while (true)
            {
                tie(status, q_new) = connectGenSpine(q_rand, q_near_new);
                if (status == base::State::Status::Reached || q_near_new->getParent() == nullptr)
                    break;
                else
                    q_near_new = q_near_new->getParent();
            }
            if (status != base::State::Status::Reached)
                q_near_new = q_near;

            // Whether currently considering tree ('tree_new_idx'-th tree) is reached
            if (status == base::State::Status::Reached)
            {
                // If 'idx-th' tree is reached
                q_new->setCost(computeCostToCome(q_rand, q_new));
                trees[tree_new_idx]->upgradeTree(q_new, q_rand, q_near_new);
                trees_exist.emplace_back(idx);
                trees_reached.emplace_back(idx);
                states_reached[idx] = q_near_new;
            }
            else if (status == base::State::Status::Advanced)
            {
                q_new->setCost(computeCostToCome(q_rand, q_new));
                trees[tree_new_idx]->upgradeTree(q_new, q_rand);
                trees_exist.emplace_back(idx);
            }

            if (checkTerminatingCondition(status))
                return planner_info->getSuccessState();
        }

        // Find the optimal edge towards each reached tree
        if (!trees_reached.empty())
        {
            // The connection of 'q_rand' with both main trees exists
            tree_idx = trees_reached[0];      // Considering the first reached tree
            bool main_trees_reached { (trees_reached.size() > 1 && trees_reached[0] == 0 && trees_reached[1] == 1) ? true : false };
            if (main_trees_reached)
            {
                std::random_device rd;
                std::mt19937 generator(rd());
                std::uniform_real_distribution<float> distribution(0.0, 1.0);
                if (distribution(generator) > (float) num_states[1] / (num_states[0] + num_states[1]))
                    tree_idx = trees_reached[1];     // 'q_rand' will be joined to the second main tree
            }

            // Considering all edges from the new tree
            q_rand = optimize(q_rand, trees[tree_idx], states_reached[tree_idx]);
            if (checkTerminatingCondition(status))
                return planner_info->getSuccessState();

            size_t k { 0 };     // Index of a state from the new tree 'tree_new_idx'
            for (size_t idx : trees_exist)
            {
                k += 1;
                if (idx == tree_idx)  // It was considered previously, so just skip now
                    continue;

                q_new = optimize(trees[tree_new_idx]->getState(k), trees[tree_idx], q_rand);   // From 'k'-th reached state to tree 'tree_idx'
                if (checkTerminatingCondition(status))
                    return planner_info->getSuccessState();

                // The connection of 'q_rand' with both main trees exists
                if (idx < 2 && main_trees_reached)
                {
                    std::shared_ptr<base::State> q_parent { states_reached[idx]->getParent() };
                    while (q_parent != nullptr)
                    {
                        q_new = optimize(q_parent, trees[tree_idx], q_new);
                        q_parent = q_parent->getParent();
                    }
                    
                    if (q_new->getCost() < cost_opt)    // The optimal connection between main trees is stored
                    {
                        q_con_opt = q_new;
                        cost_opt = q_new->getCost();
                        // std::cout << "Cost after " << planner_info->getNumStates() << " states is " << cost_opt << "\n";
                        // planner_info->setSuccessState(true);
                        // computePath(q_con_opt);
                        // outputPlannerData("/home/nermin/RPMPLv2/data/planar_2dof/scenario1_tests/plannerData" + 
                        //                   std::to_string(planner_info->getNumStates()) + ".log");
                    }
                }
                // Unification of tree 'idx' with 'tree_idx'. Main trees are never unified mutually
                else if (idx > 1 && std::find(trees_reached.begin(), trees_reached.end(), idx) != trees_reached.end())
                {
                    unifyTrees(trees[tree_idx], states_reached[idx], q_new);
                    trees_connected.emplace_back(idx);
                }
            }

            // Deleting trees that have been connected
            trees_connected.emplace_back(tree_new_idx);
            deleteTrees(trees_connected);
            tree_new_idx -= trees_connected.size() - 1;
        }
        else    // If there are no reached trees, then the new tree is added to 'trees'
            tree_new_idx += 1;

		/* Planner info and terminating condition */
        planner_info->setNumIterations(planner_info->getNumIterations() + 1);
		planner_info->addIterationTime(getElapsedTime(time_alg_start));
		size_t num_states_total { 0 };
        num_states.resize(trees.size());
        for(size_t idx = 0; idx < trees.size(); idx++)
        {
            num_states[idx] = trees[idx]->getNumStates();
		    num_states_total += num_states[idx];
        }
        planner_info->addCostConvergence(std::vector<float>(num_states_total - planner_info->getNumStates(), cost_opt));
        planner_info->addStateTimes(std::vector<float>(num_states_total - planner_info->getNumStates(), planner_info->getIterationTimes().back()));
        planner_info->setNumStates(num_states_total);
        if (checkTerminatingCondition(status))
            return planner_info->getSuccessState();
    }
}

std::shared_ptr<base::State> planning::rbt_star::RGBMTStar::getRandomState()
{
    size_t tree_idx { 0 };
    std::shared_ptr<base::State> q_rand { nullptr };
    std::shared_ptr<base::State> q_near { nullptr };
    base::State::Status status { base::State::Status::None };

    while (true)
    {
        q_rand = ss->getRandomState();    // Uniform distribution
        if (planner_info->getNumStates() > 2 * (num_states[0] + num_states[1]))     // If local trees contain more states than main trees
        {
            // std::cout << "Local trees are dominant! \n";
            tree_idx = (num_states[0] < num_states[1]) ? 0 : 1;
            q_near = trees[tree_idx]->getNearestState(q_rand);
            // q_near = trees[tree_idx]->getNearestState2(q_rand);
            tie(status, q_rand) = connectGenSpine(q_near, q_rand);
            if (status != base::State::Status::Trapped)
                return q_rand;
        }
        else if (ss->isValid(q_rand) && !ss->robot->checkSelfCollision(q_rand))    // If 'q_rand' is collision-free, it is accepted
            return q_rand;
    }
    return nullptr;
}

// Connect state 'q' with state 'q_e'
// Return 'Status'
// Return 'q_new': the reached state
std::tuple<base::State::Status, std::shared_ptr<base::State>> planning::rbt_star::RGBMTStar::connectGenSpine
    (const std::shared_ptr<base::State> q, const std::shared_ptr<base::State> q_e)
{
	float d_c { ss->computeDistance(q) };
	std::shared_ptr<base::State> q_new { q };
    std::shared_ptr<base::State> q_temp { nullptr };
	base::State::Status status { base::State::Status::Advanced };
	size_t num_ext { 0 };

	while (status == base::State::Status::Advanced)
	{
		q_temp = ss->getNewState(q_new);
		if (d_c > RBTConnectConfig::D_CRIT)
		{
			tie(status, q_new) = extendGenSpine(q_temp, q_e);
            d_c = ss->computeDistance(q_new);
		}
		else
		{
			tie(status, q_new) = extend(q_temp, q_e);
            if (++num_ext > 0.2 * RRTConnectConfig::MAX_EXTENSION_STEPS)
            {
                d_c = ss->computeDistance(q_new);   
                num_ext = 0;
            }
		}            
	}
	return {status, q_new};
}

// Return (weighted) cost-to-come from 'q1' to 'q2'
inline float planning::rbt_star::RGBMTStar::computeCostToCome(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2)
{
    // float d_c { ss->computeDistance(q2) };
    // if (d_c < 10 * DRGBTConfig::D_CRIT)
    //     return (10 * DRGBTConfig::D_CRIT / d_c) * ss->getNorm(q1, q2);

    return ss->getNorm(q1, q2);
}

// State 'q' from another tree is optimally connected to 'tree'
// 'q_reached' is a state from 'tree' that is reached by 'q'
// Return 'q_new': state 'q' which now belongs to 'tree'
std::shared_ptr<base::State> planning::rbt_star::RGBMTStar::optimize
    (const std::shared_ptr<base::State> q, const std::shared_ptr<base::Tree> tree, std::shared_ptr<base::State> q_reached)
{
    // Finding the optimal connection to the predecessors of 'q_reached'
    std::shared_ptr<base::State> q_parent { q_reached->getParent() };
    while (q_parent != nullptr)
    {
        if (std::get<0>(connectGenSpine(q, q_parent)) == base::State::Status::Reached)
            q_reached = q_parent;
        q_parent = q_parent->getParent();
    }

    std::shared_ptr<base::State> q_opt { nullptr }; 
    if (q_reached->getParent() != nullptr)
    {
        Eigen::VectorXf q_opt_ { q_reached->getCoord() };  // It is surely collision-free. It will become an optimal state later
        Eigen::VectorXf q_parent_ { q_reached->getParent()->getCoord() };   // Needs to be collision-checked
        std::shared_ptr<base::State> q_middle { ss->getRandomState() };
        size_t max_iter = std::ceil(std::log2(ss->getNorm(q_reached->getParent(), q_reached) / RRTConnectConfig::EPS_STEP));
        bool update { false };

        for (size_t i = 0; i < max_iter; i++)
        {
            q_middle->setCoord((q_opt_ + q_parent_) / 2);
            if (std::get<0>(connectGenSpine(q, q_middle)) == base::State::Status::Reached)
            {
                q_opt_ = q_middle->getCoord();
                update = true;
            }
            else
                q_parent_ = q_middle->getCoord();
        }

        if (update)
        {
            q_opt = ss->getNewState(q_opt_);
            q_opt->setCost(q_reached->getParent()->getCost() + computeCostToCome(q_reached->getParent(), q_opt));
            q_opt->addChild(q_reached);
            tree->upgradeTree(q_opt, q_reached->getParent());
            std::shared_ptr<std::vector<std::shared_ptr<base::State>>> children { q_reached->getParent()->getChildren() };

            for (size_t i = 0; i < children->size(); i++)
            {
                if (ss->isEqual(children->at(i), q_reached))
                {
                    children->erase(children->begin() + i);     // Deleting the child, since it is previously added in 'upgradeTree' when adding 'q_opt'
                    break;
                }
            }
            q_reached->setParent(q_opt);
        }
        else
            q_opt = q_reached;
    }
    else
        q_opt = q_reached;

    std::shared_ptr<base::State> q_new { ss->getNewState(q->getCoord()) };
    q_new->setCost(q_opt->getCost() + computeCostToCome(q_opt, q_new));
    tree->upgradeTree(q_new, q_opt, q);
    return q_new;
}

// Optimally unifies a local tree 'tree[idx]' with 'tree0'
// 'q_con' - a state that connects 'tree[idx]' with 'tree0'
// 'q0_con' - a state that connects 'tree0' with 'tree[idx]'
void planning::rbt_star::RGBMTStar::unifyTrees(const std::shared_ptr<base::Tree> tree0, const std::shared_ptr<base::State> q_con, 
                                               const std::shared_ptr<base::State> q0_con)
{
    std::shared_ptr<base::State> q_considered { nullptr };
    std::shared_ptr<base::State> q_con_new { ss->getNewState(q_con) };
    std::shared_ptr<base::State> q0_con_new { ss->getNewState(q0_con) };

    while (true)
    {
        considerChildren(q_con_new, tree0, q0_con_new, q_considered);
        if (q_con_new->getParent() == nullptr)
            break;

        q0_con_new = optimize(q_con_new->getParent(), tree0, q0_con_new);
        q_considered = q_con_new;
        q_con_new = q_con_new->getParent();
    }
}

// Consider all children (except 'q_considered', that has already being considered) of 'q', and connect them optimally to 'tree0' 
void planning::rbt_star::RGBMTStar::considerChildren(const std::shared_ptr<base::State> q, const std::shared_ptr<base::Tree> tree0,
                                                     const std::shared_ptr<base::State> q0_con, const std::shared_ptr<base::State> q_considered)
{
    // Remove child 'q_considered' of 'q' from 'tree'
    std::shared_ptr<std::vector<std::shared_ptr<base::State>>> children { q->getChildren() };
    if (q_considered != nullptr)
    {
        for (size_t i = 0; i < children->size(); i++)
        {
            if (ss->isEqual(children->at(i), q_considered))
            {
                children->erase(children->begin() + i);
                break;
            }
        }
    }

    std::shared_ptr<base::State> q0_con_new { nullptr };
    for (size_t i = 0; i < children->size(); i++)
    {
        q0_con_new = optimize(children->at(i), tree0, q0_con);
        if (children->at(i)->getChildren()->size() > 0)   // child has its own children
            considerChildren(children->at(i), tree0, q0_con_new, nullptr);  // 'nullptr' means that no children will be removed
    }
}

// Delete all trees with indices 'idx'
void planning::rbt_star::RGBMTStar::deleteTrees(const std::vector<size_t> &trees_connected)
{
    for (int i = trees_connected.size()-1; i >= 0; i--)
        trees.erase(trees.begin() + trees_connected[i]);
}

void planning::rbt_star::RGBMTStar::computePath(std::shared_ptr<base::State> q_con)
{
	path.clear();
    path.emplace_back(q_con);
    while (q_con->getParent() != nullptr)
    {
        path.emplace_back(q_con->getParent());
        q_con = q_con->getParent();
    }
    if (q_con->getTreeIdx() == 0)
        std::reverse(path.begin(), path.end());
}

bool planning::rbt_star::RGBMTStar::checkTerminatingCondition([[maybe_unused]] base::State::Status status)
{
    if (getElapsedTime(time_alg_start) >= RGBMTStarConfig::MAX_PLANNING_TIME ||
        planner_info->getNumStates() >= RGBMTStarConfig::MAX_NUM_STATES ||
        planner_info->getNumIterations() >= RGBMTStarConfig::MAX_NUM_ITER ||
        (RGBMTStarConfig::TERMINATE_WHEN_PATH_IS_FOUND && cost_opt < INFINITY))
    {
        if (cost_opt < INFINITY)
        {
		    computePath(q_con_opt);
		    planner_info->setSuccessState(true);
        }
        else
		    planner_info->setSuccessState(false);
        
        planner_info->setOptimalCost(cost_opt);
        planner_info->setPlanningTime(getElapsedTime(time_alg_start));
        return true;
    }
    return false;
}

void planning::rbt_star::RGBMTStar::outputPlannerData(const std::string &filename, bool output_states_and_paths, bool append_output) const
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
		output_file << "\t Number of states:     " << planner_info->getNumStates() << std::endl;
		output_file << "\t Planning time [s]:    " << planner_info->getPlanningTime() << std::endl;
		output_file << "\t Path cost [rad]:      " << planner_info->getOptimalCost() << std::endl;
		if (output_states_and_paths)
		{
            // Just to check how many states have real or underestimation of distance-to-obstacles computed
            // size_t num_real { 0 };
			// size_t num_underest { 0 };
			// size_t num_total { 0 };
            // for (size_t i = 0; i < trees.size(); i++)
            // {
            //     output_file << *trees[i];
            //     for (auto q : *trees[i]->getStates())
            //     {
            //         if (q->getDistance() > 0)
            //         {
            //             if (q->getIsRealDistance()) num_real++;
            //             else num_underest++;
            //         }
            //         num_total++;
            //     }
            // }
            // std::cout << "Num. total: " << num_total << "\n";
            // std::cout << "Num. real: " << num_real << " (" << 100 * float(num_real) / num_total << " [%]) \n";
            // std::cout << "Num. underest: " << num_underest << " (" << 100 * float(num_underest) / num_total << " [%]) \n";
            // std::cout << "Num. real + num. underest: " << num_real+num_underest << " (" 
            //           << 100 * float(num_real + num_underest) / num_total << " [%]) \n";

            for (size_t i = 0; i < trees.size(); i++)
                output_file << *trees[i];

            output_file << "Cost convergence: \n" 
                        << "Cost [rad]\t\tNum. states\t\tTime [s]" << std::endl;
			for (size_t i = 0; i < planner_info->getNumStates(); i++)
                output_file << planner_info->getCostConvergence()[i] << "\t\t"
							<< i+1 << "\t\t"
							<< planner_info->getStateTimes()[i] << std::endl;
                            
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
