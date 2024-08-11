#include "RBTConnect.h"

planning::rbt::RBTConnect::RBTConnect(const std::shared_ptr<base::StateSpace> ss_) : RRTConnect(ss_) 
{
    planner_type = planning::PlannerType::RBTConnect;
}

planning::rbt::RBTConnect::RBTConnect(const std::shared_ptr<base::StateSpace> ss_, const std::shared_ptr<base::State> q_start_,
                                      const std::shared_ptr<base::State> q_goal_) : RRTConnect(ss_, q_start_, q_goal_) 
{
    planner_type = planning::PlannerType::RBTConnect;
}

bool planning::rbt::RBTConnect::solve()
{
	time_alg_start = std::chrono::steady_clock::now(); 	// Start the clock
	size_t tree_idx { 0 };  	// Determines a tree index, i.e., which tree is chosen, 0: from q_start; 1: from q_goal
	std::shared_ptr<base::State> q_e { nullptr };
	std::shared_ptr<base::State> q_near { nullptr };
	std::shared_ptr<base::State> q_new { nullptr };
	base::State::Status status { base::State::Status::None };

	while (true)
	{
		/* Generating bur */
		// std::cout << "Iteration: " << planner_info->getNumIterations() << "\n";
		// std::cout << "Num. states: " << planner_info->getNumStates() << "\n";
		q_e = ss->getRandomState();
		// std::cout << q_e->getCoord().transpose() << "\n";
		q_near = trees[tree_idx]->getNearestState(q_e);
		// std::cout << "Tree: " << trees[tree_idx]->getTreeName() << "\n";
		if (ss->computeDistance(q_near) > RBTConnectConfig::D_CRIT)
		{
			for (size_t i = 0; i < RBTConnectConfig::NUM_SPINES; i++)
			{
				q_e = getRandomState(q_near);
				tie(status, q_new) = extendSpine(q_near, q_e);
				trees[tree_idx]->upgradeTree(q_new, q_near);
			}
		}
		else	// Distance-to-obstacles is less than d_crit
		{
			tie(status, q_new) = extend(q_near, q_e);
			if (status != base::State::Status::Trapped)
				trees[tree_idx]->upgradeTree(q_new, q_near);
		}

		tree_idx = 1 - tree_idx;	// Swapping trees

		/* Bur-Connect */
		if (status != base::State::Status::Trapped)
		{
			q_near = trees[tree_idx]->getNearestState(q_new);
			status = connectSpine(trees[tree_idx], q_near, q_new);
		}

		/* Planner info and terminating condition */
        planner_info->setNumIterations(planner_info->getNumIterations() + 1);
		planner_info->addIterationTime(getElapsedTime(time_alg_start));
		planner_info->setNumStates(trees[0]->getNumStates() + trees[1]->getNumStates());
		if (checkTerminatingCondition(status))
			return planner_info->getSuccessState();
		
    }
}

// Get a random state 'q_rand' with uniform distribution, which is centered around 'q_center'.
// The spine from 'q_center' to 'q_rand' is saturated and prunned.
std::shared_ptr<base::State> planning::rbt::RBTConnect::getRandomState(const std::shared_ptr<base::State> q_center)
{
	std::shared_ptr<base::State> q_rand { nullptr };
	do
	{
		q_rand = ss->getRandomState(q_center);
		q_rand = ss->interpolateEdge(q_center, q_rand, RBTConnectConfig::DELTA);
		q_rand = ss->pruneEdge(q_center, q_rand);
	} 
	while (ss->isEqual(q_center, q_rand));

	return q_rand;
}

// Spine is generated from 'q' towards 'q_e' using complete (expanded) bubble
// 'q_new' is the new reached state
std::tuple<base::State::Status, std::shared_ptr<base::State>> planning::rbt::RBTConnect::extendSpine
	(const std::shared_ptr<base::State> q, const std::shared_ptr<base::State> q_e)
{
	if (q->getDistance() < 0) 	// Just in case. Regularly, 'q' should have a distance!
		ss->computeDistance(q);
	
	std::vector<float> rho_profile(ss->robot->getNumLinks(), 0);	// The path length in W-space for each robot's link
	float rho { 0 }, rho_k { 0 }, rho_k_prev { 0 }; 				// The path length in W-space for (complete) robot
	float step { 0 };
	size_t counter { 0 };
	std::shared_ptr<base::State> q_temp { q };
	std::shared_ptr<base::State> q_new { nullptr };
	std::shared_ptr<Eigen::MatrixXf> skeleton { ss->robot->computeSkeleton(q) };
	std::shared_ptr<Eigen::MatrixXf> skeleton_new { nullptr };
	std::shared_ptr<Eigen::MatrixXf> R { nullptr };
	Eigen::VectorXf delta_q {};
	base::State::Status status { base::State::Status::Advanced };
	bool self_collision { false };
	
	while (true)
	{
		R = ss->robot->computeEnclosingRadii(q_temp);
		delta_q = (q_e->getCoord() - q_temp->getCoord()).cwiseAbs();
		
		if (RBTConnectConfig::USE_EXPANDED_BUBBLE)
		{
			step = INFINITY;
			for (size_t i = 0; i < ss->num_dimensions; i++)
				step = std::min(step, (q->getDistanceProfile(i) - rho_profile[i]) / R->col(i+1).dot(delta_q));
		}
		else
			step = (q->getDistance() - rho) / R->col(ss->num_dimensions).dot(delta_q);	// 'q->getDistance() - rho' is the remaining path length in W-space

		if (step > 1)
		{
			q_new = ss->getNewState(q_e->getCoord());
			status = base::State::Status::Reached;
		}
		else
			q_new = ss->getNewState(q_temp->getCoord() + step * (q_e->getCoord() - q_temp->getCoord()));

		self_collision = ss->robot->checkSelfCollision(q_temp, q_new);
		if (self_collision)
			status = !ss->isEqual(q_temp, q_new) ? base::State::Status::Advanced : base::State::Status::Trapped;
		
		if (++counter == RBTConnectConfig::NUM_ITER_SPINE || status != base::State::Status::Advanced || self_collision)
			return { status, q_new };

		// -------------------------------------------------------- //
		skeleton_new = ss->robot->computeSkeleton(q_new);
		rho_profile = std::vector<float>(ss->robot->getNumLinks(), 0);
		rho_k_prev = 0;
		
		for (size_t k = 1; k <= ss->robot->getNumLinks(); k++)
		{
			rho_k = (skeleton->col(k) - skeleton_new->col(k)).norm();
			rho_profile[k-1] = std::max(rho_k_prev, rho_k);
			rho_k_prev = rho_k;
			rho = std::max(rho, rho_k);
		}
		q_temp = q_new;
	}
}

base::State::Status planning::rbt::RBTConnect::connectSpine
	(const std::shared_ptr<base::Tree> tree, const std::shared_ptr<base::State> q, const std::shared_ptr<base::State> q_e)
{
	float d_c { ss->computeDistance(q) };
	std::shared_ptr<base::State> q_new { q };
	std::shared_ptr<base::State> q_temp { nullptr };
	base::State::Status status { base::State::Status::Advanced };
	size_t num_ext { 0 };

	while (status == base::State::Status::Advanced && num_ext++ < RRTConnectConfig::MAX_EXTENSION_STEPS)
	{
		q_temp = ss->getNewState(q_new);
		if (d_c > RBTConnectConfig::D_CRIT)
		{
			tie(status, q_new) = extendSpine(q_temp, q_e);
			d_c = ss->computeDistance(q_new);
			tree->upgradeTree(q_new, q_temp);
		}
		else
		{
			tie(status, q_new) = extend(q_temp, q_e);
			if (status != base::State::Status::Trapped)
				tree->upgradeTree(q_new, q_temp);
		}
	}
	return status;
}

bool planning::rbt::RBTConnect::checkTerminatingCondition(base::State::Status status)
{
	if (status == base::State::Status::Reached)
	{
		computePath();
		planner_info->setSuccessState(true);
		planner_info->setPlanningTime(getElapsedTime(time_alg_start));
		return true;
	}

	float time_current { getElapsedTime(time_alg_start) };
	if (time_current >= RBTConnectConfig::MAX_PLANNING_TIME ||
		planner_info->getNumStates() >= RBTConnectConfig::MAX_NUM_STATES || 
		planner_info->getNumIterations() >= RBTConnectConfig::MAX_NUM_ITER)
	{
		planner_info->setSuccessState(false);
		planner_info->setPlanningTime(time_current);
		return true;
	}

	return false;
}

void planning::rbt::RBTConnect::outputPlannerData(const std::string &filename, bool output_states_and_paths, bool append_output) const
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
		if (output_states_and_paths)
		{
			// Just to check how many states have distance-to-obstacles computed
            // size_t num_real { 0 };
			// size_t num_total { 0 };
            // for (size_t i = 0; i < trees.size(); i++)
            // {
            //     output_file << *trees[i];
            //     for (auto q : *trees[i]->getStates())
            //     {
            //         if (q->getDistance() > 0)
            //         	num_real++;
            //         num_total++;
            //     }
            // }
            // std::cout << "Num. total: " << num_total << "\n";
            // std::cout << "Num. real: " << num_real << " (" << 100 * float(num_real) / num_total << " [%]) \n";

			output_file << *trees[0];
			output_file << *trees[1];
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
