#include "RRTConnect.h"

planning::rrt::RRTConnect::RRTConnect(const std::shared_ptr<base::StateSpace> ss_) : AbstractPlanner(ss_) 
{
	planner_type = planning::PlannerType::RRTConnect;
}

planning::rrt::RRTConnect::RRTConnect(const std::shared_ptr<base::StateSpace> ss_, const std::shared_ptr<base::State> q_start_,
									  const std::shared_ptr<base::State> q_goal_) : AbstractPlanner(ss_, q_start_, q_goal_)
{
	// std::cout << "Initializing planner...\n";
	planner_type = planning::PlannerType::RRTConnect;
	if (!ss->isValid(q_start) || ss->robot->checkSelfCollision(q_start))
		throw std::domain_error("Start position is invalid!");
	if (!ss->isValid(q_goal) || ss->robot->checkSelfCollision(q_goal))
		throw std::domain_error("Goal position is invalid!");
		
	trees.emplace_back(std::make_shared<base::Tree>("q_start", 0));
	trees.emplace_back(std::make_shared<base::Tree>("q_goal", 1));
	trees[0]->setKdTree(std::make_shared<base::KdTree>(ss->num_dimensions, *trees[0], nanoflann::KDTreeSingleIndexAdaptorParams(10)));
	trees[1]->setKdTree(std::make_shared<base::KdTree>(ss->num_dimensions, *trees[1], nanoflann::KDTreeSingleIndexAdaptorParams(10)));
	trees[0]->upgradeTree(q_start, nullptr);
	trees[1]->upgradeTree(q_goal, nullptr);
	planner_info->setNumIterations(0);
    planner_info->setNumStates(2);
	// std::cout << "Planner initialized!\n";
}

planning::rrt::RRTConnect::~RRTConnect()
{
    for (size_t i = 0; i < trees.size(); i++) {
        trees[i]->clearTree();
	}

	trees.clear();
	path.clear();
}

bool planning::rrt::RRTConnect::solve()
{
	// std::cout << "Entering solve ...\n";
	time_alg_start = std::chrono::steady_clock::now(); 	// Start the clock
	size_t tree_idx { 0 };  	// Determines a tree index, i.e., which tree is chosen, 0: from q_start; 1: from q_goal
	std::shared_ptr<base::State> q_rand { nullptr }; 
	std::shared_ptr<base::State> q_near { nullptr };
	std::shared_ptr<base::State> q_new { nullptr };
	base::State::Status status { base::State::Status::None };

	while (true)
	{
		/* Extend */
		// std::cout << "Iteration: " << planner_info->getNumIterations() << "\n";
		// std::cout << "Num. states: " << planner_info->getNumStates() << "\n";
		q_rand = ss->getRandomState();
		// std::cout << q_rand->getCoord().transpose() << "\n";
		q_near = trees[tree_idx]->getNearestState(q_rand);
		// q_near = trees[tree_idx]->getNearestState2(q_rand);

		// std::cout << "Tree: " << trees[tree_idx]->getTreeName() << "\n";
		tie(status, q_new) = extend(q_near, q_rand);
		// std::cout << "Status: " << status << "\n";
		if (status != base::State::Status::Trapped)
			trees[tree_idx]->upgradeTree(q_new, q_near);

		tree_idx = 1 - tree_idx; 	// Swapping trees

		/* Connect */
		if (status != base::State::Status::Trapped)
		{	
			// std::cout << "Not Trapped \n";
			// std::cout << "Trying to connect to: " << q_new->getCoord().transpose() << " from " << trees[tree_idx]->getTreeName() << "\n";
			q_near = trees[tree_idx]->getNearestState(q_new);
			// q_near = trees[tree_idx]->getNearestState2(q_new);
			status = connect(trees[tree_idx], q_near, q_new);
		}
		
		/* Planner info and terminating condition */
        planner_info->setNumIterations(planner_info->getNumIterations() + 1);
		planner_info->addIterationTime(getElapsedTime(time_alg_start));
		planner_info->setNumStates(trees[0]->getNumStates() + trees[1]->getNumStates());
		if (checkTerminatingCondition(status))
			return planner_info->getSuccessState();
		
	}
}

base::Tree planning::rrt::RRTConnect::getTree(size_t tree_idx) const
{
	return *trees[tree_idx];
}

std::tuple<base::State::Status, std::shared_ptr<base::State>> planning::rrt::RRTConnect::extend
	(const std::shared_ptr<base::State> q, const std::shared_ptr<base::State> q_e)
{
	// std::cout << "Inside extend. \n";
	base::State::Status status { base::State::Status::None };
	std::shared_ptr<base::State> q_new { nullptr };
	tie(status, q_new) = ss->interpolateEdge2(q, q_e, RRTConnectConfig::EPS_STEP);

	if (ss->isValid(q, q_new) && !ss->robot->checkSelfCollision(q, q_new))
		return {status, q_new};
	else
		return {base::State::Status::Trapped, q};
	// std::cout << "Extended. \n";
}

base::State::Status planning::rrt::RRTConnect::connect
	(const std::shared_ptr<base::Tree> tree, const std::shared_ptr<base::State> q, const std::shared_ptr<base::State> q_e)
{
	// std::cout << "Inside connect. \n";
	std::shared_ptr<base::State> q_new { q };
	base::State::Status status { base::State::Status::Advanced };
	size_t num_ext { 0 };

	while (status == base::State::Status::Advanced && num_ext++ < RRTConnectConfig::MAX_EXTENSION_STEPS)
	{
		std::shared_ptr<base::State> q_temp { ss->getNewState(q_new) };
		tie(status, q_new) = extend(q_temp, q_e);
		if (status != base::State::Status::Trapped)
			tree->upgradeTree(q_new, q_temp);
	}
	// std::cout << "Connected. \n";
	return status;
}

void planning::rrt::RRTConnect::computePath()
{
	path.clear();
	std::shared_ptr<base::State> q_con { trees[0]->getStates()->back() };		
	while (q_con->getParent() != nullptr)
	{
		path.emplace_back(q_con->getParent());
		q_con = q_con->getParent();
	}
	std::reverse(path.begin(), path.end());

	q_con = trees[1]->getStates()->back();		
	while (q_con != nullptr)
	{
		path.emplace_back(q_con);
		q_con = q_con->getParent();
	}
}

const std::vector<std::shared_ptr<base::State>> &planning::rrt::RRTConnect::getPath() const
{
	return path;
}

bool planning::rrt::RRTConnect::checkTerminatingCondition(base::State::Status status)
{
	if (status == base::State::Status::Reached)
	{
		computePath();
		planner_info->setSuccessState(true);
		planner_info->setPlanningTime(getElapsedTime(time_alg_start));
		return true;
	}

	float time_current { getElapsedTime(time_alg_start) };
	if (time_current >= RRTConnectConfig::MAX_PLANNING_TIME ||
		planner_info->getNumStates() >= RRTConnectConfig::MAX_NUM_STATES || 
		planner_info->getNumIterations() >= RRTConnectConfig::MAX_NUM_ITER)
	{
		planner_info->setSuccessState(false);
		planner_info->setPlanningTime(time_current);
		return true;
	}

	return false;
}

void planning::rrt::RRTConnect::outputPlannerData(const std::string &filename, bool output_states_and_paths, bool append_output) const
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