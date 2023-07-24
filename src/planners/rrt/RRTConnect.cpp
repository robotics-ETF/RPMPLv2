//
// Created by dinko on 16.3.21.
// Modified by nermin on 18.02.22.
//

#include "RRTConnect.h"
#include "ConfigurationReader.h"
#include <glog/log_severity.h>
#include <glog/logging.h>
// WARNING: You need to be very careful with LOG(INFO) for console output, due to a possible "stack smashing detected" error.
// If you get this error, just use std::cout for console output.

planning::rrt::RRTConnect::RRTConnect(std::shared_ptr<base::StateSpace> ss_) : AbstractPlanner(ss_) {}

planning::rrt::RRTConnect::RRTConnect(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_,
									  std::shared_ptr<base::State> goal_) : AbstractPlanner(ss_, start_, goal_)
{
	// LOG(INFO) << "Initializing planner...";
	std::cout << "Initializing planner...\n";
	if (!ss->isValid(start_) || !ss->isValid(goal_))
		throw std::domain_error("Start or goal positions are invalid!");
		
	trees.emplace_back(std::make_shared<base::Tree>("start", 0));
	trees.emplace_back(std::make_shared<base::Tree>("goal", 1));
	trees[0]->setKdTree(std::make_shared<base::KdTree>(ss->getDimensions(), *trees[0], nanoflann::KDTreeSingleIndexAdaptorParams(10)));
	trees[1]->setKdTree(std::make_shared<base::KdTree>(ss->getDimensions(), *trees[1], nanoflann::KDTreeSingleIndexAdaptorParams(10)));
	trees[0]->upgradeTree(start, nullptr);
	trees[1]->upgradeTree(goal, nullptr);
	planner_info->setNumIterations(0);
    planner_info->setNumStates(2);
	// LOG(INFO) << "Planner initialized!";
	std::cout << "Planner initialized!\n";
}

planning::rrt::RRTConnect::~RRTConnect()
{
	trees[0]->clearTree();
	trees[1]->clearTree();
	trees.clear();
	path.clear();
}

bool planning::rrt::RRTConnect::solve()
{
	// LOG(INFO) << "Entering solve ...";
	auto time_start = std::chrono::steady_clock::now(); 	// Start the clock
	auto time_current = time_start;
	int tree_idx = 0;  	// Determines the tree index, i.e., which tree is chosen, 0: from q_start; 1: from q_goal
	std::shared_ptr<base::State> q_rand, q_near, q_new;
	base::State::Status status;

	while (true)
	{
		/* Extend */
		// LOG(INFO) << "Iteration: " << planner_info->getNumIterations();
		// LOG(INFO) << "Num. states: " << planner_info->getNumStates();
		q_rand = getSS()->randomState();
		// LOG(INFO) << q_rand->getCoord().transpose();
		q_near = trees[tree_idx]->getNearestState(q_rand);
		// q_near = trees[tree_idx]->getNearestStateV2(q_rand);

		// LOG(INFO) << "Tree: " << trees[tree_idx]->getTreeName();
		tie(status, q_new) = extend(q_near, q_rand);
		// LOG(INFO) << "Status: " << status;
		if (status != base::State::Status::Trapped)
			trees[tree_idx]->upgradeTree(q_new, q_near);

		tree_idx = 1 - tree_idx; 	// Swapping trees

		/* Connect */
		if (status != base::State::Status::Trapped)
		{	
			// LOG(INFO) << "Not Trapped";
			// LOG(INFO) << "Trying to connect to: " << q_new->getCoord().transpose() << " from " << trees[tree_idx]->getTreeName();
			q_near = trees[tree_idx]->getNearestState(q_new);
			// q_near = trees[tree_idx]->getNearestStateV2(q_new);
			status = connect(trees[tree_idx], q_near, q_new);
		}
		
		/* Planner info and terminating condition */
		time_current = std::chrono::steady_clock::now();
        planner_info->setNumIterations(planner_info->getNumIterations() + 1);
		planner_info->addIterationTime(getElapsedTime(time_start, time_current));
		planner_info->setNumStates(trees[0]->getNumStates() + trees[1]->getNumStates());
		if (checkTerminatingCondition(status))
		{
			planner_info->setPlanningTime(planner_info->getIterationTimes().back());
			return planner_info->getSuccessState();
		}
	}
}

base::Tree planning::rrt::RRTConnect::getTree(int tree_idx) const
{
	return *trees[tree_idx];
}

std::tuple<base::State::Status, std::shared_ptr<base::State>> planning::rrt::RRTConnect::extend
	(std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e)
{
	// LOG(INFO) << "Inside extend.";
	base::State::Status status;
	std::shared_ptr<base::State> q_new;
	tie(status, q_new) = ss->interpolate(q, q_e, RRTConnectConfig::EPS_STEP);
	if (status != base::State::Status::Trapped && ss->isValid(q, q_new))
		return {status, q_new};
	else
		return {base::State::Status::Trapped, q};
	// LOG(INFO) << "Extended.";
}

base::State::Status planning::rrt::RRTConnect::connect
	(std::shared_ptr<base::Tree> tree, std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e)
{
	// LOG(INFO) << "Inside connect.";
	std::shared_ptr<base::State> q_new = q;
	base::State::Status status = base::State::Status::Advanced;
	int num_ext = 0;
	while (status == base::State::Status::Advanced && num_ext++ < RRTConnectConfig::MAX_EXTENSION_STEPS)
	{
		std::shared_ptr<base::State> q_temp = ss->newState(q_new);
		tie(status, q_new) = extend(q_temp, q_e);
		if (status != base::State::Status::Trapped)
			tree->upgradeTree(q_new, q_temp);
	}
	// LOG(INFO) << "Connected.";
	return status;
}

void planning::rrt::RRTConnect::computePath()
{
	path.clear();
	std::shared_ptr<base::State> q_con = trees[0]->getStates()->back();		
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

// Get elapsed time (defaultly in milliseconds) from 'time_start' to 'time_current'
float planning::rrt::RRTConnect::getElapsedTime(std::chrono::steady_clock::time_point &time_start,
												std::chrono::steady_clock::time_point &time_current,
												std::string time_unit)
{
	if (time_unit == "milliseconds")
		return std::chrono::duration_cast<std::chrono::milliseconds>(time_current - time_start).count();
	else if (time_unit == "microseconds")
		return std::chrono::duration_cast<std::chrono::microseconds>(time_current - time_start).count();
	else
		LOG(INFO) << "Error in measuring time!";
}

bool planning::rrt::RRTConnect::checkTerminatingCondition(base::State::Status status)
{
	if (status == base::State::Status::Reached)
	{
		planner_info->setSuccessState(true);
		computePath();
		return true;
	}
	else if (planner_info->getNumStates() >= RRTConnectConfig::MAX_NUM_STATES || 
			 planner_info->getIterationTimes().back() >= RRTConnectConfig::MAX_PLANNING_TIME ||
			 planner_info->getNumIterations() >= RRTConnectConfig::MAX_NUM_ITER)
	{
		planner_info->setSuccessState(false);
		return true;
	}
	return false;
}

void planning::rrt::RRTConnect::outputPlannerData(std::string filename, bool output_states_and_paths, bool append_output) const
{
	std::ofstream output_file;
	std::ios_base::openmode mode = std::ofstream::out;
	if (append_output)
		mode = std::ofstream::app;

	output_file.open(filename, mode);
	if (output_file.is_open())
	{
		output_file << "Space Type:      " << ss->getStateSpaceType() << std::endl;
		output_file << "Space dimension: " << ss->getDimensions() << std::endl;
		output_file << "Planner type:    " << "RRTConnect" << std::endl;
		output_file << "Planner info:\n";
		output_file << "\t Succesfull:           " << (planner_info->getSuccessState() ? "yes" : "no") << std::endl;
		output_file << "\t Number of iterations: " << planner_info->getNumIterations() << std::endl;
		output_file << "\t Number of states:     " << planner_info->getNumStates() << std::endl;
		output_file << "\t Planning time [ms]:   " << planner_info->getPlanningTime() << std::endl;
		if (output_states_and_paths)
		{
			output_file << *trees[0];
			output_file << *trees[1];
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