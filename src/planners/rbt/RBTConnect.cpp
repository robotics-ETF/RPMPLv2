//
// Created by nermin on 18.02.22.
//

#include "RBTConnect.h"
#include "ConfigurationReader.h"

// #include <glog/log_severity.h>
// #include <glog/logging.h>
// WARNING: You need to be very careful with LOG(INFO) for console output, due to a possible "stack smashing detected" error.
// If you get this error, just use std::cout for console output.

planning::rbt::RBTConnect::RBTConnect(const std::shared_ptr<base::StateSpace> ss_) : RRTConnect(ss_) {}

planning::rbt::RBTConnect::RBTConnect(const std::shared_ptr<base::StateSpace> ss_, const std::shared_ptr<base::State> q_start_,
                                      const std::shared_ptr<base::State> q_goal_) : RRTConnect(ss_, q_start_, q_goal_) {}

bool planning::rbt::RBTConnect::solve()
{
	time_start = std::chrono::steady_clock::now(); 	// Start the clock
	int tree_idx = 0;  	// Determines the tree index, i.e., which tree is chosen, 0: from q_start; 1: from q_goal
	std::shared_ptr<base::State> q_e, q_near, q_new;
	base::State::Status status;

	while (true)
	{
		/* Generating bur */
		// std::cout << "Iteration: " << planner_info->getNumIterations() << "\n";
		// std::cout << "Num. states: " << planner_info->getNumStates() << "\n";
		q_e = ss->getRandomState();
		// std::cout << q_rand->getCoord().transpose() << "\n";
		q_near = trees[tree_idx]->getNearestState(q_e);
		// std::cout << "Tree: " << trees[treeNum]->getTreeName() << "\n";
		if (ss->computeDistance(q_near) > RBTConnectConfig::D_CRIT)
		{
			for (int i = 0; i < RBTConnectConfig::NUM_SPINES; i++)
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
		planner_info->addIterationTime(getElapsedTime(time_start, std::chrono::steady_clock::now()));
		planner_info->setNumStates(trees[0]->getNumStates() + trees[1]->getNumStates());
		if (checkTerminatingCondition(status))
			return planner_info->getSuccessState();
		
    }
}

// Get a random state 'q_rand' with uniform distribution, which is centered around 'q_center'.
// The spine from 'q_center' to 'q_rand' is saturated and prunned.
std::shared_ptr<base::State> planning::rbt::RBTConnect::getRandomState(const std::shared_ptr<base::State> q_center)
{
	std::shared_ptr<base::State> q_rand;
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
	float rho = 0; 														// The path length in W-space for complete robot
	float step;
	int counter = 0;
	std::shared_ptr<base::State> q_new = ss->getNewState(q->getCoord());
	std::shared_ptr<Eigen::MatrixXf> skeleton = ss->robot->computeSkeleton(q);
	std::shared_ptr<Eigen::MatrixXf> skeleton_new = skeleton;
	
	while (true)
	{
		if (RBTConnectConfig::USE_EXPANDED_BUBBLE)
			step = ss->robot->computeStep2(q_new, q_e, q->getDistanceProfile(), rho_profile, skeleton_new);
		else
			step = ss->robot->computeStep(q_new, q_e, q->getDistance(), rho, skeleton_new);

		if (step > 1)
		{
			q_new->setCoord(q_e->getCoord());
			return {base::State::Status::Reached, q_new};
		}
		else
			q_new->setCoord(q_new->getCoord() + step * (q_e->getCoord() - q_new->getCoord()));
		
		if (++counter == RBTConnectConfig::NUM_ITER_SPINE)
			return {base::State::Status::Advanced, q_new};

		rho_profile = std::vector<float>(ss->robot->getNumLinks(), 0);
		skeleton_new = ss->robot->computeSkeleton(q_new);
		float rho_k;
		float rho_k1 = 0;
		for (int k = 1; k <= ss->robot->getNumLinks(); k++)
		{
			rho_k = (skeleton->col(k) - skeleton_new->col(k)).norm();
			rho_profile[k-1] = std::max(rho_k1, rho_k);
			rho_k1 = rho_k;
			rho = std::max(rho, rho_k);
		}
	}
}

base::State::Status planning::rbt::RBTConnect::connectSpine
	(const std::shared_ptr<base::Tree> tree, const std::shared_ptr<base::State> q, const std::shared_ptr<base::State> q_e)
{
	float d_c = ss->computeDistance(q);
	std::shared_ptr<base::State> q_new = q;
	base::State::Status status = base::State::Status::Advanced;
	int num_ext = 0;

	while (status == base::State::Status::Advanced && num_ext++ < RRTConnectConfig::MAX_EXTENSION_STEPS)
	{
		std::shared_ptr<base::State> q_temp = ss->getNewState(q_new);
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
		planner_info->setPlanningTime(getElapsedTime(time_start, std::chrono::steady_clock::now()));
		return true;
	}

	int time_current = getElapsedTime(time_start, std::chrono::steady_clock::now());
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
	std::ofstream output_file;
	std::ios_base::openmode mode = std::ofstream::out;
	if (append_output)
		mode = std::ofstream::app;

	output_file.open(filename, mode);
	if (output_file.is_open())
	{
		output_file << "Space Type:      " << ss->getStateSpaceType() << std::endl;
		output_file << "Dimensionality:  " << ss->getNumDimensions() << std::endl;
		output_file << "Planner type:    " << "RBTConnect" << std::endl;
		output_file << "Planner info:\n";
		output_file << "\t Succesfull:           " << (planner_info->getSuccessState() ? "yes" : "no") << std::endl;
		output_file << "\t Number of iterations: " << planner_info->getNumIterations() << std::endl;
		output_file << "\t Number of states:     " << planner_info->getNumStates() << std::endl;
		output_file << "\t Planning time [ms]:   " << planner_info->getPlanningTime() << std::endl;
		if (output_states_and_paths)
		{
			// Just to check how many states have distance-to-obstacles computed
            // int num_real = 0, num_total = 0;
            // for (int i = 0; i < trees.size(); i++)
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
