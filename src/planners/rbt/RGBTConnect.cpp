//
// Created by nermin on 19.02.22.
//

#include "RGBTConnect.h"
#include "ConfigurationReader.h"

// #include <glog/log_severity.h>
// #include <glog/logging.h>
// WARNING: You need to be very careful with LOG(INFO) for console output, due to a possible "stack smashing detected" error.
// If you get this error, just use std::cout for console output.

planning::rbt::RGBTConnect::RGBTConnect(const std::shared_ptr<base::StateSpace> ss_) : RBTConnect(ss_) {}

planning::rbt::RGBTConnect::RGBTConnect(const std::shared_ptr<base::StateSpace> ss_, const std::shared_ptr<base::State> q_start_,
                                        const std::shared_ptr<base::State> q_goal_) : RBTConnect(ss_, q_start_, q_goal_) {}

bool planning::rbt::RGBTConnect::solve()
{
	time_alg_start = std::chrono::steady_clock::now();		// Start the clock
	int tree_idx = 0;  // Determines the tree index, i.e., which tree is chosen, 0: from q_start; 1: from q_goal
	std::shared_ptr<base::State> q_e, q_near, q_new;
    std::shared_ptr<std::vector<std::shared_ptr<base::State>>> q_new_list;
	base::State::Status status{base::State::None};

	while (true)
	{
		/* Generating generalized bur */
		// std::cout << "Iteration: " << planner_info->getNumIterations() << "\n";
		// std::cout << "Num. states: " << planner_info->getNumStates() << "\n";
		q_e = ss->getRandomState();
		// std::cout << q_rand->getCoord().transpose() << "\n";
		q_near = trees[tree_idx]->getNearestState(q_e);
		// std::cout << "Tree: " << trees[treeNum]->getTreeName() << "\n";
		if (ss->computeDistance(q_near) > RBTConnectConfig::D_CRIT)
		{
			for (size_t i = 0; i < RBTConnectConfig::NUM_SPINES; i++)
			{
				q_e = getRandomState(q_near);				
				tie(status, q_new_list) = extendGenSpine2(q_near, q_e);
                trees[tree_idx]->upgradeTree(q_new_list->front(), q_near);
                for (size_t j = 1; j < q_new_list->size(); j++)
				    trees[tree_idx]->upgradeTree(q_new_list->at(j), q_new_list->at(j-1));
			}
            q_new = q_new_list->back();
		}
		else	// Distance-to-obstacles is less than d_crit
		{
			tie(status, q_new) = extend(q_near, q_e);
			if (status != base::State::Status::Trapped)
				trees[tree_idx]->upgradeTree(q_new, q_near);
		}
		
		tree_idx = 1 - tree_idx; 	// Swapping trees

		/* GBur-Connect */
		if (status != base::State::Status::Trapped)
		{
			q_near = trees[tree_idx]->getNearestState(q_new);
			status = connectGenSpine(trees[tree_idx], q_near, q_new);
		}

		/* Planner info and terminating condition */
	 	planner_info->setNumIterations(planner_info->getNumIterations() + 1);
		planner_info->addIterationTime(getElapsedTime(time_alg_start));
		planner_info->setNumStates(trees[0]->getNumStates() + trees[1]->getNumStates());
		if (checkTerminatingCondition(status))
			return planner_info->getSuccessState();
		
    }
}

// Generalized spine is generated from 'q' towards 'q_e'
// 'q_new' is the final state from the generalized spine
std::tuple<base::State::Status, std::shared_ptr<base::State>> 
    planning::rbt::RGBTConnect::extendGenSpine(const std::shared_ptr<base::State> q, const std::shared_ptr<base::State> q_e)
{
    float d_c = ss->computeDistance(q);
	std::shared_ptr<base::State> q_new = q;
	base::State::Status status;

    for (size_t i = 0; i < RGBTConnectConfig::NUM_LAYERS; i++)
    {
        tie(status, q_new) = extendSpine(q_new, q_e);
        d_c = ss->computeDistanceUnderestimation(q_new, q->getNearestPoints());
		// d_c = ss->computeDistance(q_new);	// If you want to use real distance
        if (d_c < RBTConnectConfig::D_CRIT || status == base::State::Status::Reached)
            break;
    }
    return {status, q_new};
}

// Generalized spine is generated from 'q' towards 'q_e'
// 'q_new_list' contains all states from the generalized spine
std::tuple<base::State::Status, std::shared_ptr<std::vector<std::shared_ptr<base::State>>>> 
    planning::rbt::RGBTConnect::extendGenSpine2(const std::shared_ptr<base::State> q, const std::shared_ptr<base::State> q_e)
{
    float d_c = ss->computeDistance(q);
	std::shared_ptr<base::State> q_new = q;
	std::vector<std::shared_ptr<base::State>> q_new_list;
    base::State::Status status;

    for (size_t i = 0; i < RGBTConnectConfig::NUM_LAYERS; i++)
    {
        std::shared_ptr<base::State> q_temp = ss->getNewState(q_new);
        tie(status, q_new) = extendSpine(q_temp, q_e);
		q_new_list.emplace_back(q_new);
        d_c = ss->computeDistanceUnderestimation(q_new, q->getNearestPoints());
		// d_c = ss->computeDistance(q_new);	// If you want to use real distance
        if (d_c < RBTConnectConfig::D_CRIT || status == base::State::Status::Reached)
            break;
    }
    return {status, std::make_shared<std::vector<std::shared_ptr<base::State>>>(q_new_list)};
}

base::State::Status planning::rbt::RGBTConnect::connectGenSpine
	(const std::shared_ptr<base::Tree> tree, const std::shared_ptr<base::State> q, const std::shared_ptr<base::State> q_e)
{
    float d_c = ss->computeDistance(q);
	std::shared_ptr<base::State> q_new = q;
    std::shared_ptr<std::vector<std::shared_ptr<base::State>>> q_new_list;
	base::State::Status status = base::State::Status::Advanced;
	size_t num_ext = 0;
	
	while (status == base::State::Status::Advanced && num_ext++ < RRTConnectConfig::MAX_EXTENSION_STEPS)
	{
		std::shared_ptr<base::State> q_temp = ss->getNewState(q_new);
		if (d_c > RBTConnectConfig::D_CRIT)
		{
			tie(status, q_new_list) = extendGenSpine2(q_temp, q_e);
            tree->upgradeTree(q_new_list->front(), q_temp);
            for (size_t i = 1; i < q_new_list->size(); i++)
                tree->upgradeTree(q_new_list->at(i), q_new_list->at(i-1));
			
            q_new = q_new_list->back();
            d_c = ss->computeDistance(q_new);
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

bool planning::rbt::RGBTConnect::checkTerminatingCondition(base::State::Status status)
{
	if (status == base::State::Status::Reached)
	{
		computePath();
		planner_info->setSuccessState(true);
		planner_info->setPlanningTime(getElapsedTime(time_alg_start));
		return true;
	}

	float time_current = getElapsedTime(time_alg_start);
	if (time_current >= RGBTConnectConfig::MAX_PLANNING_TIME ||
		planner_info->getNumStates() >= RGBTConnectConfig::MAX_NUM_STATES || 
		planner_info->getNumIterations() >= RGBTConnectConfig::MAX_NUM_ITER)
	{
		planner_info->setSuccessState(false);
		planner_info->setPlanningTime(time_current);
		return true;
	}

	return false;
}

void planning::rbt::RGBTConnect::outputPlannerData(const std::string &filename, bool output_states_and_paths, bool append_output) const
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
		output_file << "Planner type:    " << "RGBTConnect" << std::endl;
		output_file << "Planner info:\n";
		output_file << "\t Succesfull:           " << (planner_info->getSuccessState() ? "yes" : "no") << std::endl;
		output_file << "\t Number of iterations: " << planner_info->getNumIterations() << std::endl;
		output_file << "\t Number of states:     " << planner_info->getNumStates() << std::endl;
		output_file << "\t Planning time [s]:    " << planner_info->getPlanningTime() << std::endl;
		if (output_states_and_paths)
		{
			// Just to check how many states have real or underestimation of distance-to-obstacles computed
            // int num_real = 0, num_underest = 0, num_total = 0;
            // for (int i = 0; i < trees.size(); i++)
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