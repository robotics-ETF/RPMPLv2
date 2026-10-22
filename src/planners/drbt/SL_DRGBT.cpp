#include "SL_DRGBT.h"

planning::drbt::SL_DRGBT::SL_DRGBT(const std::shared_ptr<base::StateSpace> ss_) : 
    DRGBT(ss_), 
    onnx_model(SL_DRGBTConfig::ONNX_MODEL_PATH)
{
    planner_type = planning::PlannerType::SL_DRGBT;
}

planning::drbt::SL_DRGBT::SL_DRGBT(const std::shared_ptr<base::StateSpace> ss_, const std::shared_ptr<base::State> q_start_,
                                   const std::shared_ptr<base::State> q_goal_) : 
                                   DRGBT(ss_, q_start_, q_goal_), 
                                   onnx_model(SL_DRGBTConfig::ONNX_MODEL_PATH)
{
	// std::cout << "Initializing SL_DRGBT planner... \n";
    planner_type = planning::PlannerType::SL_DRGBT;
    pattern_tree = std::make_shared<planning::rbt::PatternTree>(ss, SL_DRGBTConfig::NUM_LAYERS);

    // std::cout << "SL_DRGBT planner initialized! \n";
}

planning::drbt::SL_DRGBT::~SL_DRGBT() {}

bool planning::drbt::SL_DRGBT::solve()
{
    time_alg_start = std::chrono::steady_clock::now();     // Start the algorithm clock
    // std::cout << "----------------------------------------------------------------------------------------\n";

    while (true)
    {
        time_iter_start = std::chrono::steady_clock::now();     // Start the iteration clock
        std::cout << "Iteration: " << planner_info->getNumIterations() << "\n";
        std::cout << "q_current: " << q_current << "\n";

        if (std::get<0>(extendGenSpine(q_current, q_goal)) == base::State::Status::Reached)
        {
            std::cout << "Reaching goal... \n";
            q_next = std::make_shared<planning::drbt::HorizonState>(ss->getNewState(q_goal->getCoord()), -1);
        }
        else
        {
            // ------------------------------------------------------------------------------- //
            // Generate input data and predict the best next state
            std::shared_ptr<base::State> q_current_temp = q_current;
            size_t num { 0 }, num_steps { 10 };

            while (num++ < num_steps)
            {
                std::vector<float> input {};
                generateInput(q_current_temp, input);

                // std::cout << "\t dist: " << q_current_temp->getDistance() << "\t input data: " << "\n";
                // for (auto in : input)
                //     std::cout << in << ", ";
                    
                const auto output = onnx_model.predict(input);
                std::shared_ptr<base::State> q_next_temp = computeNextState(input, output, 2);
                q_current_temp = ss->interpolateEdge(q_current_temp, q_next_temp, RRTConnectConfig::EPS_STEP * rand() / RAND_MAX);
                // std::cout << "q_next_temp_" << num << ":    " << q_next_temp << "\n";
                // std::cout << "q_current_temp_" << num << ": " << q_current_temp << "\n";
            }
            
            q_next = std::make_shared<planning::drbt::HorizonState>(ss->getNewState(q_current_temp->getCoord()), -1);
        }

        computeReachedState(q_next);
        // std::cout << "q_next: " << q_next << "\n";
        
        // ------------------------------------------------------------------------------- //
        // Compute a trajectory and update current state
        // auto time_updateCurrentState { std::chrono::steady_clock::now() };
        visited_states = { q_next };
        updating_state->setNonZeroFinalVel(false);
        updating_state->setTimeIterStart(time_iter_start);
        updating_state->update(q_previous, q_current, q_next->getState(), q_next->getStateReached(), status);
        // planner_info->addRoutineTime(getElapsedTime(time_updateCurrentState, planning::TimeUnit::us), 5);

        // ------------------------------------------------------------------------------- //
        // Checking the real-time execution
        float time_iter_remain = DRGBTConfig::MAX_ITER_TIME * 1e3 - getElapsedTime(time_iter_start, planning::TimeUnit::ms);
        std::cout << "Time elapsed: " << getElapsedTime(time_iter_start, planning::TimeUnit::ms) << " [ms] \n";
        // std::cout << "Remaining iteration time is " << time_iter_remain << " [ms] \n";
        if (time_iter_remain < 0)
            std::cout << "*************** Real-time is broken. " << -time_iter_remain << " [ms] exceeded!!! *************** \n";

        // ------------------------------------------------------------------------------- //
        // Update environment and check if the collision occurs
        bool is_valid { false };
        switch (DRGBTConfig::TRAJECTORY_INTERPOLATION)
        {
        case planning::TrajectoryInterpolation::None:
            is_valid = motion_validity->check(q_previous, q_current);
            break;

        default:
            is_valid = motion_validity->check(traj->getTrajPointCurrentIter());
            break;
        }

        if (!is_valid)
        {
            std::cout << "*************** Collision has been occurred!!! *************** \n";
            planner_info->setSuccessState(false);
            planner_info->setPlanningTime(planner_info->getIterationTimes().back());
            return false;
        }

        // ------------------------------------------------------------------------------- //
        // Planner info and terminating condition
        planner_info->setNumIterations(planner_info->getNumIterations() + 1);
        planner_info->addIterationTime(getElapsedTime(time_alg_start));
        if (checkTerminatingCondition(status))
            return planner_info->getSuccessState();

        std::cout << "----------------------------------------------------------------------------------------\n";
    }
}

void planning::drbt::SL_DRGBT::generateInput(const std::shared_ptr<base::State> &q_root, std::vector<float> &input)
{
    std::shared_ptr<base::Tree> tree { pattern_tree->generateLocalTree(q_root) };
    // std::cout << *tree << "\n";
    
    for (size_t j = 0; j < ss->num_dimensions; j++)
        input.emplace_back(q_root->getCoord(j));

    for (size_t j = 0; j < ss->num_dimensions; j++)
        input.emplace_back(q_goal->getCoord(j));

    for (size_t i = 1; i < pattern_tree->getNumNodes(SL_DRGBTConfig::NUM_LAYERS); i++)
        input.emplace_back((tree->getState(i)->getCoord() - tree->getState(i)->getParent()->getCoord()).norm());

    for (size_t i = 0; i < pattern_tree->getNumNodes(SL_DRGBTConfig::NUM_LAYERS-1); i++)
    {
        // input.emplace_back(tree->getState(i)->getDistance());	// If only distance to obstacles is used

        std::vector<float> d_c_profile { tree->getState(i)->getDistanceProfile() };
        for (size_t j = 0; j < ss->num_dimensions; j++)
            input.emplace_back(d_c_profile[j]);
    }

    for (size_t i = 0; i < pattern_tree->getNumNodes(SL_DRGBTConfig::NUM_LAYERS-1); i++)
    {
        std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points { tree->getState(i)->getNearestPoints() };
        Eigen::Vector3f R {}, R_min {}, O {}, O_min {}, OR {};
        for (size_t j = 0; j < ss->num_dimensions; j++)
        {
            float d_min { RealVectorSpaceConfig::MAX_DISTANCE };
            for (size_t k = 0; k < ss->env->getNumObjects(); k++)
            {
                R = nearest_points->at(k).col(j).head(3);
                O = nearest_points->at(k).col(j).tail(3);
                R += (O - R).normalized() * ss->robot->getCapsuleRadius(j);
                if ((R - O).norm() < d_min)
                {
                    R_min = R;
                    O_min = O;
                    d_min = (R_min - O_min).norm();
                }
            }

            OR = (R_min - O_min).normalized();
            input.emplace_back(OR.x());
            input.emplace_back(OR.y());
            input.emplace_back(OR.z());
        }
    }
}

std::shared_ptr<base::State> planning::drbt::SL_DRGBT::computeNextState
    (const Ort::ShapeInferContext::Floats &input, const Predictor::Probabilities &output, const size_t use_layers)
{
    // Left, right, down, up.
    const int directions[4][2] = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1}
    };

    const int children[4][3] = {
        {0, 2, 3},
        {1, 2, 3},
        {0, 1, 2},
        {0, 1, 3}
    };

    // Find the highest first-layer probability to define the candidate cutoff.
    int first = 0;
    for (int i = 1; i < 4; ++i)
        if (output[i] > output[first])
            first = i;

    // std::cout << "Output probabilities: " << std::setprecision(4);
    // for (std::size_t i = 0; i < output.size(); ++i) 
    // {
    //     if (i) std::cout << ' ';
    //     std::cout << output[i];
    // }
    // std::cout << '\n';

    auto computeCandidate = [&](int first_index)
    {
        // Select the best second-layer child for this first-layer node.
        const int offset = 4 + 3 * first_index;
        int child = 0;
        for (int j = 1; j < 3; ++j)
            if (output[offset + j] > output[offset + child])
                child = j;

        const int second = children[first_index][child];
        const float second_step = (use_layers > 1) ? output[offset + child] * input[8 + 3 * first_index + child] : 0.0f;
        Eigen::VectorXf coord = q_current->getCoord();
        for (int axis = 0; axis < 2; ++axis)
        {
            coord(axis) += input[4 + first_index] * directions[first_index][axis];
            coord(axis) += second_step * directions[second][axis];
        }
        return ss->getNewState(coord);
    };

    auto q_best = computeCandidate(first);
    float distance_best = ss->getNorm(q_best, q_goal);
    int best_index = first;
    // const float probability_cutoff = output[first] - SL_DRGBTConfig::PROBABILITY_THRESHOLD;
    // for (int i = 0; i < 4; ++i)
    // {
    //     if (i == first || output[i] <= probability_cutoff)
    //         continue;

    //     const auto q_candidate = computeCandidate(i);
    //     const float distance = ss->getNorm(q_candidate, q_goal);
    //     if (distance < distance_best)
    //     {
    //         q_best = q_candidate;
    //         distance_best = distance;
    //         best_index = i;
    //     }
    // }

    std::cout << "Best direction: " << best_index << "\n";
    // if (best_index != first)
    //     std::cout << "Best prob. is: " << output[first] 
    //               << ", but choosing the closest node to the goal with prob: " << output[best_index] << "\n";
    
    return q_best;
}

