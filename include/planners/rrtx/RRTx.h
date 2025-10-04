//
// Created by dinko on 18.3.25.
//

#ifndef RPMPL_RRTX_H
#define RPMPL_RRTX_H

#include "RRTConnect.h"
#include "RRTxConfig.h"
#include "UpdatingState.h"
#include "MotionValidity.h"
#include "Trajectory.h"

// #include <glog/log_severity.h>
// #include <glog/logging.h>
// WARNING: You need to be very careful with using LOG(INFO) for console output, due to a possible "stack smashing detected" error.
// If you get this error, just use std::cout for console output.

namespace planning::rrtx
{
    class RRTx : public planning::rrt::RRTConnect
    {
    public:
        // Constructor with state space only
        RRTx(const std::shared_ptr<base::StateSpace> ss_);
        
        // Constructor with state space, start and goal
        RRTx(const std::shared_ptr<base::StateSpace> ss_, 
             const std::shared_ptr<base::State> q_start_,
             const std::shared_ptr<base::State> q_goal_);
            
        // Destructor
        ~RRTx();
        
        // Main planning function
        bool solve() override;
        
        // Update obstacles and mark affected nodes
        bool updateObstacles(const std::vector<std::tuple<std::shared_ptr<base::State>, 
                             std::shared_ptr<base::State>>> &changed_regions);
        
        // Overload for updating obstacles with a single region
        bool updateObstacles(const std::shared_ptr<base::State> region_min, 
                             const std::shared_ptr<base::State> region_max);
        
        // Get the tree
        base::Tree getTree() const;
        
        // Output planner data to file
        void outputPlannerData(const std::string &filename, 
                               bool output_states_and_paths = true, 
                               bool append_output = false) const override;
                               
    protected:
        // Custom comparator for priority queue based on cost
        struct CostComparator 
        {
            bool operator()(const std::shared_ptr<base::State> &a, const std::shared_ptr<base::State> &b) const {
                return a->getCost() > b->getCost();
            }
        };

        // Single tree for RRTx (unlike RRTConnect which uses two trees)
        std::shared_ptr<base::Tree> tree;
        
        // Store start state separately
        std::shared_ptr<base::State> start_state;
        
        // Store current, previous, and next state separately
        std::shared_ptr<base::State> q_current;
        std::shared_ptr<base::State> q_previous;
        std::shared_ptr<base::State> q_next;

        // Radius for rewiring
        float r_rewire;

        // Parameters for computing radius for rewiring
        float eta, mi, zeta, gamma_rrt;
        
        // Sets and queues for dynamic replanning
        std::unordered_set<std::shared_ptr<base::State>> orphan_set;
        std::unordered_set<std::shared_ptr<base::State>> rewire_set;
        std::priority_queue<std::shared_ptr<base::State>, 
                            std::vector<std::shared_ptr<base::State>>, 
                            CostComparator> propagation_queue;
        
        // Path from start to goal
        std::vector<std::shared_ptr<base::State>> path_current;

        std::shared_ptr<planning::trajectory::Trajectory> traj;                     // Trajectory which is generated using splines from RPMPLv2 library
        std::shared_ptr<planning::trajectory::TrajectoryRuckig> traj_ruckig;        // Trajectory which is generated using Ruckig library
        std::shared_ptr<planning::trajectory::UpdatingState> updating_state;        // Class for updating current state
        std::shared_ptr<planning::trajectory::MotionValidity> motion_validity;      // Class for checking validity of motion
        
        // Helper method to calculate distance between states (using getNorm)
        float distance(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) const;
            
        // Find neighbors within a radius
        std::vector<std::shared_ptr<base::State>> findNeighbors(const std::shared_ptr<base::State> q, float radius);
            
        // Choose best parent for a node
        bool chooseParent(std::shared_ptr<base::State> q_new, const std::vector<std::shared_ptr<base::State>> &neighbors);
                            
        // Rewire neighbors to improve path costs
        void rewireNeighbors(std::shared_ptr<base::State> q_new, const std::vector<std::shared_ptr<base::State>> &neighbors = {});
                            
        // Propagate cost changes through the tree
        void propagateCostChanges(std::shared_ptr<base::State> node);
        
        // Check if the current path needs updating
        bool updatePath();
        
        // Compute path from start to goal
        void computePath();
        
        // Handle dynamic obstacles
        void handleDynamicObstacles();
        
        // Mark a node as orphaned
        void markAsOrphan(std::shared_ptr<base::State> node);
        
        // Propagate orphan status to children
        void propagateOrphanStatus();
        
        // Update neighbors after obstacles change
        void updateNeighbors();
        
        // Rewire tree after obstacles change
        void rewireTree();
        
        // Remove orphaned nodes
        void removeOrphanNodes();
        
        // Generate random number in range [min, max]
        float generateRandomNumber(float min, float max);

        // Compute the shrinking ball radius
        float shrinkingBallRadius(size_t num_states);
        
        // Check if planning should terminate
        bool checkTerminatingCondition(base::State::Status status) override;
    };
}

#endif //RPMPL_RRTX_H