//
// Created by dinko on 7.3.21.
// Modified by nermin on 18.02.22.
//

#ifndef RPMPL_STATE_H
#define RPMPL_STATE_H

#include "StateSpaceType.h"
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace base
{
	class State
	{
	public:
		enum Status {Advanced, Trapped, Reached};
		
	private:
		StateSpaceType state_space_type;
		int num_dimensions;												// Dimensionality in C-space
		Eigen::VectorXf coord;											// Coordinates in C-space
		uint tree_idx;													// Tree index in which the state is stored
		size_t idx; 													// Index of the state in the tree
		float d_c;														// Distance-to-obstacles
		float cost;                  									// Cost-to-come
		std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points;	// Set of nearest points between each robot segment and each obstacle
		std::shared_ptr<State> parent;
		std::shared_ptr<std::vector<std::shared_ptr<State>>> children;
		
	public:
		State();
		virtual ~State() = 0;

		inline StateSpaceType getStateSpaceType() const { return state_space_type; }
		inline int getNumDimensions() const { return num_dimensions; }
		inline const Eigen::VectorXf &getCoord() const { return coord; }
		inline float getCoord(int idx) const { return coord(idx); }
		inline uint getTreeIdx() const { return tree_idx; }
		inline size_t getIdx() const { return idx; }
		inline float getDistance() const { return d_c; }
		inline float getCost() const { return cost; }
		inline std::shared_ptr<std::vector<Eigen::MatrixXf>> getNearestPoints() const { return nearest_points; }
		inline std::shared_ptr<State> getParent() const { return parent; }
		inline std::shared_ptr<std::vector<std::shared_ptr<State>>> getChildren() const { return children; };

		inline void setStateSpaceType(StateSpaceType state_space_type_) { state_space_type = state_space_type_; }
		inline void setDimensions(int num_dimensions_) { num_dimensions = num_dimensions_; }
		inline void setCoord(const Eigen::VectorXf &coord_) { coord = coord_; }
		inline void setCoord(const float coord_, int idx) { coord(idx) = coord_; }
		inline void setTreeIdx(uint tree_idx_) { tree_idx = tree_idx_; }
		inline void setIdx(size_t idx_) { idx = idx_; }
		inline void setDistance(float d_c_) { d_c = d_c_; }
		inline void setCost(float cost_) { cost = cost_; }
		inline void setNearestPoints(std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points_) { nearest_points = nearest_points_; }
		inline void setParent(std::shared_ptr<State> parent_) { parent = parent_; }
		inline void setChildren(std::shared_ptr<std::vector<std::shared_ptr<State>>> children_) { children = children_; }

		void addChild(std::shared_ptr<State> child);
		friend std::ostream &operator<<(std::ostream &os, const State *state);
	};
}
#endif //RPMPL_STATE_H
