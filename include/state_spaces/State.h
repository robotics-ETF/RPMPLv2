//
// Created by dinko on 7.3.21.
// Modified by nermin on 18.02.22.
//

#ifndef RPMPL_STATE_H
#define RPMPL_STATE_H

#include <Eigen/Dense>
#include <vector>
#include <memory>

#include "StateSpaceType.h"
namespace base
{
	class State
	{
	public:
		enum class Status {None, Advanced, Trapped, Reached};
		
	protected:
		base::StateSpaceType state_space_type;
		size_t num_dimensions;											// Dimensionality in C-space
		Eigen::VectorXf coord;											// Coordinates in C-space
		size_t tree_idx;												// Tree index in which the state is stored
		size_t idx; 													// Index of the state in the tree
		float d_c;														// Distance-to-obstacles
		std::vector<float> d_c_profile; 								// Distance-to-obstacles for each robot's link
		bool is_real_d_c;												// Is real or underestimation of distance-to-obstacles used
		float cost;                  									// Cost-to-come
		std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points;	// Set of nearest points between each robot segment and each obstacle
		std::shared_ptr<State> parent;
		std::shared_ptr<std::vector<std::shared_ptr<State>>> children;
		
	public:
		State() {}
		State(const Eigen::VectorXf &coord_);
		virtual ~State() = 0;

		inline base::StateSpaceType getStateSpaceType() const { return state_space_type; }
		inline size_t getNumDimensions() const { return num_dimensions; }
		inline const Eigen::VectorXf &getCoord() const { return coord; }
		inline float getCoord(size_t idx) const { return coord(idx); }
		inline size_t getTreeIdx() const { return tree_idx; }
		inline size_t getIdx() const { return idx; }
		inline float getDistance() const { return d_c; }
		inline const std::vector<float> &getDistanceProfile() const {return d_c_profile; }
		inline bool getIsRealDistance() const { return is_real_d_c; }
		inline float getCost() const { return cost; }
		inline std::shared_ptr<std::vector<Eigen::MatrixXf>> getNearestPoints() const { return nearest_points; }
		inline std::shared_ptr<State> getParent() const { return parent; }
		inline std::shared_ptr<std::vector<std::shared_ptr<State>>> getChildren() const { return children; };

		inline void setStateSpaceType(base::StateSpaceType state_space_type_) { state_space_type = state_space_type_; }
		inline void setNumDimensions(size_t num_dimensions_) { num_dimensions = num_dimensions_; }
		inline void setCoord(const Eigen::VectorXf &coord_) { coord = coord_; }
		inline void setCoord(const float coord_, size_t idx) { coord(idx) = coord_; }
		inline void setTreeIdx(size_t tree_idx_) { tree_idx = tree_idx_; }
		inline void setIdx(size_t idx_) { idx = idx_; }
		inline void setDistance(float d_c_) { d_c = d_c_; }
		inline void setDistanceProfile(const std::vector<float> &d_c_profile_) { d_c_profile = d_c_profile_; }
		inline void setIsRealDistance(bool is_real_d_c_) { is_real_d_c = is_real_d_c_; }
		inline void setCost(float cost_) { cost = cost_; }
		inline void setNearestPoints(const std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points_) { nearest_points = nearest_points_; }
		inline void setParent(const std::shared_ptr<State> parent_) { parent = parent_; }
		inline void setChildren(const std::shared_ptr<std::vector<std::shared_ptr<State>>> children_) { children = children_; }

		void addChild(const std::shared_ptr<State> child);
		friend std::ostream &operator<<(std::ostream &os, const std::shared_ptr<base::State> state);
	};
}
#endif //RPMPL_STATE_H
