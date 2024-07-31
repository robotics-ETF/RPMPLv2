//
// Created by nermin on 18.02.22.
//

#ifndef RPMPL_TREE_H
#define RPMPL_TREE_H

#include <nanoflann.hpp>

#include "State.h"
#include "StateSpaceType.h"

namespace base
{
	class Tree;
	typedef nanoflann::KDTreeSingleIndexDynamicAdaptor
			<nanoflann::L2_Simple_Adaptor<float, base::Tree>, base::Tree /* dim */> KdTree;

	class Tree
	{
	private:
		std::string tree_name;
		size_t tree_idx;
		std::shared_ptr<std::vector<std::shared_ptr<base::State>>> states; 	// List of all nodes in the tree
        std::shared_ptr<base::KdTree> kd_tree;

	public:
		Tree() {}
		Tree(const std::string &tree_name_, size_t tree_idx_);
		Tree(const std::shared_ptr<std::vector<std::shared_ptr<base::State>>> states_);
		~Tree();

		inline const std::string &getTreeName() const { return tree_name; }
		inline size_t getTreeIdx() const { return tree_idx; }
		inline std::shared_ptr<std::vector<std::shared_ptr<base::State>>> getStates() const { return states; }
		inline std::shared_ptr<base::State> getState(size_t idx) const { return states->at(idx); }
        inline std::shared_ptr<base::KdTree> getKdTree() const { return kd_tree; }
		inline size_t getNumStates() const { return states->size(); }

		inline void setTreeName(const std::string &tree_name_) { tree_name = tree_name_; }
		inline void setTreeIdx(const size_t tree_idx_) { tree_idx = tree_idx_; }
		inline void setStates(const std::shared_ptr<std::vector<std::shared_ptr<base::State>>> states_) { states = states_; }
		inline void setState(const std::shared_ptr<base::State> state, size_t idx) { states->at(idx) = state; }
        inline void setKdTree(const std::shared_ptr<base::KdTree> kdtree_) { kd_tree = kdtree_; }

		void clearTree();
		std::shared_ptr<base::State> getNearestState(const std::shared_ptr<base::State> q);
		std::shared_ptr<base::State> getNearestState2(const std::shared_ptr<base::State> q);
		void upgradeTree(const std::shared_ptr<base::State> q_new, const std::shared_ptr<base::State> q_parent);
		void upgradeTree(const std::shared_ptr<base::State> q_new, const std::shared_ptr<base::State> q_parent, 
						 const std::shared_ptr<base::State> q_ref);

		template <class BBOX> 
        bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
		inline size_t kdtree_get_point_count() const { return states->size(); }
		inline float kdtree_get_pt(const size_t idx, const size_t dim) const { return states->at(idx)->getCoord(dim); }

		friend std::ostream &operator<<(std::ostream &os, const Tree &tree);
	};
}

#endif //RPMPL_TREE_H