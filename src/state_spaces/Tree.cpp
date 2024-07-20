#include "Tree.h"

base::Tree::Tree(const std::string &tree_name_, size_t tree_idx_)
{
	tree_name = tree_name_;
	tree_idx = tree_idx_;
	states = std::make_shared<std::vector<std::shared_ptr<base::State>>>();
	kd_tree = nullptr;
}

base::Tree::Tree(const std::shared_ptr<std::vector<std::shared_ptr<base::State>>> states_)
{
	states = states_;
	kd_tree = nullptr;
}

base::Tree::~Tree()
{
	clearTree();
}

void base::Tree::clearTree()
{
	states->clear();
}

std::shared_ptr<base::State> base::Tree::getNearestState(const std::shared_ptr<base::State> q)
{
	const size_t num_results { 1 };
	size_t q_near_idx { 0 };
	float out_dist_sqr { 0 };
	nanoflann::KNNResultSet<float> result_set(num_results);
	result_set.init(&q_near_idx, &out_dist_sqr);

	Eigen::VectorXf v { q->getCoord() };
	std::vector<float> vec(&v[0], v.data()+v.cols()*v.rows());

	float *vec_c = &vec[0];
	kd_tree->findNeighbors(result_set, vec_c, nanoflann::SearchParams(10));
	vec_c = nullptr;
	return getState(q_near_idx);
}

// Get nearest state without using nanoflann library
std::shared_ptr<base::State> base::Tree::getNearestState2(const std::shared_ptr<base::State> q)
{
	size_t q_near_idx { 0 };
	float d { 0 }, d_min { INFINITY };
	bool is_out { false };
	Eigen::VectorXf q_temp {};

	for (size_t i = 0; i < states->size(); i++)
	{
		q_temp = getState(i)->getCoord();
		is_out = false;
		for (size_t k = 0; k < q->getNumDimensions(); k++)
		{
			if (abs(q_temp(k) - q->getCoord(k)) > d_min)	// Is outside the box?
			{
				is_out = true;
				break;
			}
		}
		if (!is_out)	// Is inside the box?
		{
			d = (q_temp - q->getCoord()).norm();
			if (d < d_min)
			{
				q_near_idx = i;
				d_min = d;
			}
		}
	}
	return getState(q_near_idx);
}

// 'q_new' - new state added to tree
// 'q_parent' - parent of 'q_new'
void base::Tree::upgradeTree(const std::shared_ptr<base::State> q_new, const std::shared_ptr<base::State> q_parent)
{
	size_t N { states->size() };
	states->emplace_back(q_new);
	kd_tree->addPoints(N, N); 	// Comment this line if you are not using Kd-Trees
	q_new->setTreeIdx(getTreeIdx());
	q_new->setIdx(N);
	q_new->setParent(q_parent);
	if (q_parent != nullptr)
		q_parent->addChild(q_new);
}

// 'q_new' - new state added to tree
// 'q_parent' - parent of 'q_new'
// 'q_ref' - referent state containing useful distance information that are copied to 'q_new'
void base::Tree::upgradeTree(const std::shared_ptr<base::State> q_new, const std::shared_ptr<base::State> q_parent, 
							 const std::shared_ptr<base::State> q_ref)
{
	upgradeTree(q_new, q_parent);
	q_new->setDistance(q_ref->getDistance());
	q_new->setDistanceProfile(q_ref->getDistanceProfile());
	q_new->setIsRealDistance(q_ref->getIsRealDistance());
	q_new->setNearestPoints(q_ref->getNearestPoints());
}

namespace base 
{
	std::ostream &operator<<(std::ostream &os, const Tree &tree)
	{
		os << "Tree: " << tree.getTreeName() << std::endl;
		for (size_t i = 0; i < tree.getNumStates(); i++)
			os << tree.getState(i) << std::endl;
			
		return os;
	}
}