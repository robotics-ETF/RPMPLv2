//
// Created by nermin on 18.02.22.
//

#include "Tree.h"

base::Tree::Tree(std::string tree_name_, uint tree_idx_)
{
	tree_name = tree_name_;
	tree_idx = tree_idx_;
	states = std::make_shared<std::vector<std::shared_ptr<base::State>>>();
	kd_tree = nullptr;
}

base::Tree::Tree(std::shared_ptr<std::vector<std::shared_ptr<base::State>>> states_)
{
	states = states_;
	kd_tree = nullptr;
}

void base::Tree::clearTree()
{
	states->clear();
}

std::shared_ptr<base::State> base::Tree::getNearestState(std::shared_ptr<base::State> q)
{
	const size_t num_results = 1;
	size_t q_near_idx;
	float out_dist_sqr;
	nanoflann::KNNResultSet<float> result_set(num_results);
	result_set.init(&q_near_idx, &out_dist_sqr);

	Eigen::VectorXf v = q->getCoord();
	std::vector<float> vec(&v[0], v.data()+v.cols()*v.rows());

	float *vec_c = &vec[0];
	kd_tree->findNeighbors(result_set, vec_c, nanoflann::SearchParameters(10));
	vec_c = nullptr;
	return states->at(q_near_idx);
}

// Get nearest state without using nanoflann library
std::shared_ptr<base::State> base::Tree::getNearestStateV2(std::shared_ptr<base::State> q)
{
	int q_near_idx = 0;
	float d, d_min = INFINITY;
	bool is_out;
	Eigen::VectorXf q_temp;

	for (int i = 0; i < states->size(); i++)
	{
		q_temp = getState(i)->getCoord();
		is_out = false;
		for (int k = 0; k < q->getNumDimensions(); k++)
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

// 'q_new' - new node added to tree
// 'q_parent' - parent of 'q_new'
// 'd_c' - distance-to-obstacles for 'q_new'
// 'nearest_points' - nearest_points for 'q_new'
// 'cost' - cost-to-come for 'q_new'
void base::Tree::upgradeTree(std::shared_ptr<base::State> q_new, std::shared_ptr<base::State> q_parent,
							 float d_c, std::shared_ptr<std::vector<Eigen::MatrixXf>> nearest_points, float cost)
{
	size_t N = states->size();
	states->emplace_back(q_new);
	kd_tree->addPoints(N, N); 	// Comment this line if you are not using Kd-Trees
	q_new->setTreeIdx(getTreeIdx());
	q_new->setIdx(N);
	q_new->setParent(q_parent);
	q_new->setDistance(d_c);
	q_new->setNearestPoints(nearest_points);
	q_new->setCost(cost);
	if (q_parent != nullptr)
		q_parent->addChild(q_new);
}

std::ostream &base::operator<<(std::ostream &os, const Tree &tree)
{
	os << "Tree: " << tree.getTreeName() << std::endl;
	for (int i = 0; i < tree.getNumStates(); ++i)
		os << tree.getState(i) << std::endl;
		
	return os;
}