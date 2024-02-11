//
// Created by dinko on 7.3.21.
// Modified by nermin on 18.02.22.
//

#include "State.h"

base::State::State(const Eigen::VectorXf &coord_)
{
	coord = coord_;
	num_dimensions = coord.size();
	tree_idx = -1;
	idx = -1;
	d_c = -1;
	d_c_profile = std::vector<float>();
	is_real_d_c = true;
	cost = -1;
	nearest_points = nullptr;
	parent = nullptr;
	children = std::make_shared<std::vector<std::shared_ptr<base::State>>>();
}

base::State::~State() {}

void base::State::addChild(const std::shared_ptr<base::State> child)
{
	children->emplace_back(child);
}

std::ostream &base::operator<<(std::ostream &os, const std::shared_ptr<base::State> state)
{
	if (state->getParent() == nullptr)
		os << "q: (" << state->getCoord().transpose() << "); parent q: NONE";
	else
		os << "q: (" << state->getCoord().transpose() << "); parent q: (" << state->getParent()->getCoord().transpose() << ")";
	return os;
}