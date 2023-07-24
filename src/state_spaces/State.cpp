//
// Created by dinko on 7.3.21.
// Modified by nermin on 18.02.22.
//

#include "State.h"

base::State::State()
{
	tree_idx = -1;
	idx = -1;
	d_c = -1;
	cost = -1;
	planes = nullptr;
	parent = nullptr;
	children = std::make_shared<std::vector<std::shared_ptr<base::State>>>();
}

base::State::~State() {}

void base::State::addChild(std::shared_ptr<base::State> child)
{
	children->emplace_back(child);
}

std::ostream &base::operator<<(std::ostream &os, const base::State *state)
{
	if (state->getParent() == nullptr)
		os << "q: (" << state->getCoord().transpose() << "); parent q: NONE";
	else
		os << "q: (" << state->getCoord().transpose() << "); parent q: (" << state->getParent()->getCoord().transpose() << ")";
	return os;
}