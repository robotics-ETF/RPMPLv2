#include "State.h"

base::State::State(const Eigen::VectorXf &coord_)
{
	coord = coord_;
	num_dimensions = coord.size();
	tree_idx = 0;
	idx = 0;
	d_c = -1;
	d_c_profile = std::vector<float>();
	is_real_d_c = true;
	cost = -1;
	nearest_points = nullptr;
	parent = nullptr;
	children = std::make_shared<std::vector<std::shared_ptr<base::State>>>();
	frames = nullptr;
	skeleton = nullptr;
	enclosing_radii = nullptr;
}

base::State::~State() {}

void base::State::addChild(const std::shared_ptr<base::State> child)
{
	children->emplace_back(child);
}

void base::State::addNeighbourState(const std::shared_ptr<State> neighbour)
{
	// Check if the neighbour already exists to avoid duplicates
	auto it = std::find(neighbours->begin(), neighbours->end(), neighbour);
	if (it == neighbours->end()) {
		// Neighbour not found, add it
		neighbours->push_back(neighbour);
	}
}

void base::State::clearNeighbourStates()
{
	// Clear all neighbours
	neighbours->clear();
}

void base::State::removeChild(const std::shared_ptr<State> child)
{
	// Find the child in the children vector
	auto it = std::find(children->begin(), children->end(), child);
	if (it != children->end()) {
		// Child found, remove it
		children->erase(it);
	}
}

namespace base 
{
	std::ostream &operator<<(std::ostream &os, const std::shared_ptr<base::State> state)
	{
		if (state->getParent() == nullptr)
			os << "q: (" << state->getCoord().transpose() << "); parent q: NONE";
		else
			os << "q: (" << state->getCoord().transpose() << "); parent q: (" << state->getParent()->getCoord().transpose() << ")";
		return os;
	}
}