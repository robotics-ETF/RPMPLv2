#include "StateSpace.h"

base::StateSpace::StateSpace()
{
    robot = nullptr;
    env = nullptr;
}

base::StateSpace::StateSpace(size_t num_dimensions_)
{
    num_dimensions = num_dimensions_;
    robot = nullptr;
    env = nullptr;
}

base::StateSpace::StateSpace(size_t num_dimensions_, std::shared_ptr<robots::AbstractRobot> robot_, std::shared_ptr<env::Environment> env_)
{
    num_dimensions = num_dimensions_;
    robot = robot_;
    env = env_;
}

base::StateSpace::~StateSpace() {}