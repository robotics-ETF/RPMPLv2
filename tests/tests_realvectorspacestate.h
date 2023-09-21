//
// Created by dinko on 28.5.21..
//
#include "RealVectorSpaceState.h"
#include <Eigen/Dense>


TEST(RealVectorSpaceStateTest, testConstructor)
{
    std::shared_ptr<base::State> ss = std::make_shared<base::RealVectorSpaceState>(Eigen::Vector2f({1, 2}));

    ASSERT_EQ(ss->getNumDimensions(), 2);
    ASSERT_EQ(ss->getCoord(), Eigen::Vector2f({1, 2}));
}

TEST(RealVectorSpaceStateTest, testConstructor1)
{
    Eigen::VectorXf state_coord(6);
    state_coord << 0, 0, 0, 0, 0, 0;
    std::shared_ptr<base::State> ss = std::make_shared<base::RealVectorSpaceState>(state_coord);

    ASSERT_EQ(ss->getNumDimensions(), 2);
    ASSERT_EQ(ss->getCoord(), state_coord);
}
