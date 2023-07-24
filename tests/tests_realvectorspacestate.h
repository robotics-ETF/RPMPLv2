//
// Created by dinko on 28.5.21..
//
#include "RealVectorSpaceState.h"
#include <Eigen/Dense>


TEST(RealVectorSpaceStateTest, testConstructor)
{
std::shared_ptr<base::State> ss = std::make_shared<base::RealVectorSpaceState>(2);

ASSERT_EQ(ss->getDimensions(), 2);
}

TEST(RealVectorSpaceStateTest, testConstructor1)
{
std::shared_ptr<base::State> ss = std::make_shared<base::RealVectorSpaceState>(Eigen::Vector2f({1,2}));

ASSERT_EQ(ss->getDimensions(), 2);
ASSERT_EQ(ss->getCoord(), Eigen::Vector2f({1,2}));
}
