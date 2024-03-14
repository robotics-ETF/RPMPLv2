#include <gtest/gtest.h>
#include "RealVectorSpaceState.h"
#include <Eigen/Dense>


TEST(RealVectorSpaceStateTest, testConstructor)
{
    std::shared_ptr<base::State> q = std::make_shared<base::RealVectorSpaceState>(Eigen::Vector2f({1, 2}));

    ASSERT_EQ(q->getNumDimensions(), 2);
    ASSERT_EQ(q->getCoord(), Eigen::Vector2f({1, 2}));
}

TEST(RealVectorSpaceStateTest, testConstructor1)
{
    Eigen::VectorXf state_coord(6);
    state_coord << 0, 0, 0, 0, 0, 0;
    std::shared_ptr<base::State> q = std::make_shared<base::RealVectorSpaceState>(state_coord);

    ASSERT_EQ(q->getNumDimensions(), 6);
    ASSERT_EQ(q->getCoord(), state_coord);
}



int main(int argc, char **argv) 
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}