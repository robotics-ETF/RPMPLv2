#include <gtest/gtest.h>
#include "Vector.h"
#include <Eigen/Dense>


TEST(VectorFTest, testConstructor)
{
    base::VectorF vec = {1.0f, 2.0f, 3.0f};

    ASSERT_EQ(vec(0), 1.0f);
    ASSERT_EQ(vec(1), 2.0f);
    ASSERT_EQ(vec(2), 3.0f);
    
    std::cout << vec.size() << "\n";
    ASSERT_EQ(vec.size(), 3);
}

int main(int argc, char **argv) 
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}