#include <gtest/gtest.h>
#include "Vector.h"
#include <Eigen/Dense>


TEST(VectorFTest, testConstructor)
{
    base::VectorF vec{1.0f, 2.0f, 3.0f};

    ASSERT_EQ(vec(0), 1.0f);
    ASSERT_EQ(vec(1), 2.0f);
    ASSERT_EQ(vec(2), 3.0f);
    
    ASSERT_EQ(vec.size(), 3);
}

TEST(VectorFTest, testAddition)
{
    base::VectorF vec1{1.0f, 2.0f, 3.0f};
    base::VectorF vec2{4.0f, 5.0f, 6.0f};

    base::VectorF vec = vec1 + vec2; 

    ASSERT_EQ(vec(0), 5.0f);
    ASSERT_EQ(vec(1), 7.0f);
    ASSERT_EQ(vec(2), 9.0f);
    
    ASSERT_EQ(vec.size(), 3);
}



int main(int argc, char **argv) 
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}