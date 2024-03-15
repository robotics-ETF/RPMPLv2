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

TEST(VectorFTest, testAddition1)
{
    base::VectorF vec1{1.0f, 2.0f, 3.0f};
    base::VectorF vec2{4.0f, 5.0f, 6.0f};

    base::VectorF vec = vec1 + vec2; 

    ASSERT_EQ(vec(0), 5.0f);
    ASSERT_EQ(vec(1), 7.0f);
    ASSERT_EQ(vec(2), 9.0f);
    
    ASSERT_EQ(vec.size(), 3);
}

TEST(VectorFTest, testZeroOne)
{
    constexpr size_t dim{4};
    base::VectorF vec1 = base::VectorF::zeros(dim);
    base::VectorF vec2 = base::VectorF::ones(dim);

    ASSERT_EQ(vec1(0), 0.0f);
    ASSERT_EQ(vec2(0), 1.0f);
    
    ASSERT_EQ(vec1.size(), dim);
    ASSERT_EQ(vec2.size(), dim);
}

TEST(VectorFTest, testAddition2)
{
    base::VectorF vec1{1.0f, 2.0f, 3.0f};
    base::VectorF vec2{4.0f, 5.0f, 6.0f};

    vec1 += vec2;

    ASSERT_EQ(vec1(0), 5.0f);
    ASSERT_EQ(vec1(1), 7.0f);
    ASSERT_EQ(vec1(2), 9.0f);
    
    ASSERT_EQ(vec1.size(), 3);
}

TEST(VectorFTest, testAdditionWithScalar1)
{
    base::VectorF vec1{1.0f, 2.0f, 3.0f};
    constexpr const float val{1.0f};

    base::VectorF vec = vec1 + val; 

    ASSERT_EQ(vec(0), 2.0f);
    ASSERT_EQ(vec(1), 3.0f);
    ASSERT_EQ(vec(2), 4.0f);
    
    ASSERT_EQ(vec.size(), 3);
}

TEST(VectorFTest, testAdditionWithScalar2)
{
    base::VectorF vec1{1.0f, 2.0f, 3.0f};
    constexpr float val{1.0f};

    vec1 += val; 

    ASSERT_EQ(vec1(0), 2.0f);
    ASSERT_EQ(vec1(1), 3.0f);
    ASSERT_EQ(vec1(2), 4.0f);
    
    ASSERT_EQ(vec1.size(), 3);
}



int main(int argc, char **argv) 
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}