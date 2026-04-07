#include "simple_math.h"
#include <gtest/gtest.h>

TEST(SimpleMathTest, AddWorks) {
    EXPECT_EQ(simple_math::add(1, 2), 3);
    EXPECT_EQ(simple_math::add(-5, 5), 0);
    EXPECT_EQ(simple_math::add(0, 0), 0);
}

TEST(SimpleMathTest, MultiplyWorks) {
    EXPECT_EQ(simple_math::multiply(3, 4), 12);
    EXPECT_EQ(simple_math::multiply(-2, 5), -10);
    EXPECT_EQ(simple_math::multiply(0, 10), 0);
}

TEST(SimpleMathTest, IsPrimeWorks) {
    EXPECT_FALSE(simple_math::isPrime(-1));
    EXPECT_FALSE(simple_math::isPrime(0));
    EXPECT_FALSE(simple_math::isPrime(1));
    EXPECT_TRUE(simple_math::isPrime(2));
    EXPECT_TRUE(simple_math::isPrime(3));
    EXPECT_FALSE(simple_math::isPrime(4));
    EXPECT_TRUE(simple_math::isPrime(13));
    EXPECT_FALSE(simple_math::isPrime(21));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
