#include "KalmanFilter.hpp"
#include <gtest/gtest.h>

class KalmanFilterTest : public testing::Test
{
    public:
        KalmanFilterTest(){};

    protected:
        KalmanFilter filter{0.1, 0.4};
};

/**
 * @brief Test if the filter is initialized
 */
TEST_F(KalmanFilterTest, InitializesProperly)
{
    double result = filter.update(320);
    ASSERT_NEAR(result, 320, 0);
}

/**
 * @brief Test updating the estimate
 *
 * Adding a measurement to the filter  above its init value should return
 * a greater value than the intial one and a lower value than what we just
 * updated
 */
TEST_F(KalmanFilterTest, RespondsToNewMeasurements)
{
    double result = filter.update(330);

    EXPECT_GT(result, 320);
    EXPECT_LT(result, 330);
}

/**
 * @brief Test if the estimate converges with constant measurements
 */
TEST_F(KalmanFilterTest, ConvergesWithConstMeasurements)
{
    double result = 0;
    for (int i = 0; i < 10; i++)
        result = filter.update(330);
    EXPECT_NEAR(result, 330, 1);
}

/**
 * @brief Test for very noisy filter
 *
 * the filter should trust more the prediciton if the measurement vriance is
 * high
 */
TEST_F(KalmanFilterTest, HandlesNoisyMeasurements)
{
    KalmanFilter noisy_filter(1.0, 100);
    double result = noisy_filter.update(400);

    EXPECT_LT(result, 400);
    EXPECT_GT(result, 320);
}
