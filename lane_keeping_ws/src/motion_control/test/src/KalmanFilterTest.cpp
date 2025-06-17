#include "KalmanFilter.hpp"
#include <gtest/gtest.h>

class KalmanFilterTest : public testing::Test
{
    public:
        KalmanFilterTest(){};

    protected:
        KalmanFilter filter{0.1, 0.4};
};

TEST_F(KalmanFilterTest, InitializesProperly)
{
    double result = filter.update(320);
    ASSERT_NEAR(result, 320, 0);
}
