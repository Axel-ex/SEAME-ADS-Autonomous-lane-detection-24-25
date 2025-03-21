#pragma once

/**
 * @class KalmanFilter
 * @brief 1d Kalman filter.
 *
 * This filter helps filter out absurd measurements of lane center, typically
 * occuring when our algorithm fails to detect properly the lanes.
 *
 */
class KalmanFilter
{
    public:
        KalmanFilter(double process_variance_, double measurement_variance);
        ~KalmanFilter() = default;

        double update(double lane_center);

    private:
        double x_; // Estimated position
        double p_; // Estimate uncertainty
        double q_; // Process noise variance
        double r_; // Measurement noise variance
        double k_; // Kalman gain
};
