#pragma once

/**
 * @class KalmanFilter
 * @brief 1d Kalman filter.
 *
 * this filter helps smooth out noisy lane center measurements. If a single
 * camera frame has an outlier measurement,the filter will "trust" this
 * measurement less than the prediction based on previous frames, resulting in
 * smoother control.
 *
 * process_variance / q_ : How much true lane center might change between frames
 * measurement_variance / r_: How noisy our lane detection is
 *
 * higher r_: More smoothing, slower response
 * higher q_: faster response, less smoothing
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
