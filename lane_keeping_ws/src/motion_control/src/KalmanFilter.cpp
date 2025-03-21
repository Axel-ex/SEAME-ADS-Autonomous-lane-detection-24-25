#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter(double process_variance_,
                           double measurement_variance)
    : x_(320), p_(1.0), k_(0)
{
    q_ = process_variance_;
    r_ = measurement_variance;
}

/**
 * @brief Stabilize the measurements of lane center
 *
 * the filtering is done in steps:
 * - prediction: increase uncertainty by adding process noise
 * - Calculate kalman gain: How much do we trust the prediction
 * - correct the measurement with uncertainty and gain.
 * - Update uncertainty: the closer prediction and measurement are the smaller
 * this value gets
 *
 * @param lane_center
 * @return
 */
double KalmanFilter::update(double lane_center)
{
    // Predict
    p_ = p_ + q_;

    // Kalman gain
    k_ = p_ / (p_ + r_);

    // Update the estimate with the measurement
    x_ = x_ + k_ * (lane_center - x_);

    // Update uncertainty
    p_ = (1 - k_) * p_;

    return x_;
}
