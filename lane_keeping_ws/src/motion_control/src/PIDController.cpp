#include "../includes/PIDController.hpp"

PIDController::PIDController(float kp, float ki, float kd)
    : kp_(kp), ki_(ki), kd_(kd)
{
}

PIDController::PIDController() : kp_(0.5), ki_(0.1), kd_(0.2) {}

void PIDController::initializePID(float kp, float ki, float kd)
{
    kp_ = kp;
    kd_ = kd;
    ki_ = ki;
}

void PIDController::calculateSteering()
{
    // TODO:
    // 1- calculate vehicle position and orientation
    // 2- find centerline at the lookahead distance
    // 3- calculate lateral error
}
