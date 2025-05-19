#pragma once

#include <opencv2/opencv.hpp>

// INFERENCE ENGINE
constexpr auto ENGINE_PATH = "/home/axel/models/engines/unet_optimized.engine";
constexpr auto INPUT_LAYER_NAME = "input_1";
constexpr auto OUTPUT_LAYER_NAME = "conv2d_14";

// PERSPECTIVE MAPPER
/**
 * @class CameraConfig
 * @brief Holds configuration needed for perspective mapping
 *
 */
struct CameraConfig
{
        float cameraHeight;  // in meters
        float cameraPitch;   // in degrees
        float horizontalFOV; // in degrees
        float verticalFOV;   // in degrees
        float nearDistance;  // meters from camera
        float farDistance;   // meters from camera
        float laneWidth;     // in meters
};

constexpr CameraConfig CAMERA_CONFIG = {0, 0, 0, 0, 0, 0, 0};

// define points of the trapezoid to project
const std::array<cv::Point2f, 4> OG_POINTS = {
    cv::Point2f{0.0f, 0.0f}, cv::Point2f{1.0f, 0.0f}, cv::Point2f{1.0f, 1.0f},
    cv::Point2f{0.0f, 1.0f}};

// Final points
const std::array<cv::Point2f, 4> TRANSFORM_POINTS = {
    cv::Point2f{0.0f, 0.0f}, cv::Point2f{1.0f, 0.0f}, cv::Point2f{1.0f, 1.0f},
    cv::Point2f{0.0f, 1.0f}};
