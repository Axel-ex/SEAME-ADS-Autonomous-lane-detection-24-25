#pragma once

#include <opencv2/opencv.hpp>

// NODE
constexpr int LOG_FREQ = 5000;
const cv::Size INPUT_IMG_SIZE(256, 256);
const cv::Size OUTPUT_IMG_SIZE(256, 256);

// INFERENCE ENGINE
constexpr auto ENGINE_PATH = "/home/axel/models/engines/unet_optimized.engine";
constexpr auto INPUT_LAYER_NAME = "input_1";
constexpr auto OUTPUT_LAYER_NAME = "conv2d_14";

// IMAGE TRANSFO
constexpr int LOW_CANNY = 50;
constexpr int HIGH_CANNY = 80;
constexpr float TRESHOLD = 190;
constexpr int MIN_LINE_LENGTH = 20;
constexpr int MAX_LINE_GAP = 10;
constexpr int MAX_DETECTED_LINE = 500;
constexpr int KERNEL_SIZE = 3;
