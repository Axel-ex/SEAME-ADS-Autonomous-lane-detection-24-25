#include "PerspectiveMapper.hpp"

void PerspectiveMapper::init(const cv::Size& input_size,
                             const cv::Size& output_size)
{
    float width = static_cast<float>(input_size.width);
    float height = static_cast<float>(input_size.height);

    // Source points - trapezoid on the original image
    og_points_ = {
        cv::Point2f(width * 0.35f, height * 0.65f), // Top-left
        cv::Point2f(width * 0.65f, height * 0.65f), // Top-right
        cv::Point2f(width * 0.9f, height * 0.95f),  // Bottom-right
        cv::Point2f(width * 0.1f, height * 0.95f)   // Bottom-left
    };

    // Destination points - rectangle in bird's eye view
    transfo_points_ = {
        cv::Point2f(0, 0),                                  // Top-left
        cv::Point2f(output_size.width, 0),                  // Top-right
        cv::Point2f(output_size.width, output_size.height), // Bottom-right
        cv::Point2f(0, output_size.height)                  // Bottom-left
    };
}
