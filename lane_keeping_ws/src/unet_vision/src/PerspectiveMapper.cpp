#include <PerspectiveMapper.hpp>
#include <opencv2/cudawarping.hpp>

PerspectiveMapper::PerspectiveMapper(const cv::Size& input_size,
                                     const cv::Size& output_size)
    : input_size_(input_size), output_size_(output_size)
{
    float width = static_cast<float>(input_size.width);
    float height = static_cast<float>(input_size.height);

    // Source points - trapezoid on the original image
    og_points_ = {
        cv::Point2f(width * 0.15f, height * 0.55f), // Top-left
        cv::Point2f(width * 0.85f, height * 0.55f), // Top-right
        cv::Point2f(width * 1.0f, height * 0.95f),  // Bottom-right
        cv::Point2f(width * 0.0f, height * 0.95f)   // Bottom-left
    };

    // Destination points - rectangle in bird's eye view
    transformed_points_ = {
        cv::Point2f(0, 0),                                  // Top-left
        cv::Point2f(output_size.width, 0),                  // Top-right
        cv::Point2f(output_size.width, output_size.height), // Bottom-right
        cv::Point2f(0, output_size.height)                  // Bottom-left
    };

    // Calculate perspective transform matrix
    perspective_mat_ =
        cv::getPerspectiveTransform(og_points_, transformed_points_);
    inverse_perspective_mat_ =
        cv::getPerspectiveTransform(transformed_points_, og_points_);
}

cv::cuda::GpuMat
PerspectiveMapper::applyPerspectiveTransform(cv::cuda::GpuMat& input)
{

    // Check if input size matches
    if (input.size() != input_size_)
    {
        std::cerr << "Input size doesn't match initialization size"
                  << std::endl;
        cv::resize(input, input, input_size_);
    }

    cv::cuda::GpuMat warped;
    cv::cuda::warpPerspective(input, warped, perspective_mat_, input_size_,
                              cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    return warped;
}
