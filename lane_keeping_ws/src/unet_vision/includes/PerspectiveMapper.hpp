#pragma once

#include <opencv2/opencv.hpp>

class PerspectiveMapper
{
    public:
        PerspectiveMapper() = default;
        ~PerspectiveMapper() = default;

        void init(const cv::Size& input_size, const cv::Size& output_size);

    private:
        std::vector<cv::Point2f> og_points_;
        std::vector<cv::Point2f> transfo_points_;
};
