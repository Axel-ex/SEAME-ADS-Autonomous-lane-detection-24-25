#pragma once

#include <opencv2/opencv.hpp>

class PerspectiveMapper
{
    public:
        PerspectiveMapper(cv::Size& input_size, cv::Size& output_size);
        ~PerspectiveMapper();

        void transformPerspective(cv::Mat& input);

    private:
        cv::Size input_size_;
        cv::Size output_size_;
        std::vector<cv::Point2f> og_points_;
        std::vector<cv::Point2f> transformed_points_;
        cv::Mat perspective_mat_;
        cv::Mat inverse_perspective_mat_;
};
