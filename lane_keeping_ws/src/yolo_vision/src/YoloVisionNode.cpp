#include <Logger.hpp>
#include <YoloVisionNode.hpp>
#include <cv_bridge/cv_bridge.h>

/**
 * @brief Initialize ROS subscriber and publisher as well as OpenCV objects used
 * for post processing of inference output
 */
YoloVisionNode::YoloVisionNode() : rclcpp::Node("ml_vision_node")
{
    raw_img_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "image_raw", 1,
        [this](sensor_msgs::msg::Image::SharedPtr img)
        { rawImageCallback(img); });

    lane_pos_pub_ =
        create_publisher<lane_msgs::msg::LanePositions>("lane_positions", 10);
}

/**
 * @brief Initializes inference engine, image processor, and debug publishers.
 * @return True if successful.
 */
bool YoloVisionNode::init()
{
    inference_engine_ = std::make_unique<InferenceEngine>(shared_from_this());
    inference_engine_->init();
    image_processor_ =
        std::make_unique<ImageProcessor>(INPUT_IMG_SIZE, OUTPUT_IMG_SIZE);

    // For debug purpose
    image_transport::ImageTransport it(shared_from_this());
    processed_img_pub_ = it.advertise("processed_img", 1);

    RCLCPP_INFO(get_logger(), "YoloVisionNode initiated.");

    return true;
}

/**
 * @brief Callback for raw camera image subscription.
 *
 * Handles image conversion, inference, and postprocessing.
 *
 * @param img_msg The incoming image message.
 */
void YoloVisionNode::rawImageCallback(
    sensor_msgs::msg::Image::SharedPtr img_msg)
{
    auto converted = cv_bridge::toCvShare(img_msg, img_msg->encoding);
    auto image = converted->image;
    if (image.empty())
    {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), LOG_FREQ,
                              "Error: received image is empty");
        return;
    }

    std::vector<float> input = image_processor_->flattenImage(image);
    bool status = inference_engine_->runInference(input);
    if (!status)
    {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), LOG_FREQ,
                              "Fail running inference");
        return;
    }

    float* gpu_data = inference_engine_->getOutputDevicePtr();
    cv::cuda::GpuMat gpu_img(OUTPUT_IMG_SIZE, CV_32FC1, gpu_data);
    cv::cuda::normalize(gpu_img, gpu_img, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    auto lines = image_processor_->getLines(gpu_img); // TODO: get objects
    publishLanePositions(lines); // TODO: publish object position
}

/**
 * @brief Publishes detected lane line segments as ROS message.
 *
 * @param lines Vector of detected lines (Vec4i format).
 */
void YoloVisionNode::publishLanePositions(std::vector<cv::Vec4i>& lines)
{
    lane_msgs::msg::LanePositions msg;
    msg.header.stamp = this->now();

    std::vector<cv::Vec4i> left_lines, right_lines;

    for (const auto& line : lines)
    {
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
        double slope = static_cast<double>(y2 - y1) / (x2 - x1);

        // remove horizontal lines
        if (std::abs(slope) < 0.3)
            continue;

        // Classify lines based on slope and position
        if (slope < 0 && x1 < (OUTPUT_IMG_SIZE.width / 2))
            left_lines.push_back(line);
        else if (slope > 0 && x1 > (OUTPUT_IMG_SIZE.width / 2))
            right_lines.push_back(line);
    }

    // Add left lane lines to the message
    for (auto& line : left_lines)
    {
        geometry_msgs::msg::Point32 p1, p2;
        p1.x = line[0];
        p1.y = line[1];
        p2.x = line[2];
        p2.y = line[3];
        msg.left_lane.insert(msg.left_lane.end(), {p1, p2});
    }

    // Add right lane lines to the message
    for (auto& line : right_lines)
    {
        geometry_msgs::msg::Point32 p1, p2;
        p1.x = line[0];
        p1.y = line[1];
        p2.x = line[2];
        p2.y = line[3];
        msg.right_lane.insert(msg.right_lane.end(), {p1, p2});
    }

    msg.image_width.data = OUTPUT_IMG_SIZE.width;
    msg.image_height.data = OUTPUT_IMG_SIZE.height;

    lane_pos_pub_->publish(msg);
}

/**
 * @brief Publishes debug image to the topic associated with the publisher
 * passed in argument.
 *
 * @param gpu_img The image to publish.
 * @param publisher The publisher associated with the debug topic.
 */
void YoloVisionNode::publishDebug(cv::cuda::GpuMat& gpu_img,
                                  image_transport::Publisher& publisher) const
{
    std_msgs::msg::Header header;
    cv::Mat cpu_img;
    gpu_img.download(cpu_img);
    auto msg = cv_bridge::CvImage(header, "mono8", cpu_img);
    publisher.publish(msg.toImageMsg());
}
