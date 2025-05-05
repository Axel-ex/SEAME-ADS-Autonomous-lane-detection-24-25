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

    YoloResult res = extractResult();
}

YoloResult YoloVisionNode::extractResult()
{
    std::vector<float> host_output(inference_engine_->getOuputSize() /
                                   sizeof(float));
    cudaMemcpy(host_output.data(), inference_engine_->getOutputDevicePtr(),
               inference_engine_->getOuputSize(), cudaMemcpyDeviceToHost);

    // Loop over the detections (Refer to check_bindings for this information)
    const int nb_elements = 25200;
    const int num_classes = 80;
    const int element_size = 5 + num_classes;

    float conf_threshold = 0.2;
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;

    for (int i = 0; i < nb_elements; i++)
    {
        const float* element = &host_output[i * element_size];
        float elem_conf = element[4];

        if (elem_conf < conf_threshold)
            continue;

        // Find class with max score
        float max_class_prob = 0.0f;
        int class_id = -1;
        for (int c = 0; c < num_classes; ++c)
        {
            if (element[5 + c] > max_class_prob)
            {
                max_class_prob = element[5 + c];
                class_id = c;
            }
        }

        float final_conf = elem_conf * max_class_prob;
        if (final_conf < conf_threshold)
            continue;

        // YOLO box format is center_x, center_y, width, height
        float cx = element[0];
        float cy = element[1];
        float w = element[2];
        float h = element[3];

        int left = static_cast<int>(cx - w / 2.0f);
        int top = static_cast<int>(cy - h / 2.0f);
        int width = static_cast<int>(w);
        int height = static_cast<int>(h);

        boxes.emplace_back(left, top, width, height);
        confidences.push_back(final_conf);
        class_ids.push_back(class_id);
    }

    // Filter with non maximum suppression (NMS)
    std::vector<int> indices;
    float nms_treshold = 0.45f;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_treshold,
                      indices);

    YoloResult result;

    for (int idx : indices)
    {
        result.boxes.push_back(boxes[idx]);
        result.confidences.push_back(confidences[idx]);
        result.class_ids.push_back(mapIdtoString(class_ids[idx]));
    }

    return result;
}

std::string YoloVisionNode::mapIdtoString(int id)
{
    if (id >= 80)
        return "Invalid id";
    return COCO_CLASSES[id];
}

//========================================================================================
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
