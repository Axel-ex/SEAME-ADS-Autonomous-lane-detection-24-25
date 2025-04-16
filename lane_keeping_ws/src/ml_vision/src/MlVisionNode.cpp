#include <Logger.hpp>
#include <MlVisionNode.hpp>
#include <fstream>

MlVisionNode::MlVisionNode() : rclcpp::Node("ml_vision_node")
{
    raw_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "raw_img", 1,
        [this](sensor_msgs::msg::Image::SharedPtr img)
        { this->rawImageCallback(img); });

    lane_pos_pub_ = this->create_publisher<lane_msgs::msg::LanePositions>(
        "lane_positions", 10);
}

bool MlVisionNode::init()
{
    Logger logger;

    runtime_.reset(createInferRuntime(logger));
    if (!runtime_)
    {
        RCLCPP_ERROR(this->get_logger(), "Fail creating runtime");
        return false;
    }

    engine_.reset(createCudaEngine());
    if (!engine_)
    {
        RCLCPP_ERROR(this->get_logger(), "Fail creating engine");
        return false;
    }

    context_.reset(engine_->createExecutionContext());
    if (!context_)
    {
        RCLCPP_ERROR(this->get_logger(), "Fail creating context");
        return false;
    }

    return true;
}

ICudaEngine* MlVisionNode::createCudaEngine()
{
    std::ifstream infile(engine_path, std::ios::binary);
    if (!infile)
    {
        RCLCPP_ERROR(this->get_logger(), "Couldnt open engine file");
        return nullptr;
    }
    RCLCPP_INFO(this->get_logger(), "Engine file loaded");

    infile.seekg(0, std::ios::end);
    auto size = infile.tellg();
    infile.seekg(0, std::ios::beg);

    std::vector<char> engine_data(size);
    infile.read(engine_data.data(), size);
    return runtime_->deserializeCudaEngine(engine_data.data(), size);
}

void MlVisionNode::rawImageCallback(sensor_msgs::msg::Image::SharedPtr img) {}
