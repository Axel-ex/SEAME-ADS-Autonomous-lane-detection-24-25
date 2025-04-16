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

/**
 * @brief Initialize inference runtime, load engine and create context
 *
 * @return
 */
bool MlVisionNode::init()
{
    Logger logger;

    runtime_.reset(createInferRuntime(logger));
    if (!runtime_)
    {
        RCLCPP_ERROR(this->get_logger(), "Fail creating runtime");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Runtime succefully created");

    engine_.reset(createCudaEngine());
    if (!engine_)
    {
        RCLCPP_ERROR(this->get_logger(), "Fail creating engine");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "engine succefully created");

    context_.reset(engine_->createExecutionContext());
    if (!context_)
    {
        RCLCPP_ERROR(this->get_logger(), "Fail creating context");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "context succefully created");

    allocateDevices();
    if (!d_input_ || !d_output_)
        return false;

    return true;
}

/**
 * @brief Create engine for inference
 *
 * Reads the file found at @engine_path byte by byte and deserialize the data
 * using the runtime
 *
 * @return
 */
ICudaEngine* MlVisionNode::createCudaEngine()
{
    std::ifstream infile(ENGINE_PATH, std::ios::binary);
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

/**
 * @brief allocate memory for input and output device
 */
void MlVisionNode::allocateDevices()
{
    const int input_index = engine_->getBindingIndex(INPUT_LAYER_NAME);
    const int output_index = engine_->getBindingIndex(OUTPUT_LAYER_NAME);

    Dims input_dims = engine_->getBindingDimensions(input_index);
    Dims output_dims = engine_->getBindingDimensions(output_index);

    // WARN: make sure these match the expected input / output of the model
    size_t input_size = 1;
    for (int i = 0; i < input_dims.nbDims; i++)
        input_size *= input_dims.d[i];
    input_size *= sizeof(float);

    size_t output_size = 1;
    for (int i = 0; i < output_dims.nbDims; i++)
        output_size *= output_dims.d[i];
    output_size *= sizeof(float);

    // allocate memory and transfer ownership to our smartpointer
    void* raw_input_ptr = nullptr;
    void* raw_output_ptr = nullptr;

    cudaError_t input_err = cudaMalloc(&raw_input_ptr, input_size);
    cudaError_t output_err = cudaMalloc(&raw_output_ptr, output_size);
    if (input_err != cudaSuccess && output_err != cudaSuccess)
        RCLCPP_ERROR(this->get_logger(),
                     "An error occured while allocating for input / output "
                     "device: input: %s, output: %s",
                     cudaGetErrorString(input_err),
                     cudaGetErrorString(output_err));

    d_input_.reset(raw_input_ptr);
    d_output_.reset(raw_output_ptr);
}

/**
 * @brief Callback upon receving a new frame from the Camera node
 *
 * @param img
 */
void MlVisionNode::rawImageCallback(sensor_msgs::msg::Image::SharedPtr img_msg)
{
    auto converted = cv_bridge::toCvShare(img_msg, img_msg->encoding);
    if (converted->image.empty())
    {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), LOG_FREQ,
                              "Error: received image is empty");
        return;
    }
    auto flat_img = flattenImage(converted);
    // TODO: Perform inference
}

std::vector<float>
MlVisionNode::flattenImage(cv_bridge::CvImageConstPtr img_ptr)
{
    cv::Mat img = img_ptr->image;
    cv::resize(img, img, INPUT_SIZE);
    std::vector<float> flatten_img(INPUT_SIZE.height * INPUT_SIZE.width * 3);

    for (int y = 0; y < img.rows; y++)
    {
        for (int x = 0; x < img.cols; x++)
        {
            cv::Vec3b pixel = img.at<cv::Vec3b>(x, y);
            flatten_img[(y * img.cols + x) * 3] =
                static_cast<float>(pixel[2] / 255.0);
            flatten_img[(y * img.cols + x) * 3 + 1] =
                static_cast<float>(pixel[1] / 255.0);
            flatten_img[(y * img.cols + x) * 3 + 2] =
                static_cast<float>(pixel[0] / 255.0);
        }
    }
    return flatten_img;
}
