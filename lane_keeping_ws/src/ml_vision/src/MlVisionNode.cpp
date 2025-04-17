#include <Logger.hpp>
#include <MlVisionNode.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>

/**
 * @brief Initialize ROS subscriber and publisher as well as OpenCV objects used
 * for post processing of inference output
 */
MlVisionNode::MlVisionNode() : rclcpp::Node("ml_vision_node")
{
    raw_img_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "image_raw", 1,
        [this](sensor_msgs::msg::Image::SharedPtr img)
        { rawImageCallback(img); });

    lane_pos_pub_ =
        create_publisher<lane_msgs::msg::LanePositions>("lane_positions", 10);

    canny_edge_detector_ =
        cv::cuda::createCannyEdgeDetector(LOW_CANNY, HIGH_CANNY);
    line_detector_ = cv::cuda::createHoughSegmentDetector(
        1.0, 180 / CV_PI, MIN_LINE_LENGTH, MAX_LINE_GAP, MAX_DETECTED_LINE);

    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(KERNEL_SIZE * 2 + 1, KERNEL_SIZE * 2 + 1),
        cv::Point(KERNEL_SIZE, KERNEL_SIZE));

    dilation_filter_ =
        cv::cuda::createMorphologyFilter(cv::MORPH_DILATE, CV_8UC1, kernel);
    erosion_filter_ =
        cv::cuda::createMorphologyFilter(cv::MORPH_ERODE, CV_8UC1, kernel);
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
        RCLCPP_ERROR(get_logger(), "Fail creating runtime");
        return false;
    }
    RCLCPP_INFO(get_logger(), "Runtime succefully created");

    engine_.reset(createCudaEngine());
    if (!engine_)
    {
        RCLCPP_ERROR(get_logger(), "Fail creating engine");
        return false;
    }
    RCLCPP_INFO(get_logger(), "engine succefully created");

    context_.reset(engine_->createExecutionContext());
    if (!context_)
    {
        RCLCPP_ERROR(get_logger(), "Fail creating context");
        return false;
    }
    RCLCPP_INFO(get_logger(), "context succefully created");

    allocateDevices();
    if (!d_input_ || !d_output_)
        return false;

    // For debug purpose
    image_transport::ImageTransport it(shared_from_this());
    edge_img_pub_ = it.advertise("edge_img", 1);
    raw_mask_pub_ = it.advertise("raw_mask", 1);
    processed_mask_pub_ = it.advertise("processed_mask", 1);

    return true;
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

    std::vector<float> input = flattenImage(converted);
    std::vector<float> output = runInference(input);

    if (output.empty())
    {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), LOG_FREQ,
                              "Fail running inference");
        return;
    }

    cv::Mat output_img(OUTPUT_IMG_SIZE.height, OUTPUT_IMG_SIZE.width, CV_32FC1,
                       output.data());
    cv::normalize(output_img, output_img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::cuda::GpuMat gpu_img;
    gpu_img.upload(output_img);

    postProcessing(gpu_img);
    // auto lines = getLines(gpu_img);
    // publishLanePositions(lines);
}

/**
 * @brief Create engine for inference
 *
 * Reads the file found at @engine_path byte by byte and deserialize the data
 * using the runtime.
 *
 * @return
 */
ICudaEngine* MlVisionNode::createCudaEngine()
{
    std::ifstream infile(ENGINE_PATH, std::ios::binary);
    if (!infile)
    {
        RCLCPP_ERROR(get_logger(), "Couldnt open engine file");
        return nullptr;
    }
    RCLCPP_INFO(get_logger(), "Engine file loaded");

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
    for (int i = 0; i < input_dims.nbDims; i++)
        input_size_ *= input_dims.d[i];
    input_size_ *= sizeof(float);

    for (int i = 0; i < output_dims.nbDims; i++)
        output_size_ *= output_dims.d[i];
    output_size_ *= sizeof(float);

    // allocate memory and transfer ownership to our smartpointer
    void* raw_input_ptr = nullptr;
    void* raw_output_ptr = nullptr;

    cudaError_t input_err = cudaMalloc(&raw_input_ptr, input_size_);
    cudaError_t output_err = cudaMalloc(&raw_output_ptr, output_size_);
    if (input_err != cudaSuccess || output_err != cudaSuccess)
        RCLCPP_ERROR(get_logger(),
                     "An error occured while allocating for input / output "
                     "device: input: %s, output: %s",
                     cudaGetErrorString(input_err),
                     cudaGetErrorString(output_err));

    d_input_.reset(raw_input_ptr);
    d_output_.reset(raw_output_ptr);
}

/**
 * @brief transform an Image into a flat vector (raw sequence of bytes)
 *
 * @param img_ptr
 * @return
 */
std::vector<float>
MlVisionNode::flattenImage(cv_bridge::CvImageConstPtr img_ptr) const
{
    cv::Mat img = img_ptr->image;
    cv::resize(img, img, INPUT_IMG_SIZE);
    std::vector<float> flatten_img(INPUT_IMG_SIZE.height *
                                   INPUT_IMG_SIZE.width * 3);

    for (int y = 0; y < img.rows; y++)
    {
        for (int x = 0; x < img.cols; x++)
        {
            cv::Vec3b pixel = img.at<cv::Vec3b>(y, x);
            size_t base = (y * img.cols + x) * 3;
            flatten_img[base + 0] = static_cast<float>(pixel[2] / 255.0);
            flatten_img[base + 1] = static_cast<float>(pixel[1] / 255.0);
            flatten_img[base + 2] = static_cast<float>(pixel[0] / 255.0);
        }
    }
    return flatten_img;
}

std::vector<float>
MlVisionNode::runInference(const std::vector<float>& flat_img) const
{
    // Transfer raw data to GPU
    cudaMemcpy(d_input_.get(), flat_img.data(), input_size_,
               cudaMemcpyHostToDevice);

    void* bindings[2] = {d_input_.get(), d_output_.get()};
    bool status = context_->executeV2(bindings);
    if (!status)
        return std::vector<float>();

    // Get back the result onto CPU
    std::vector<float> output(output_size_ / sizeof(float));
    cudaMemcpy(output.data(), d_output_.get(), output_size_,
               cudaMemcpyDeviceToHost);

    return output;
}

void MlVisionNode::postProcessing(cv::cuda::GpuMat& gpu_img)
{
    cv::cuda::threshold(gpu_img, gpu_img, TRESHOLD, 255, CV_THRESH_BINARY);
    dilation_filter_->apply(gpu_img, gpu_img);
    erosion_filter_->apply(gpu_img, gpu_img);
    publishDebug(gpu_img, raw_mask_pub_);
    canny_edge_detector_->detect(gpu_img, gpu_img);
    publishDebug(gpu_img, edge_img_pub_);
}

std::vector<cv::Vec4i> MlVisionNode::getLines(cv::cuda::GpuMat& gpu_img)
{

    cv::cuda::GpuMat gpu_lines;
    line_detector_->detect(gpu_img, gpu_lines);

    // Convert to vector of Vec4i
    std::vector<cv::Vec4i> lines;
    if (!gpu_lines.empty())
    {
        cv::Mat lines_cpu(1, gpu_lines.cols, CV_32SC4);
        gpu_lines.download(lines_cpu);
        lines.assign(lines_cpu.ptr<cv::Vec4i>(),
                     lines_cpu.ptr<cv::Vec4i>() + lines_cpu.cols);
    }

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), LOG_FREQ,
                          "Detected %zu valid lines", lines.size());
    return lines;
}

void MlVisionNode::publishLanePositions(std::vector<cv::Vec4i>& lines)
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

void MlVisionNode::publishDebug(cv::cuda::GpuMat& gpu_img,
                                image_transport::Publisher& publisher) const
{
    std_msgs::msg::Header header;
    cv::Mat cpu_img;
    gpu_img.download(cpu_img);
    auto msg = cv_bridge::CvImage(header, "mono8", cpu_img);
    publisher.publish(msg.toImageMsg());
}
