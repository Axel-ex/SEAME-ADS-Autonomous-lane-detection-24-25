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
        "raw_img", 1,
        [this](sensor_msgs::msg::Image::SharedPtr img)
        { rawImageCallback(img); });

    lane_pos_pub_ =
        create_publisher<lane_msgs::msg::LanePositions>("lane_positions", 10);

    canny_edge_detector_ =
        cv::cuda::createCannyEdgeDetector(LOW_CANNY, HIGH_CANNY);
    hough_segment_detector_ = cv::cuda::createHoughSegmentDetector(
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
    publishRawOutput(output);
    // cv::cuda::GpuMat gpu_img(OUTPUT_IMG_SIZE.height, OUTPUT_IMG_SIZE.width,
    //                          CV_32FC1, output.data());
    // postProcessing(gpu_img);

    // TODO:
    // 1) Make the output a gpu image
    // 2) Post process
    // 3) extract lines
    // 4) Publish lane position
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
    // morpho transformation
    // apply canny edge
}

void MlVisionNode::publishRawOutput(std::vector<float>& output) const
{
    cv::Mat output_img(OUTPUT_IMG_SIZE.height, OUTPUT_IMG_SIZE.height, CV_32FC1,
                       output.data());
    cv::normalize(output_img, output_img, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    sensor_msgs::msg::Image mask_msg;
    std_msgs::msg::Header header;
    auto msg = cv_bridge::CvImage(header, "mono8", output_img);
}
