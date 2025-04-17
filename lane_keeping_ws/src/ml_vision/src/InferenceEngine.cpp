#include <InferenceEngine.hpp>
#include <Logger.hpp>
#include <fstream>

InferenceEngine::InferenceEngine(std::shared_ptr<rclcpp::Node> node_ptr)
    : node_ptr_(node_ptr)
{
}

bool InferenceEngine::init()
{
    Logger logger;
    runtime_.reset(createInferRuntime(logger));
    if (!runtime_)
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Fail creating runtime");
        return false;
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Runtime succefully created");

    engine_.reset(createCudaEngine());
    if (!engine_)
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Fail creating engine");
        return false;
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "engine succefully created");

    context_.reset(engine_->createExecutionContext());
    if (!context_)
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Fail creating context");
        return false;
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "context succefully created");

    allocateDevices();
    if (!d_input_ || !d_output_)
        return false;

    return true;
}

ICudaEngine* InferenceEngine::createCudaEngine()
{
    std::ifstream infile(ENGINE_PATH, std::ios::binary);
    if (!infile)
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Couldnt open engine file");
        return nullptr;
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Engine file loaded");

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
void InferenceEngine::allocateDevices()
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
        RCLCPP_ERROR(node_ptr_->get_logger(),
                     "An error occured while allocating for input / output "
                     "device: input: %s, output: %s",
                     cudaGetErrorString(input_err),
                     cudaGetErrorString(output_err));

    d_input_.reset(raw_input_ptr);
    d_output_.reset(raw_output_ptr);
}

/**
 * @brief Run inference.
 *
 * The result of the inference ends up on the output device.
 *
 * @param flat_img
 * @return
 */
bool InferenceEngine::runInference(const std::vector<float>& flat_img) const
{
    // Transfer raw data to GPU
    cudaMemcpy(d_input_.get(), flat_img.data(), input_size_,
               cudaMemcpyHostToDevice);

    void* bindings[2] = {d_input_.get(), d_output_.get()};
    bool status = context_->executeV2(bindings);
    return status;
}

/**
 * @brief get the underlying data of the output device.
 *
 * this function is usefull to get GPU data directly after running inference.
 * This way we can keep the data on the GPU until we finish processing it.
 *
 * @return
 */
float* InferenceEngine::getOutputDevicePtr() const
{
    return static_cast<float*>(d_output_.get());
}
