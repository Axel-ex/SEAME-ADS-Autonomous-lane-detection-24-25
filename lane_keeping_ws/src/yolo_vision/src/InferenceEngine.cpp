#include <InferenceEngine.hpp>
#include <Logger.hpp>
#include <fstream>

/**
 * @brief Construct a new InferenceEngine object.
 *
 * @param node_ptr Shared ROS2 node pointer used for logging and context.
 */
InferenceEngine::InferenceEngine(std::shared_ptr<rclcpp::Node> node_ptr)
    : node_ptr_(node_ptr)
{
}

/**
 * @brief Destruct InferenceEngine object
 *
 * the order of the destruction has to be specified to avoid any segfault
 */
InferenceEngine::~InferenceEngine()
{
    context_.reset();
    engine_.reset();
    runtime_.reset();
}

/**
 * @brief Initializes the inference engine: loads the serialized engine, creates
 * context, and allocates memory.
 *
 * @return true if all components were successfully initialized, false
 * otherwise.
 */
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
    RCLCPP_INFO(node_ptr_->get_logger(), "Engine succefully created");

    context_.reset(engine_->createExecutionContext());
    if (!context_)
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Fail creating context");
        return false;
    }
    RCLCPP_INFO(node_ptr_->get_logger(),
                "Execution context succefully created");

    allocateDevices();
    if (!d_input_ || !d_output_)
        return false;

    return true;
}

/**
 * @brief Deserializes the engine from a precompiled engine file.
 *
 * @return Pointer to the created ICudaEngine object.
 */
ICudaEngine* InferenceEngine::createCudaEngine()
{
    std::ifstream infile(ENGINE_PATH, std::ios::binary);
    if (!infile)
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Couldnt open engine file");
        return nullptr;
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Engine sucefully created");

    infile.seekg(0, std::ios::end);
    auto size = infile.tellg();
    infile.seekg(0, std::ios::beg);

    std::vector<char> engine_data(size);
    infile.read(engine_data.data(), size);
    return runtime_->deserializeCudaEngine(engine_data.data(), size);
}

/**
 * @brief Allocates CUDA device memory for input and output bindings.
 *
 * Uses dimensions from the engine's bindings and wraps raw allocations
 * in smart pointers with custom deleters.
 */
void InferenceEngine::allocateDevices()
{
    const int input_index = engine_->getBindingIndex(INPUT_LAYER_NAME);
    const int output_index = engine_->getBindingIndex(OUTPUT_LAYER_NAME);

    Dims input_dims = engine_->getBindingDimensions(input_index);
    Dims output_dims = engine_->getBindingDimensions(output_index);

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
        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "An error occured while allocating for input / output, device "
            "input: %s, device output: %s",
            cudaGetErrorString(input_err), cudaGetErrorString(output_err));

    d_input_.reset(raw_input_ptr);
    d_output_.reset(raw_output_ptr);
}

/**
 * @brief Runs inference on the GPU.
 *
 * The input image must be a flattened vector of floats. The result is stored on
 * the GPU, and can be retrieved via getOutputDevicePtr().
 *
 * @param flat_img Flattened input image.
 * @return true if inference executed successfully, false if it failed.
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
 * @brief Returns a pointer to the GPU memory containing inference results.
 *
 * This allows the post-processing pipeline to operate directly on GPU memory,
 * avoiding unnecessary memory copies.
 *
 * @return float* Pointer to device memory containing the output tensor.
 */
float* InferenceEngine::getOutputDevicePtr() const
{
    return static_cast<float*>(d_output_.get());
}

std::vector<float> InferenceEngine::getOutputDeviceData() const
{
    std::vector<float> host_output(getOuputSize() / sizeof(float));
    cudaMemcpy(host_output.data(), getOutputDevicePtr(), getOuputSize(),
               cudaMemcpyDeviceToHost);

    return host_output;
}

size_t InferenceEngine::getOuputSize() const { return output_size_; }

size_t InferenceEngine::getInputSize() const { return input_size_; }

void InferenceEngine::checkEngineSpecs()
{
    int numBindings = engine_->getNbBindings();
    std::cout << "Number of bindings: " << numBindings << std::endl;

    for (int i = 0; i < numBindings; ++i)
    {
        const char* name = engine_->getBindingName(i);
        nvinfer1::Dims dims = engine_->getBindingDimensions(i);
        nvinfer1::DataType dtype = engine_->getBindingDataType(i);
        bool isInput = engine_->bindingIsInput(i);

        std::cout << "Binding index " << i << ": " << name << std::endl;
        std::cout << "  Is input: " << (isInput ? "Yes" : "No") << std::endl;
        std::cout << "  Dimensions: ";
        for (int j = 0; j < dims.nbDims; ++j)
        {
            std::cout << dims.d[j] << " ";
        }
        std::cout << std::endl;
        std::cout << "  Data type: "
                  << (dtype == nvinfer1::DataType::kFLOAT ? "FLOAT" : "Other")
                  << std::endl;
    }
}
