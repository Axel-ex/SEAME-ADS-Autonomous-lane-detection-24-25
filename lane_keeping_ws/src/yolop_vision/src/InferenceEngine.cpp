#include <InferenceEngine.hpp>
#include <Logger.hpp>
#include <array>
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
        std::cerr << "Fail creating runtime";
        return false;
    }
    std::cout << "Runtime succefully created";

    engine_.reset(createCudaEngine());
    if (!engine_)
    {
        std::cerr << "Fail creating engine";
        return false;
    }
    std::cout << "engine succefully created";

    context_.reset(engine_->createExecutionContext());
    if (!context_)
    {
        std::cerr << "Fail creating context";
        return false;
    }
    std::cout << "context succefully created";

    allocateDevices();
    if (!d_input_ || !d_output_det_ || !d_output_road_seg_ ||
        !d_output_lane_seg_)
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
        std::cerr << "Couldnt open engine file";
        return nullptr;
    }
    std::cout << "Engine file loaded";

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
    const int obj_det_index = engine_->getBindingIndex(OUTPUT_OBJECT_DETECTION);
    const int road_seg_index =
        engine_->getBindingIndex(OUTPUT_ROAD_SEGMENTATION);
    const int lane_seg_index =
        engine_->getBindingIndex(OUTPUT_LANE_SEGMENTATION);

    Dims input_dims = engine_->getBindingDimensions(input_index);
    Dims obj_det_dims = engine_->getBindingDimensions(obj_det_index);
    Dims road_seg_dims = engine_->getBindingDimensions(road_seg_index);
    Dims lane_seg_dims = engine_->getBindingDimensions(lane_seg_index);

    for (int i = 0; i < input_dims.nbDims; i++)
        input_size_ *= input_dims.d[i];
    input_size_ *= sizeof(float);

    // Object detection
    for (int i = 0; i < obj_det_dims.nbDims; i++)
        output_det_size_ *= obj_det_dims.d[i];
    output_det_size_ *= sizeof(float);

    // Road segmentation
    for (int i = 0; i < road_seg_dims.nbDims; i++)
        output_road_seg_size_ *= road_seg_dims.d[i];
    output_road_seg_size_ *= sizeof(float);

    // Lane segmentation
    for (int i = 0; i < lane_seg_dims.nbDims; i++)
        output_lane_seg_size_ *= lane_seg_dims.d[i];
    output_lane_seg_size_ *= sizeof(float);

    // allocate memory and transfer ownership to our smartpointer
    void* raw_input_ptr = nullptr;
    void* obj_det_ptr = nullptr;
    void* road_seg_ptr = nullptr;
    void* lane_seg_ptr = nullptr;

    cudaError_t input_err = cudaMalloc(&raw_input_ptr, input_size_);
    cudaError_t output_err = cudaMalloc(&obj_det_ptr, output_det_size_);
    cudaError_t output_err2 = cudaMalloc(&road_seg_ptr, output_road_seg_size_);
    cudaError_t output_err3 = cudaMalloc(&lane_seg_ptr, output_lane_seg_size_);

    if (input_err != cudaSuccess || output_err != cudaSuccess ||
        output_err2 != cudaSuccess || output_err3 != cudaSuccess)
    {
        std::cerr << "An error occured while allocating for input / output "
                     "device: input: "
                  << cudaGetErrorString(input_err)
                  << ", output1: " << cudaGetErrorString(output_err);
    }

    d_input_.reset(raw_input_ptr);
    d_output_det_.reset(obj_det_ptr);
    d_output_road_seg_.reset(road_seg_ptr);
    d_output_lane_seg_.reset(lane_seg_ptr);
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

    void* bindings[4] = {d_input_.get(), d_output_det_.get(),
                         d_output_road_seg_.get(), d_output_lane_seg_.get()};
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
std::array<float*, 3> InferenceEngine::getOutputDevicePtrs() const
{
    return {static_cast<float*>(d_output_det_.get()),
            static_cast<float*>(d_output_road_seg_.get()),
            static_cast<float*>(d_output_lane_seg_.get())};
}

std::array<size_t, 3> InferenceEngine::getOuputSizes() const
{
    return {output_det_size_, output_road_seg_size_, output_lane_seg_size_};
}

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
