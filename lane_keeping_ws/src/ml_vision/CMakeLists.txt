cmake_minimum_required(VERSION 3.8)
project(ml_vision)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-10.2")
set(CMAKE_CUDA_COMPILER "${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc")

include_directories(
  ${CUDA_TOOLKIT_ROOT_DIR}/include  )

# Manually specify TensorRT paths (Jetson-specific)
set(TENSORRT_INCLUDE_DIRS "/usr/include/aarch64-linux-gnu")
set(TENSORRT_LIB_DIR "/usr/lib/aarch64-linux-gnu")
include_directories(${TENSORRT_INCLUDE_DIRS})
link_directories(${TENSORRT_LIB_DIR})

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(lane_msgs REQUIRED)

include_directories(includes)


add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/MlVisionNode.cpp
  src/Logger.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBS}
  nvinfer
  nvinfer_plugin
  cudart  # From CUDA
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  cv_bridge
  lane_msgs
)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
ament_package()
