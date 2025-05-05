cmake_minimum_required(VERSION 3.8)
project(classic_vision)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# CUDA support
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-10.2")
set(CMAKE_CUDA_COMPILER "${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc")
set(CMAKE_CUDA_ARCHITECTURES 53)  # Jetson Nano uses sm_53
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3)
find_package(CUDA REQUIRED)
enable_language(CUDA)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui cudaimgproc cudawarping cudafilters)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

set(exec_name ${PROJECT_NAME}_node)
set(src_dir src/)
set(srcs
        ${src_dir}main.cpp
        ${src_dir}VisionNode.cpp)


# Include directories
include_directories(${OpenCV_INCLUDE_DIRS} includes)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(custom_msgs REQUIRED)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

add_executable(${exec_name} ${srcs})

# Link against OpenCV and other libraries
target_link_libraries(${exec_name}
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
)

ament_target_dependencies(${exec_name} rclcpp std_msgs sensor_msgs cv_bridge image_transport custom_msgs)

install(TARGETS ${exec_name} DESTINATION lib/${PROJECT_NAME})
ament_package()
