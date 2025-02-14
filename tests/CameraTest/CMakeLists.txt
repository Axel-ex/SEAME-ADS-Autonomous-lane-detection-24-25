cmake_minimum_required(VERSION 3.8)
project(camera_test)

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
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(exec_name ${PROJECT_NAME})
set(srcs main.cpp)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS} includes)
add_executable(${exec_name} ${srcs})

# Link against OpenCV and other libraries
target_link_libraries(${exec_name}
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
)
