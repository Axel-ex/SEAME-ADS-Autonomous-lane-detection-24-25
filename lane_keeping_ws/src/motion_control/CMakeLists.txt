cmake_minimum_required(VERSION 3.8)
project(motion_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(exec_name ${PROJECT_NAME}_node)
set(src_dir src/)
set(srcs
        ${src_dir}main.cpp
		${src_dir}MotionControlNode.cpp)


# Include directories
include_directories(${OpenCV_INCLUDE_DIRS} includes)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(${exec_name} ${srcs})

ament_target_dependencies(${exec_name} rclcpp std_msgs sensor_msgs)

install(TARGETS ${exec_name} DESTINATION lib/${PROJECT_NAME})
ament_package()
