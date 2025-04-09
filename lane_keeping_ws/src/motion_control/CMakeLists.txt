cmake_minimum_required(VERSION 3.8)
project(motion_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic -g)
# endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)

set(exec_name ${PROJECT_NAME}_node)
set(src_dir src/)
set(srcs
    ${src_dir}main.cpp
	${src_dir}MotionControlNode.cpp
	${src_dir}PIDController.cpp
	${src_dir}PolyFit.cpp
	${src_dir}KalmanFilter.cpp
	${src_dir}LaneBuffer.cpp
)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS} includes)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lane_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Gtest
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  
  # Create test executable
  ament_add_gmock(${PROJECT_NAME}_test 
	  test/src/LaneBufferTest.cpp
	  ${src_dir}LaneBuffer.cpp
  )
  
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    test/include
  )
  
  # Add dependencies to the test target
  ament_target_dependencies(${PROJECT_NAME}_test 
    rclcpp
	lane_msgs
  )
endif()

add_executable(${exec_name} ${srcs})

target_link_libraries(${exec_name}
  ${OpenCV_LIBS}
)
ament_target_dependencies(${exec_name} rclcpp std_msgs sensor_msgs lane_msgs visualization_msgs)

install(TARGETS ${exec_name} DESTINATION lib/${PROJECT_NAME})
ament_package()
