# ROS2 Lane Detection and Motion Control System

## Overview
This ROS2-based system runs on a Jetson Nano and processes camera input to detect lane markings and control vehicle motion accordingly. The system consists of multiple nodes that handle image capture, lane detection, motion control, and visualization.

### High-Level System Flow:
1. **Camera Node**: Captures and publishes raw image data.
2. **Vision Processing Nodes**: Extract lane markings and publish lane position data.
3. **Motion Control Node**: Estimates lane distance and adjusts steering.
4. **Lane Visualization Node**: Publishes processed lane data for visualization in Rviz.

## Package Structure
```
.
├── camera
├── classic_vision
├── image_publisher
├── lane_msgs
├── lane_visualization
└── motion_control
```
Each package follows the ROS2 standard with `CMakeLists.txt`, `package.xml`, source files (`src`), and header files (`includes`).

---
## Node Descriptions

### 1. Camera Node (`camera`)
**Function**: Captures image frames and publishes them.
- **Publishes**:
  - `/image_raw` (sensor_msgs::Image)

### 2. Classic Vision Node (`classic_vision`)
**Function**: Processes raw images to detect lane markings.
- **Subscribes**:
  - `/image_raw` (sensor_msgs::Image)
- **Publishes**:
  - `/lane_position` (custom message)
- **Parameters**:
  - `low_canny_threshold` / `high_canny_treshold`: Edge detection parameters
  - `hough_transform_params`: Line detection parameters
	- `max_detected_lines`
	- `max_line_gap`
	- `mine_line_length`
	- `rho`

### 3. Image Publisher Node (`image_publisher`)
**Function**: Auxiliary node for testing image data flow.
- **Publishes**:
  - `/debug_image` (sensor_msgs::Image)
- **Parameters**:
  - `image_name`: image from assets folder

### 4. Lane Messages (`lane_msgs`)
**Function**: Defines custom message types for lane detection data.
- **Custom Messages**:
  - `LanePosition.msg`: Lane position relative to vehicle

### 5. Lane Visualization (`lane_visualization`)
**Function**: Publishes visualization markers for Rviz.
- **Subscribes**:
  - `/lane_position` (custom message)
- **Publishes**:
  - `/visualization_marker` (visualization_msgs::Marker)

### 6. Motion Control Node (`motion_control`)
**Function**: Computes steering commands based on lane position.
- **Subscribes**:
  - `/lane_position` (custom message)
- **Publishes**:
  - `/cmd_vel` (geometry_msgs::Twist)
- **Parameters**:
  - `base_speed`: base speed of vehicle
  - `bucket_size`: vertical size in pixel for points clustering
  - `lookahead_index`: distance (in buckets) to the reference point
  - `kp`, `ki`, `kd`: proportional, integral and derivative gain.
---
```
