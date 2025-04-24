# SEA:ME ADS Project - Autonomous Driving System
## Overview
This project focuses on the development of an **Autonomous Driving System (ADS)** capable of detecting lanes and steering accordingly.

Utilizing **ROS 2**, a **Jetson Nano**, and a **Raspberry Pi**, we designed a system that integrates **computer vision and machine learning** to detect lanes, calculate optimal paths, and execute autonomous steering. The system leverage GPU accelerated computations on the jetson nano, reducing significantly computation time compared to a CPU based approach.

Documentation for the sources of this project can be find [here](https://axel-ex.github.io/SEAME-ADS-Autonomous-lane-detection-24-25).

## Project Scope
This project aims to bridge the gap between **simulation-based** testing and **real-world autonomous driving** by designing and deploying an AI-driven lane detection and control system. The system consists of multiple ROS 2 nodes encapsulating specific aspects of the system (see [here](https://github.com/Axel-ex/SEAME-ADS-Autonomous-lane-detection-24-25/tree/ml_vision/lane_keeping_ws) for more detail on each node responsability):

- **ROS 2 Nodes** for real-time data processing and vehicle control.
- **Computer Vision (OpenCV)** for basic lane detection.
- **Machine Learning (PyTorch/TensorFlow)** for advanced lane detection.
- **Carla sim** for software in the loop testing.

## Key Features
- **ROS 2 Integration**: Modular and scalable for future improvements.
- **Hybrid Lane Detection**: Ability to switch from traditional computer vision to ML-based approaches.
- **Real-time Processing**: Optimized to run efficiently on Jetson Nano (CUDA acceleration).
- **Simulation / Reality testing**: Tested in both virtual (Carla) and real-world environments.

## Launch instructions
1. To launch all nodes:
```bash
ros2 run launch launch/jetson_launch.py

```
2. To launch the system without the camera (usefull to replay data using ros2 bags)
```bash
ros2 run launch launch/bags_launch.py
```
then you can replay some recorded data in another terminal
```bash
cd ../bags
ros2 bag play <name_of_the_bag>
```

3. To launch the system with the image publisher (usefull to develop / to debug)
```bash
ros2 run launch launch/bags_launch.py

```
4. To launch RQT visualization tool and visualize images beeing published into topics, run:
```bash
ros2 run rqt_image_view rqt_image_view
```
