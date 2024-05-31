
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

# Lane Detection from Webcam using ROS2, and OpenCV

This project integrates ROS2 (Robot Operating System 2), and OpenCV to develop a real-time lane detection system using a webcam. The system is designed to detect and track lanes, making it useful for autonomous driving and robotics applications.

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Installation](#installation)
  - [Prerequisites](#prerequisites)
  - [Setting up the Environment](#setting-up-the-environment)
  - [Installing Dependencies](#installing-dependencies)
- [Usage](#usage)
  - [Running the Lane Detection Node](#running-the-lane-detection-node)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

## Introduction

This project combines the power of ROS2 for robotics middleware, Ultralytics for advanced machine learning models, and OpenCV for computer vision tasks to create a robust lane detection system. The system captures video from a webcam and processes it to detect and highlight lanes in real-time.

## Features

- **Real-time Lane Detection**: Utilizes state-of-the-art machine learning models and computer vision techniques.
- **ROS2 Integration**: Seamlessly integrates with ROS2, enabling easy integration with other ROS2-based systems.
- **OpenCV Support**: Uses OpenCV for image processing and visualization.
- **Customizable**: Allows customization of detection models and parameters.

## Installation

### Prerequisites

Ensure you have the following installed on your system:

- ROS2 (Foxy, Galactic, or Humble recommended)
- Python 3.8+
- OpenCV


### Setting up the Environment

1. **Install ROS2**: Follow the official ROS2 installation guide [here](https://docs.ros.org/en/foxy/Installation.html).

2. **Create a ROS2 Workspace**:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/
    colcon build
    source install/setup.bash
    ```

### Installing Dependencies

3. **Clone the Repository**:
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/mrufaid/Lane-detection-with-webcam-in-ROS2.git
    ```

4. **Install Python Dependencies**:
    ```bash
    cd ~/ros2_ws/src/Lane-detection-with-webcam-in-ROS2
    pip install -r requirements.txt
    ```

5. **Build the Workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```

## Usage

### Running the Lane Detection Node

1. **Run ROS2 Nodes**:
    ```bash
    ros2 run lane_detection cam_pub.py
    ros2 run lane_detection cam_sub.py
    ```

2. **Viewing Lane Detection Results**:
    A window will open accessing the webcam and displaying detected lane lines.

### Configuring the System

## Contributing

Contributions are welcomed to this project. To contribute:

1. Fork the repository.
2. Create a new branch (`git checkout -b feature-xyz`).
3. Make your changes.
4. Commit your changes (`git commit -m 'Add feature xyz'`).
5. Push to the branch (`git push origin feature-xyz`).
6. Create a new Pull Request.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgements

- [ROS2](https://docs.ros.org/en/foxy/index.html) - Robotics middleware framework.
- [OpenCV](https://opencv.org/) - Open-source computer vision library.

---
