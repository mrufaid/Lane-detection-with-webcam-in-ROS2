

# ROS 2 Lane Detection from Webcam

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

## Description

This project demonstrates real-time lane detection using ROS 2 and OpenCV, with support for external webcams. It detects and overlays lane markings on live video streams from webcams, making it suitable for applications such as autonomous driving and robotics projects.

## Features

- Real-time lane detection
- Support for external webcams
- Adjustable parameters for lane detection algorithm
- Easy integration with ROS2 systems

## Installation

1. Clone this repository in the src folder of the ros2 workspace:

   ```bash
   git clone https://github.com/your_username/ros2-lane-detection.git
   ```


2. Build the ROS 2 package:

   ```bash
   colcon build
   ```

## Usage

1. Source setup.bash file:

   ```bash
   source /install/setup.bash
   ```

2. Run the ROS 2 node:

   ```bash
   ros2 run your_package_name cam_pub
   ```
   ```bash
   ros2 run your_package_name cam_sub
   ```
   

## Adjust the parameters in the subscriber node to detect lanes properly if necessary.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgements

- Special thanks to [OpenCV](https://opencv.org/) and [ROS 2](https://index.ros.org/doc/ros2/) communities for their contributions to open-source software.

## Contributing

Contributions are welcome! Please feel free to open a pull request or create an issue for any improvements or suggestions. 

```

Replace `your_username`, `your_package_name`, and adjust any other details as necessary. This template provides sections for description, features, installation instructions, usage, license, acknowledgments, and contributing guidelines. Feel free to customize it further to suit your project's specific needs!
