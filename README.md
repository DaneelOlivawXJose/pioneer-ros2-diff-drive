# Pioneer: ROS 2 Differential Drive Mobile Robot

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue) ![C++](https://img.shields.io/badge/C++-17-blue) ![micro-ROS](https://img.shields.io/badge/micro--ROS-embedded-green)

Pioneer is a custom-built differential drive mobile robot designed from scratch. It integrates low-level motor control via FreeRTOS with high-level navigation and planning using ROS 2 Humble.

![Pioneer Robot](./docs/robot_front.jpg) *(Añadir foto del robot)*

## System Architecture

The software stack is divided into two main domains:
1. **Low-Level (MCU):** Runs FreeRTOS and micro-ROS. Handles raw encoder reading, closed-loop PID velocity control for each wheel, and publishes Odometry (`/odom`) to the ROS 2 graph.
2. **High-Level (Host/SBC):** Runs standard ROS 2 Humble. Manages the differential kinematics, trajectory planning (Nav2), and teleoperation.

## Hardware Specs
* **Motors:** DC Motors with quadrature encoders.
* **Motor Driver:** *(Añade tu driver aquí, ej. L298N / TB6612FNG)*
* **MCU:** *(Añade tu micro, ej. ESP32)* * **Sensors:** *(Añade LiDAR/Cámara si la hay)*

## Quick Start

### 1. MCU Firmware Upload
Navigate to the firmware directory and build the micro-ROS agent:
\`\`\`bash
cd firmware_mcu
# Use your preferred build system (PlatformIO / ESP-IDF) to flash the MCU
\`\`\`

### 2. ROS 2 Workspace Setup
Clone and build the colcon workspace on your host machine:
\`\`\`bash
source /opt/ros/humble/setup.bash
cd pioneer_ws
colcon build --symlink-install
source install/setup.bash
\`\`\`

### 3. Launching the System
Launch the core robot nodes and the micro-ROS agent:
\`\`\`bash
ros2 launch pioneer_core bringup.launch.py
\`\`\`
To control the robot via joystick:
\`\`\`bash
ros2 launch pioneer_core teleop.launch.py
\`\`\`
