# Pioneer: ROS 2 Differential Drive Mobile Robot

![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue) ![micro-ROS](https://img.shields.io/badge/micro--ROS-embedded-green) ![FreeRTOS](https://img.shields.io/badge/RTOS-FreeRTOS-orange) ![C++](https://img.shields.io/badge/C++-14/17-blue)

Pioneer is a custom-built differential drive mobile platform. Instead of relying on off-the-shelf ROS controllers, this project implements the entire stack from scratch: from the low-level deterministic PID motor control on a microcontroller to the high-level ROS 2 Humble integration for teleoperation and telemetry.

![Pioneer Robot](./docs/robot_front.jpg)

## System Architecture

The architecture is split into two distinct layers to separate hard real-time constraints from high-level planning.

### 1. Low-Level Control (MCU / FreeRTOS)
Running on an ESP32, this layer handles the physical hardware. I used FreeRTOS to ensure the PID velocity loops run deterministically.
* **Kinematics Engine:** Converts incoming `geometry_msgs/Twist` (`/cmd_vel`) from the ROS graph into target RPMs for the left and right wheels (Inverse Kinematics).
* **Closed-Loop PID:** Two independent PID controllers regulate the DC motors. Encoder pulses are read via hardware interrupts/pulse counters to avoid CPU bottlenecking.
* **Odometry:** Calculates dead-reckoning (Forward Kinematics) based on encoder ticks and publishes `nav_msgs/Odometry` and the `odom -> base_link` TF directly to the ROS 2 network via micro-ROS over Serial/WiFi.

### 2. High-Level Control (Host SBC / PC)
Running on Ubuntu 22.04 with ROS 2 Humble.
* Manages the robot description (URDF/Xacro) and the TF tree.
* Handles joystick inputs and teleoperation mapping.
* Runs a custom Python-based SCADA interface for real-time monitoring of motor effort, battery, and wheel slip.

---

## Hardware Setup
* **Microcontroller:** ESP32 (NodeMCU / WROOM)
* **Actuators:** 12V DC Motors with Quadrature Encoders
* **Motor Driver:** BTS7960 / L298N *(Ajusta esto según tu hardware real)*
* **Power:** 3S LiPo Battery with DC-DC buck converters for logic.

---

## 🚀 How to Build and Run

To replicate or run this project, you need a machine with Ubuntu 22.04 and ROS 2 Humble installed, plus the ESP-IDF or PlatformIO for the microcontroller.

### Step 1: Flash the Firmware
The MCU code is located in the `/firmware` folder. It uses micro-ROS, so ensure you have the micro-ROS build environment set up.
\`\`\`bash
cd firmware
# Compile and upload to the ESP32 via USB
pio run -t upload 
\`\`\`

### Step 2: Build the ROS 2 Workspace
Clone this repository into your `ros2_ws/src` directory and build the packages:
\`\`\`bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
\`\`\`

### Step 3: Run the System

**1. Start the micro-ROS Agent:**
The host needs to talk to the ESP32. Start the micro-ROS agent on the specific serial port (change `/dev/ttyUSB0` if necessary).
\`\`\`bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
\`\`\`
*Once connected, you should see the ESP32 nodes joining the ROS graph.*

**2. Launch the Core System (TF, URDF, and Teleop):**
Open a new terminal, source the workspace, and launch the bringup file. This will start the robot state publisher and the joystick node.
\`\`\`bash
ros2 launch pioneer_core bringup.launch.py
\`\`\`

**3. Launch the Python SCADA Interface (Optional):**
To view real-time telemetry (PID performance, target vs actual velocity):
\`\`\`bash
ros2 run pioneer_scada dashboard_node
\`\`\`

## ROS Graph & Topics

* **Subscribes to:**
  * `/cmd_vel` (`geometry_msgs/Twist`): Target linear and angular velocities.
* **Publishes:**
  * `/odom` (`nav_msgs/Odometry`): Computed odometry from wheel encoders.
  * `/tf`: `odom` -> `base_link` transformation.
  * `/motor_telemetry` (`std_msgs/Float32MultiArray`): Debugging data for the SCADA GUI.

---
*Developed by [Tu Nombre/Usuario]*
