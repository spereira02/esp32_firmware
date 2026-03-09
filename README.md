# esp32_firmware
Integrated an ICM-20948 9-axis sensor via I2C (400kHz) using a custom FreeRTOS task. Implemented register-bank switching and power management for high-stability raw data streaming, targeting ROS 2 Jazzy integration on a Raspberry Pi 5.


# micro-ROS Serial Agent Setup (ESP32 → ROS 2)

This guide outlines the deployment of the micro-ROS Agent to bridge communication between the ESP32 client and the ROS 2 computational graph via serial transport.

> **Note:** This setup is validated for ROS 2 Jazzy. For other distributions, replace the `jazzy` keyword with your specific version (e.g., `humble`).

---

## 1. Install the micro-ROS Agent

The agent must be built within a dedicated ROS 2 workspace to manage the XRCE-DDS middleware.

```bash
# Create workspace and clone repository
mkdir -p uros_ws/src && cd uros_ws/src
git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git
```

# Install dependencies
```bash
cd ..
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
# Build and source environment
```bash
colcon build --symlink-install
source install/local_setup.bash
``` 
Verify installation:
```bash
ros2 pkg list | grep micro_ros_agent
```
2. Configure Serial Permissions

To access serial devices without root privileges, add the current user to the dialout group.
```bash

sudo usermod -a -G dialout $USER
```

Important: You must log out and log back in (or reboot) for group changes to take effect. Verify with the groups command.

3. Identify the ESP32 Serial Port

Connect the ESP32 and identify the assigned device node:
```bash

ls /dev/ttyUSB*
ls /dev/ttyACM*
```

Typical output: /dev/ttyUSB0

4. Start the micro-ROS Agent

Execute the agent in serial mode. Ensure the baud rate matches your firmware configuration (default: 115200).
```bash

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```
Synchronizing the Client

Once the agent is active, press the RESET (RST) button on the ESP32 to establish the session.

Expected output:

[info] Root.cpp | create_client | session established
[info] ProxyClient.cpp | create_participant | participant created
[info] ProxyClient.cpp | create_topic | topic created: /imu/acc_gyro
[info] ProxyClient.cpp | create_datawriter | datawriter created
5. Verify ROS 2 Telemetry

Open a new terminal, source your ROS 2 environment, and verify the data stream:
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
```
Echo the IMU data stream:
```bash
ros2 topic echo /imu/acc_gyro
Troubleshooting
```
Baud Rate: If the session fails to establish, specify the baud rate explicitly:
```bash

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 921600
```

Port Access: Ensure no other serial monitors (Arduino IDE, etc.) are accessing the port.

Architecture: If deploying on Raspberry Pi 5, ensure uros_ws was built locally on the Pi to match the ARM64 architecture.
