# ROS System Documentation
This document provides a complete overview of the ROS 2 system architecture, including **packages**, **nodes** and **topics**. It is intended as a single source of truth for all developers working on the project.

## Workspace Overview
- **ROS2 Distribution**: Humble
- **Platform**: Ubuntu 22.04
```plaintext
machcore_ws/
├── src/
│   ├── mach_control/
│   ├── nucleo_interface/
│   └── ...
├── install/
├── build/
└── log/
```

## Packages

### [nucleo_interface](../src/nucleo_interface) 
**Description**: 
Hardware interface package responsible for all communication with the Nucleo microcontroller over serial. Converts high-level Ackermann commands into motor and steering commands and publishes sensor feedback.

**Package type**: ament_cmake

**Dependencies**:
- rclcpp
- ackermann_msgs
- std_msgs
- sensor_msgs

### [mach_control](../src/mach_control)
**Description**: 
High-level control package containing teleoperation and autonomous control nodes. All nodes in this package publish vehicle commands using Ackermann semantics.

**Package type**: ament_cmake

**Dependencies**:
- rclcpp
- ackermann_msgs

## Nodes

### hw_interface
**Package**: nucleo_interface

**Executable**: `hw_interface`

**Purpose**: Hardware interface node. Communicates with the Nucleo microcontroller over serial. Converts high-level commands into motor and steering commands.

**Subscribes**:
- `/drive_cmd` (ackermann_msgs/AckermannDriveStamped)

**Publishes**:
- `/serial_data` (std_msgs/String)
- `/imu` (sensor_msgs/Imu)
  
**Parameters**:
- `port` (string, default: `/dev/ttyACM0`)
- `baudrate` (int, default: 115200)
- `steer_center_units` (double)
- `steer_max_units` (double)
- `steer_max_angle_deg` (double, degrees)


### ackermann_teleop_key
**Package**: mach_control

**Executable**: `ackermann_teleop_key`

**Purpose**: Manual keyboard-based control of the vehicle. Publishes speed and steering commands using Ackermann semantics.

**Publishes**:
- `/drive_cmd` (AckermannDriveStamped)
  
**Parameters**:
- `max_speed` (double, m/s)
- `min_speed` (double, m/s)
- `max_steering_angle` (double, rad)
- `speed_increment` (double)
- `publish_rate` (double, Hz)

## Topics

### /drive_cmd
**Type**: `ackermann_msgs/AckermannDriveStamped`

**Description**: Primary actuation command topic for the vehicle. Used by teleop and autonomous controllers.

**Fields**:
- `header.stamp` &mdash; time of command
- `header.frame_id` &mdash; reference frame
- `drive.speed` &mdash; vehicle longitudinal speed (m/s)
- `drive.steering_angle` &mdash; front wheel steering angle (rad)

### /imu
**Type**: `sensor_msgs/Imu`

**Description**: IMU data received from the Nucleo and published to ROS.

### /serial_data
**Type**: `std_msgs/String`

**Description**: Raw serial messages received from the Nucleo for debugging and monitoring.
