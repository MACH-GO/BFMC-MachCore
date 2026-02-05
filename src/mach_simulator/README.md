# MACH Simulator

This package handles the simulation of the BFMC car using Gazebo and ROS 2.

## Features
- **Launch Files**: Start the full simulation environment.
- **Config**: ROS 2 Control configuration for the simulated vehicle.

## Usage

To launch the simulator:

colcon build --symlink-install
source install/setup.bash
ros2 launch mach_simulator bfmc_sim.launch.py

## Configuration

- **ros2_control.yaml**: Defines the controllers and hardware interfaces for the simulation.
