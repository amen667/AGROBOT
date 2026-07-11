# AGROBOT Mobile Base Plan

## Final decision for AGROBOT v1

The first mobile base of AGROBOT will use a differential drive architecture:

- 2 × JGA25-371 DC gear motors with built-in encoders
- 1 × ball/caster wheel
- ESP32 for motor control and encoder reading
- BTS7960 motor drivers
- 3S LiPo battery for motor power
- Laptop/Raspberry Pi for ROS2 processing
- YDLIDAR X4 Pro for LiDAR scan data

This base is better for SLAM than the 4 non-encoder motors because the encoders allow real wheel odometry.

## Why differential drive?

The robot has:

- one left motor
- one right motor
- one free ball wheel

Movement is controlled by changing the speed and direction of the left and right motors.

Examples:

- Left forward + right forward = forward
- Left backward + right backward = backward
- Left slow + right fast = turn left
- Left forward + right backward = rotate in place

## Odometry plan

The encoders will measure wheel rotation.

The ESP32 will count encoder pulses from:

- left motor encoder
- right motor encoder

From these pulses, we can calculate:

- distance traveled by the left wheel
- distance traveled by the right wheel
- robot linear movement
- robot rotation

The result will later be sent to ROS2 as `/odom`.

## ROS2 target architecture

LiDAR:

- YDLIDAR publishes `/scan`

Mobile base:

- ESP32 reads encoders
- ESP32 controls motors
- ESP32 sends odometry data
- ROS2 receives odometry as `/odom`

SLAM:

- SLAM Toolbox uses `/scan` and `/odom`
- RViz displays `/map`, `/scan`, and robot pose

## Current status

Completed:

- Ubuntu installed
- ROS2 Jazzy installed
- AGROBOT GitHub repository created
- First ROS2 package created: `agrobot_basics`
- YDLIDAR X4 Pro working
- `/scan` topic working
- RViz displays LiDAR scan
- First SLAM Toolbox launch tested

Pending:

- Bring JGA25 encoder motor robot base
- Identify motor and encoder wire colors
- Cable motors to BTS7960
- Cable encoder signals to ESP32
- Test motor movement
- Test encoder pulse counting
- Create odometry calculation
- Send odometry to ROS2