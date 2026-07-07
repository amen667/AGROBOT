# AGROBOT

AGROBOT is a low-cost mini-greenhouse inspection robot using ROS2, LiDAR mapping, and AI vision.

## Project Objective

The objective of this project is to build a mobile robot that can move through a simulated mini-greenhouse, create a map using LiDAR, inspect plants using a camera, and detect early visual signs of plant stress.

This project is also a learning project focused on robotics, ROS2, AI, embedded systems, Git, and project management.

## Main Features

- Mobile robot based on Raspberry Pi 4 and ESP32
- 2D LiDAR mapping using YDLIDAR X4 Pro
- ROS2-based robot software
- Plant inspection using Raspberry Pi Camera
- AI/computer vision for plant health detection
- Simple inspection report generation

## Hardware

- Raspberry Pi 4
- ESP32
- YDLIDAR X4 Pro
- Raspberry Pi Camera
- 4 DC motors RM-J002
- L298N motor driver
- LiPo 3S battery
- Ultrasonic sensor
- MH sensor modules
- Robot chassis plates

## Software

- Ubuntu 24.04
- ROS2 Jazzy
- Python
- OpenCV
- SLAM Toolbox
- Nav2
- Git and GitHub

## Project Structure

```text
AGROBOT/
├── ai_vision/
├── datasets/
├── docs/
├── firmware_esp32/
├── hardware/
├── images/
├── maps/
├── raspberry_pi/
├── ros2_ws/
├── videos/
└── README.md