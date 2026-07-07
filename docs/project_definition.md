# AGROBOT - Mini Greenhouse Inspection Robot

## Problem

Small greenhouse spaces can be narrow and difficult to inspect frequently. Some plants may show early signs of stress such as yellow leaves, dryness, weak growth, or disease signs.

## Objective

Build a low-cost mobile robot that moves through a mini-greenhouse environment, maps the path using LiDAR and ROS2, captures plant images, and detects visual signs of plant stress using AI/computer vision.

## First Prototype Scope

The robot will move on a flat floor.
The greenhouse will be simulated using cardboard/plastic walls or a small corridor.
The first plant tests may use printed plant images or photos displayed on a screen.
The system will detect visual risk signs, not make medical/agricultural certainty.

## Main Functions

1. Move through a mini-greenhouse path.
2. Use YDLIDAR X4 Pro to scan the environment.
3. Create and save a ROS2 map.
4. Capture plant images using Raspberry Pi Camera.
5. Detect plant stress signs:
   - healthy
   - yellow/dry
   - diseased-looking
   - weak growth
6. Generate a simple inspection report.

## Hardware

- ASUS TUF A15 laptop, RTX 2050, Ryzen 5, 24 GB RAM
- Raspberry Pi 4
- ESP32
- YDLIDAR X4 Pro
- Raspberry Pi Camera
- 4 RM-J002 DC motors with wheels
- L298N motor driver
- LiPo 3S battery
- Ultrasonic sensor
- MH sensor modules
- Robot chassis plates
- Breadboard and jumper wires

## Software

- Ubuntu 24.04
- ROS2 Jazzy
- Python
- OpenCV
- YDLIDAR ROS2 driver
- SLAM Toolbox
- Nav2
- GitHub