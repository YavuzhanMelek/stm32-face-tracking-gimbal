# 3-Axis Face-Tracking Camera Gimbal (STM32 & ESP32-CAM)

![STM32](https://img.shields.io/badge/STM32-ARM_Cortex--M4-blue.svg)
![Computer Vision](https://img.shields.io/badge/Computer_Vision-Face_Tracking-orange.svg)
![Status](https://img.shields.io/badge/Status-Completed-success.svg)

## 📌 Overview
This project is an autonomous 3-axis motorized camera gimbal that actively tracks human faces in real-time. Powered by an **STM32F407 (ARM Cortex-M4)** microcontroller and an **ESP32-CAM** module, the system detects a face, calculates its center coordinates, and autonomously drives servo motors to keep the face perfectly centered in the camera's frame.

Rather than simple predefined movements, this project implements a dynamic closed-loop control system bridging embedded computer vision with low-level hardware actuation.

## ⚙️ System Architecture & Logic
The system operates on a continuous feedback loop (Error Minimization):

1. **Face Detection (ESP32-CAM):** The camera captures the video feed and runs a face detection algorithm to identify a human face in the frame.
2. **Coordinate Extraction:** It calculates the `(X, Y)` coordinates of the face's center point.
3. **Data Transmission:** The center coordinates are transmitted in real-time to the STM32 microcontroller via UART communication.
4. **Error Calculation (STM32):** The STM32 parses the incoming data and compares the face's center coordinates with the camera frame's actual center (the target). It calculates the positional error (dx, dy).
5. **Hardware Actuation:** Based on the error margin, the STM32 dynamically adjusts the hardware PWM signals (using internal Timers) sent to the Pan and Tilt servos. The motors move in the direction that minimizes the error, effectively keeping the face tracked in the center.

## 🛠️ Hardware Requirements
* STM32F407VGT6 Development Board
* ESP32-CAM Module (for vision and coordinate processing)
* 3x Servo Motors (Pan, Tilt, Roll architecture)
* External Power Supply for servos
* Breadboard & Jumper Wires

## 📂 Repository Structure
* `/Core/Src/` - Contains the main C application, PWM control logic, and UART parsing algorithms (`main.c`, `stm32f4xx_it.c`).
* `/Core/Inc/` - Header files for peripheral drivers.
* `*.ioc` - STM32CubeMX configuration file mapping the pinouts, clock tree, and hardware timers.

## 🚀 Getting Started
1. Clone this repository:
   ```bash
   git clone [https://github.com/YourUsername/stm32-face-tracking-gimbal.git](https://github.com/YourUsername/stm32-face-tracking-gimbal.git)
