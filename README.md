# ESP32 Autonomous Navigation Robot

## Overview

This project implements a real-time autonomous obstacle avoiding robot using ESP32 microcontroller, ultrasonic sensing, IMU-based sensor fusion, PID motor control, and finite state machine (FSM) navigation logic.

The robot continuously detects obstacles, processes sensor data in real-time, and dynamically controls movement using embedded firmware architecture.

---

## Features

- Real-time obstacle detection
- Ultrasonic distance sensing
- MPU6050 IMU integration
- Complementary filter sensor fusion
- PID-based motor control
- Finite State Machine (FSM) navigation
- Modular embedded firmware architecture
- ESP32 real-time control system
- Wokwi simulation support

---

## Hardware Components

- ESP32 Development Board
- HC-SR04 Ultrasonic Sensor
- MPU6050 IMU Sensor
- Motor driver logic
- LEDs for motor state indication

---

## Software & Technologies

- Embedded C++
- Arduino IDE
- ESP32 Framework
- Sensor Fusion
- PID Control
- Finite State Machine (FSM)
- I2C Communication
- Wokwi Embedded Simulation

---

## Project Structure

```text
ESP32-Obstacle-Avoiding-Robot/
│
├── autonomous_robot.ino
├── PIDController.h
├── RobotFSM.h
├── ComplementaryFilter.h
├── diagram.json
├── wokwi-simulation.png
├── sensor-configuration.png
└── system-overview.png
```

---

## System Architecture

The ESP32 acts as the central controller for sensor acquisition, state management, and motor control.

### Main Functional Blocks

- Sensor acquisition layer
- Complementary filter processing
- FSM navigation logic
- PID motor control
- Real-time actuator response

---

## Simulation

The project was developed and validated using the Wokwi embedded systems simulation platform.

---

## Future Improvements

- FreeRTOS task scheduling
- Bluetooth/WiFi telemetry
- Autonomous path planning
- Camera integration
- SLAM implementation

---

## Author

**Sowmiya Ashokkumar**

M.Sc Embedded Systems  
Technische Universität Chemnitz
