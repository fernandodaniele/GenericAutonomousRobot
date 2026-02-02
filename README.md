# GenericAutonomousRobot (GAR)
A comprehensive educational robotics platform for Electronic Engineering students. Features autonomous navigation, DSP, and Real-Time Systems integration using STM32 F4 (FreeRTOS) and ESP32 S3 (IoT Gateway), focused on modular firmware and technical analysis.

**GenericAutonomousRobot** is an open-source educational platform designed for Electronic Engineering students to bridge the gap between theoretical concepts and industrial applications. 

The project focuses on building an autonomous rover capable of real-time navigation, telemetry, and self-logging for analysis.

## 🚀 Core Concepts

* **Real-Time Systems (RTOS):** Powered by **FreeRTOS** on an **STM32 F4 Discovery**. Focuses on task prioritization, preemptive scheduling, and inter-task communication.
* **Digital Signal Processing (DSP):** Implementation of digital filters (FIR/IIR).
* **IoT & Networking:** An **ESP32 S3** acts as a high-level gateway, managing connectivity for real-time monitoring.
* **Autonomous Navigation:** Integration of simple obstacle avoidance (Ultrasonic) and cliff detection (TCRT5000) for autonomous movement.
* **Technical Forensics & Appraisal:** Includes a "Black Box" logging system. Students learn to perform post-failure analysis and create technical reports (Pericias).

## 🛠 Hardware Stack
- **Control MCU:** STM32F407G-DISC1 (ARM Cortex-M4 @ 168MHz)
- **Comms MCU:** ESP32 S3 DevKit (Xtensa Dual-Core with AI acceleration)
- **Sensors:** MPU6050 (IMU), HC-SR04 (Ultrasonic), 5x TCRT5000 (Infrared)
- **Actuators:** Dual DC Motors via TB6612FNG Driver
- **Power:** TBD

## 📂 Project Structure
- `/firmware/stm32_control`: Core control logic, RTOS tasks, and DSP filters.
- `/firmware/esp32_gateway`: Communication bridge and telemetry.
- `/analysis`: Octave/Matlab scripts for signal analysis and filter design.
- `/hardware/kicad`: Electronic design files, including schematics and PCB layouts.
- `/mechanical`: Bill of Materials (BOM) and physical assembly specifications.
- `/docs/doxygen`: Automated technical documentation generated from source code.
- `/docs/reports`: Pericial report templates and valuation tools.

## ⚙️ CI/CD
This repository is configured with **GitHub Actions** to automatically build and verify both STM32 and ESP32 firmwares on every push, ensuring code integrity and modularity.
