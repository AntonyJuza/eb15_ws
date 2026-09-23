# EB-15 Collaborative Robotic Arm - ROS 2 & Embedded Firmware Workspace

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-informational.svg)](https://docs.ros.org/en/jazzy/)
[![Microcontroller](https://img.shields.io/badge/Firmware-Arduino%20Nano-blue.svg)](https://www.arduino.cc/)
[![Actuators](https://img.shields.io/badge/Motors-NEMA%2017%20%2B%20TB6600-orange.svg)]()
[![Sensors](https://img.shields.io/badge/Encoders-AS5600%20(I2C)-green.svg)]()
[![Architecture](https://img.shields.io/badge/DoF-3%20Active%20(Expandable%20to%206)-brightgreen.svg)]()

A complete hardware-in-the-loop robotics stack for the **EB-15 3D-printed collaborative robotic arm** (ToolBoxRobotics design). This workspace contains the mechanical description (URDF/Xacro), kinematic modeling, RViz digital twin visualization, ROS 2 driver nodes, and embedded Arduino firmware for multi-axis closed-loop motion and automated limit switch calibration.

---

## Table of Contents
1. [Project Objectives](#1-project-objectives)
2. [System Architecture](#2-system-architecture)
   - [Mechanical & Kinematic Model](#mechanical--kinematic-model)
   - [Electronics & Actuation](#electronics--actuation)
   - [Sensors & Feedback](#sensors--feedback)
3. [Wiring & Hardware Pinout](#3-wiring--hardware-pinout)
4. [Firmware & Calibration Architecture](#4-firmware--calibration-architecture)
   - [Automated Dual-Limit Shoulder Calibration](#automated-dual-limit-shoulder-calibration)
   - [Serial Communication Protocol](#serial-communication-protocol)
5. [ROS 2 Software Stack](#5-ros-2-software-stack)
   - [Packages Overview](#packages-overview)
   - [Topic Interface](#topic-interface)
6. [Current Progress & Milestones](#6-current-progress--milestones)
7. [Task Roadmap (What to Do Next)](#7-task-roadmap-what-to-do-next)
8. [Setup & Quickstart Guide](#8-setup--quickstart-guide)
   - [Firmware Flashing](#1-firmware-flashing)
   - [Building ROS 2 Workspace](#2-building-ros-2-workspace)
   - [Running Hardware-in-the-Loop Digital Twin](#3-running-hardware-in-the-loop-digital-twin)
9. [Troubleshooting & Diagnostics](#9-troubleshooting--diagnostics)

---

## 1. Project Objectives

The primary goal of this project is to transform a desktop 3D-printable robotic arm into an open-source, industrially capable research and education platform with robust closed-loop control and ROS 2 Jazzy integration:

- **Accurate Kinematic Digital Twin**: Maintain a synchronized 3D representation in RViz matching physical robot dimensions, gear ratios, and joint orientation.
- **Hardware-in-the-Loop Feedback**: Continuous telemetry streaming from AS5600 magnetic encoders to ROS 2 at 20 Hz without encoder drift.
- **Autonomous Zero-Position Calibration**: Automatic dual-switch limit discovery for shoulder pitch (Joint 2), computing exact travel range and homing to midpoint (90° physical / 0.0 rad URDF frame).
- **Extensible 6-DoF Trajectory Control**: Foundation designed for 3 primary positioning axes (Base Yaw, Shoulder Pitch, Elbow Pitch) with direct scalability to a full 6-DoF arm with wrist roll/pitch/yaw and MoveIt 2 motion planning.

---

## 2. System Architecture

```
                       ┌──────────────────────────────────────┐
                       │           ROS 2 Workstation          │
                       │                                      │
                       │  [RViz2 3D Model]                    │
                       │         ▲                            │
                       │         │ /joint_states              │
                       │  [robot_state_publisher]             │
                       │         ▲                            │
                       │         │ /joint_states              │
                       │  [eb15_serial_node.py]               │
                       │    (Subscribes /eb15/joint_commands) │
                       └──────────────────┬───────────────────┘
                                          │ USB Serial (115200 Baud)
                                          │ Text Protocol ($STATE, $CMD)
                       ┌──────────────────▼───────────────────┐
                       │         Arduino Nano Firmware        │
                       │                                      │
                       │  - I2C Multiplexer (TCA9548A)       │
                       │  - Step/Dir Pulse Generation         │
                       │  - Limit Switch Interrupts/Polling   │
                       └───────────┬──────────────────┬───────┘
                                   │                  │
                Step / Dir / Enable│                  │ I2C (0x70, 0x36)
                                   ▼                  ▼
                    ┌─────────────────────┐    ┌─────────────────────┐
                    │  3x TB6600 Drivers  │    │ 3x AS5600 Encoders  │
                    │  3x NEMA 17 Steppers│    │ Base, Shoulder,     │
                    │  (EBA-17 Actuators) │    │ Elbow Joints        │
                    └─────────────────────┘    └─────────────────────┘
```

### Mechanical & Kinematic Model
- **Base Fixture**: Rigid base link with 001 base plate and 002 thrust washer.
- **Joint 1 (Base Pan / Yaw)**: Revolute Z-axis `[-π, +π]`, driven by NEMA 17 with EBA-17 planetary gearbox.
- **Joint 2 (Shoulder Pitch)**: Revolute Y-axis `[-π, +π]` (physical limit: 0° to 180°), driven by NEMA 17 with EBA-17-S high-torque planetary gearbox.
- **Joint 3 (Elbow Pitch)**: Revolute Y-axis `[-2.700, +2.700] rad` (`±154.7°`), constrained by dual limit switches to avoid collision with the base.
- **Wrist Link (`wrist_link`)**: Custom servo-driven wrist module (MG995) mounting flush onto the forearm tube at Z = 0.178m.
- **Tool Flange (`tool0`)**: End-effector mounting frame at wrist termination (Z = 0.2497m / 71.7mm above wrist base) for gripper attachment.

### Electronics & Actuation
- **Microcontroller**: Arduino Nano (ATmega328P).
- **Motor Drivers**: 3x TB6600 Microstep Drivers configured for 1/8 or 1/16 microstepping.
- **Motors**: 3x NEMA 17 Stepper Motors (42BYGH series).
- **Limit Switches**: 4x Mechanical snap-action microswitches:
  - 2x Joint 2 Shoulder (D3 = MIN 0°, D2 = MAX 180°)
  - 2x Joint 3 Elbow (A1 = MIN -2.700 rad, A2 = MAX +2.700 rad)
- **Servos**: 2x MG995 Servomotors:
  - D13 = Wrist Roll Servo
  - A0 = EBG-20 Gripper Servo

### Sensors & Feedback
- **Magnetic Rotary Encoders**: 3x AS5600 12-bit contactless encoders (`0.087°` resolution).
- **I2C Bus Multiplexer**: TCA9548A 8-channel I2C switch (`0x70`), routing separate I2C buses to identical AS5600 sensors (`0x36`).

---

## 3. Wiring & Hardware Pinout

### Arduino Nano Pinout

| Pin | Function / Connection | Connected To | Signal Type | Notes |
|---|---|---|---|---|
| **D2** | Shoulder Limit Switch MAX (180°) | Limit Switch COM / NO | Digital Input | Internal Pull-Up (`INPUT_PULLUP`, Active LOW) |
| **D3** | Shoulder Limit Switch MIN (0°) | Limit Switch COM / NO | Digital Input | Internal Pull-Up (`INPUT_PULLUP`, Active LOW) |
| **D4** | Motor 1 (Joint 1 Base) - STEP | TB6600 #1 `PUL+` / `PUL` | Digital Output | Pulse train |
| **D5** | Motor 1 (Joint 1 Base) - DIR | TB6600 #1 `DIR+` / `DIR` | Digital Output | Direction level |
| **D6** | Motor 1 (Joint 1 Base) - ENA | TB6600 #1 `ENA+` / `ENA` | Digital Output | Active LOW enable |
| **D7** | Motor 2 (Joint 2 Shoulder) - STEP | TB6600 #2 `PUL+` / `PUL` | Digital Output | Pulse train |
| **D8** | Motor 2 (Joint 2 Shoulder) - DIR | TB6600 #2 `DIR+` / `DIR` | Digital Output | Direction level |
| **D9** | Motor 2 (Joint 2 Shoulder) - ENA | TB6600 #2 `ENA+` / `ENA` | Digital Output | Active LOW enable |
| **D10**| Motor 3 (Joint 3 Elbow) - STEP | TB6600 #3 `PUL+` / `PUL` | Digital Output | Pulse train |
| **D11**| Motor 3 (Joint 3 Elbow) - DIR | TB6600 #3 `DIR+` / `DIR` | Digital Output | Direction level |
| **D12**| Motor 3 (Joint 3 Elbow) - ENA | TB6600 #3 `ENA+` / `ENA` | Digital Output | Active LOW enable |
| **D13**| Wrist Roll Servo (MG995) | Servo Signal PWM | Digital Output | 50 Hz PWM |
| **A0** | EBG-20 Gripper Servo (MG995) | Servo Signal PWM | Digital Output | 50 Hz PWM |
| **A1** | Elbow Limit Switch MIN (-2.700 rad) | Limit Switch COM / NO | Digital Input | Internal Pull-Up (`INPUT_PULLUP`, Active LOW) |
| **A2** | Elbow Limit Switch MAX (+2.700 rad) | Limit Switch COM / NO | Digital Input | Internal Pull-Up (`INPUT_PULLUP`, Active LOW) |
| **A4** | I2C SDA | TCA9548A SDA | I2C Bus | 400 kHz Fast-Mode |
| **A5** | I2C SCL | TCA9548A SCL | I2C Bus | 400 kHz Fast-Mode |
| **5V / GND** | Logic Power | TB6600 Signal GND, TCA9548A, Encoders | Power | 5V rail |

### TCA9548A I2C Channel Mapping

| Multiplexer Channel | Sensor | Target Joint | Register Address |
|---|---|---|---|
| **SD0 / SC0 (Ch 0)** | AS5600 #1 | Joint 1 (Base Yaw) | `0x36` |
| **SD1 / SC1 (Ch 1)** | AS5600 #2 | Joint 2 (Shoulder Pitch) | `0x36` |
| **SD2 / SC2 (Ch 2)** | AS5600 #3 | Joint 3 (Elbow Pitch) | `0x36` |

---

## 4. Firmware & Calibration Architecture

### Automated Dual-Limit Shoulder Calibration
The file [`eb15_auto_calibration.ino`](file:///home/juza/eb15_ws/firmware/eb15_auto_cal/eb15_auto_calibration.ino) handles hands-off calibration of Joint 2 upon boot:

```
  Phase 1             Phase 2              Phase 3             Phase 4              Phase 5
  Find MIN (0°)       Back-off & Zero      Find MAX (180°)     Move to Midpoint     Re-zero Midpoint
┌──────────────┐    ┌──────────────┐     ┌──────────────┐    ┌──────────────┐     ┌──────────────┐
│ Move toward  │───>│ Back off 200 │────>│ Step toward  │───>│ Return 50%   │────>│ Set encoder  │
│ MIN switch   │    │ steps, read  │     │ MAX switch,  │    │ of measured  │     │ offset = 0   │
│ until hit    │    │ MIN encoder  │     │ count steps  │    │ total steps  │     │ (URDF 0 rad) │
└──────────────┘    └──────────────┘     └──────────────┘    └──────────────┘     └──────────────┘
```

- **Physical MIN**: 0° (`-π/2 rad` = `-1.5708 rad` in URDF)
- **Midpoint / HOME**: 90° (`0.0 rad` in URDF)
- **Physical MAX**: 180° (`+π/2 rad` = `+1.5708 rad` in URDF)

### Serial Communication Protocol
The serial bridge operates at **115200 baud**:

#### Telemetry Packet (Arduino → ROS 2, ~20 Hz)
```
$STATE,<j1_rad>,<j2_rad>,<j3_rad>,<lim_min>,<lim_max>,<cal_flag>
```
*Example*: `$STATE,0.0000,0.0000,0.0000,0,0,1`

#### Command Packets (ROS 2 → Arduino)
- `$CMD,<j1_rad>,<j2_rad>,<j3_rad>\n`: Sets target positions in radians.
- `$HOME\n`: Triggers homing / auto-calibration sequence.
- `$SET_ZERO\n`: Overwrites current encoder readings as 0 radians.
- `$STATUS\n`: Requests formatted diagnostic status printout.

---

## 5. ROS 2 Software Stack

### Packages Overview
- **`eb15_description`**:
  - `urdf/eb15.urdf`: 3-DoF kinematic definition with mesh scale, origins, and mass inertial matrices.
  - `meshes/*.stl`: 3D printable meshes (001 through 015).
  - `launch/display.launch.py`: Simulation and joint slider test launch.
  - `rviz/display.rviz`: RViz camera, grid, and TF configuration.
- **`eb15_driver`**:
  - `eb15_driver/eb15_serial_node.py`: ROS 2 Node parsing serial packets, publishing `/joint_states`, and taking joint command goals.
  - `launch/hardware_display.launch.py`: Hardware-in-the-loop launch file combining serial bridge, `robot_state_publisher`, and RViz2.

### Topic Interface

| Topic Name | Message Type | Direction | Description |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | Published by Node | Real-time joint positions from AS5600 encoders |
| `/eb15/diagnostics` | `std_msgs/msg/String` | Published by Node | Firmware logs, calibration step counts, raw angles |
| `/eb15/joint_commands` | `std_msgs/msg/Float64MultiArray` | Subscribed by Node | Commanded target positions for `[j1, j2, j3]` in radians |
| `/eb15/home` | `std_msgs/msg/Empty` | Subscribed by Node | Trigger signal to initiate homing routine |
| `/eb15/set_zero` | `std_msgs/msg/Empty` | Subscribed by Node | Overwrite current encoder positions as 0.0 rad (home) |

---

## 6. Current Progress & Milestones

### Completed Milestones
- [x] **CAD to URDF Conversion**: Clean URDF kinematic tree created for base, shoulder, elbow, and tool flange.
- [x] **Mesh Alignment & Offsets**: Mesh visuals (001–015) aligned with joint axes and frame origins.
- [x] **Simulation Visualization**: RViz2 visualization working via `display.launch.py` with `joint_state_publisher_gui`.
- [x] **Multi-Channel Encoder Interface**: TCA9548A I2C multiplexing implemented and tested with AS5600 sensors.
- [x] **Diagnostic Firmware**: `eb15_diagnostic.ino` implemented to scan I2C buses and verify encoder signals.
- [x] **ROS 2 Serial Driver**: Python-based bidirectional serial node implemented with CycloneDDS configuration.
- [x] **Hardware-in-the-Loop Display**: `hardware_display.launch.py` connects physical robot telemetry directly to RViz.
- [x] **Automated Shoulder Calibration**: Multi-phase auto-calibration algorithm implemented in `eb15_auto_calibration.ino`, automatically measuring total step span, setting URDF-aligned zero at 90°, and configuring encoder direction.

---

## 7. Task Roadmap (What to Do Next)

### Immediate Tasks
1. **Upload & Validate Auto-Calibration on Physical Hardware**:
   - Upload [`eb15_auto_calibration.ino`](file:///home/juza/eb15_ws/firmware/eb15_auto_cal/eb15_auto_calibration.ino) to Arduino Nano.
   - Run the sequence and verify that motor moves toward MIN switch, backs off, counts steps to MAX switch, and centers at 90°.
2. **Step Interpolation & Motion Smoothing**:
   - Integrate an acceleration/deceleration trapezoidal or S-curve step planner on the Arduino or use AccelStepper / FastAccelStepper to prevent jerky stepper movements.
3. **Closed-Loop PID Tracking**:
   - Implement real-time closed-loop PID control inside Arduino loop comparing target vs. AS5600 encoder position.

### Medium-Term Tasks
4. **Homing for Joint 1 (Base) and Joint 3 (Elbow)**:
   - Add limit switches or hall-effect zero index sensors for Joint 1 and Joint 3.
5. **MoveIt 2 Integration**:
   - Generate an `eb15_moveit_config` package using MoveIt Setup Assistant.
   - Configure OMPL motion planning, kinematics solver (KDL or TRAC-IK), and trajectory execution controller.
6. **Hardware Controller / `ros2_control` Interface**:
   - Transition `eb15_serial_node.py` into a standardized `hardware_interface::SystemInterface` C++ plugin for `ros2_control`.

### Long-Term Tasks
7. **Expansion to 6-DoF**:
   - Add Wrist Pitch, Wrist Yaw, and Tool Roll joints from ToolBoxRobotics files.
   - Integrate vacuum or parallel gripper end-effector.

---

## 8. Setup & Quickstart Guide

### 1. Firmware Flashing
1. Open the Arduino IDE.
2. Open [`firmware/eb15_auto_cal/eb15_auto_calibration.ino`](file:///home/juza/eb15_ws/firmware/eb15_auto_cal/eb15_auto_calibration.ino).
3. Select board: **Arduino Nano** (Processor: ATmega328P or ATmega328P Old Bootloader).
4. Select port (e.g. `/dev/ttyUSB0`).
5. Click **Upload**.
6. Open Serial Monitor at **115200 baud** to observe the auto-calibration sequence.

### 2. Building ROS 2 Workspace
```bash
# Source ROS 2 Jazzy environment
source /opt/ros/jazzy/setup.bash

# Ensure CycloneDDS is selected
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Navigate to workspace and build
cd /home/juza/eb15_ws
colcon build --symlink-install

# Source workspace overlay
source install/setup.bash
```

### 3. Running Hardware-in-the-Loop Digital Twin
Once the Arduino is plugged into `/dev/ttyUSB0`:

```bash
# Grant serial permissions if needed
sudo chmod 666 /dev/ttyUSB0

# Launch the serial bridge, robot state publisher, and RViz
ros2 launch eb15_driver hardware_display.launch.py serial_port:=/dev/ttyUSB0
```
Move the robot joints physically; the 3D model in RViz will mirror the physical arm orientation in real time!

---

## 9. Troubleshooting & Diagnostics

| Symptom | Probable Cause | Solution |
|---|---|---|
| **Motor moves away from MIN switch during Phase 1** | Reverse motor direction logic | In `eb15_auto_calibration.ino`, swap `SHOULDER_DIR_TOWARD_MIN` and `SHOULDER_DIR_TOWARD_MAX`. |
| **I2C Bus Error / AS5600 raw angle reads 0** | SDA/SCL pull-ups or loose wiring | Check TCA9548A connections; run `eb15_diagnostic.ino` to scan all 8 channels. |
| **Serial port permission denied** | User not in `dialout` group | Run `sudo usermod -a -G dialout $USER` and log out/in, or run `sudo chmod 666 /dev/ttyUSB0`. |
| **FastCDR error / RViz crashes on ROS 2 Jazzy** | Default FastDDS bug on specific messages | Ensure `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is set in bash environment or launch file. |
| **Homing timeout reached** | Switch pin miswired or not triggered | Verify D2/D3 pins with digital multimeter or check `$STATE` telemetry switch bits. |
