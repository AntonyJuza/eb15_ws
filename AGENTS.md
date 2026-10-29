# EB-15 Robot Arm — Agent Guidelines & Architecture Manual (AGENTS.md)

This document provides developer and AI agent guidelines, architecture details, conventions, and operational runbooks for the **EB-15 5-DOF Collaborative Robot Arm** workspace.

---

## 1. Workspace Architecture & Packages

| Package | Path | Purpose |
|---|---|---|
| **`eb15_description`** | `src/eb15_description` | Modular URDF/XACRO model, STL meshes, materials, and joint limit specifications. |
| **`eb15_driver`** | `src/eb15_driver` | Hardware communication layer: `eb15_serial_node` (USB serial bridge to Arduino Nano) and `unity_bridge_node` (UDP digital twin). |
| **`eb15_moveit_config`**| `src/eb15_moveit_config`| MoveIt 2 configuration, kinematics (KDL), OMPL motion planning pipeline, SRDF semantic models, and RViz display profiles. |
| **`firmware`** | `firmware/` | Microcontroller firmware for Arduino Nano (`eb15_arduino.ino`), AS5600 multi-turn magnetic encoder drivers, and limit switch homing routines. |

---

## 2. Kinematic Structure & Joint Limits

| Joint Name | Type | Axis | Active Range (URDF) | Padded Range (Limits) | Notes |
|---|---|---|---|---|---|
| `joint1` | Revolute | Z | `[-3.1416, 3.1416]` | `[-3.1416, 3.1416]` | Base Turntable |
| `joint2` | Revolute | Y | `[-1.5708, 1.5708]` | `[-1.5708, 1.5708]` | Shoulder (-90° to +90°) |
| `joint3` | Revolute | Y | `[-2.7000, 2.7000]` | `[-2.7000, 2.7000]` | Elbow (-154° to +154°) |
| `wrist_joint` | Revolute | Z | `[-1.5708, 1.5708]` | `[-1.5708, 1.5708]` | MG995 Wrist Roll Servo |
| `gripper_left_joint` | Prismatic | X | `[-0.016, 0.012]` | `[-0.018, 0.015]` | Active finger (-0.016m open, +0.012m closed) |
| `gripper_right_joint`| Prismatic | -X | `[-0.016, 0.012]` | `[-0.018, 0.015]` | **Passive mimic** of `gripper_left_joint` (multiplier 1.0) |

> [!IMPORTANT]
> **Mimic Joint Rule in SRDF:**  
> Never add `gripper_right_joint` to the `<group name="gripper">` planning group or named states in `eb15.srdf`. MoveIt automatically updates mimic joints from the primary active joint (`gripper_left_joint`). Adding mimic joints to planning groups causes MoveIt kinematic solver errors and start-state bounds rejection.

> [!TIP]
> **Padded Limits Rule:**  
> Always keep physical software limits (in `joint_limits.yaml` and URDF `<limit>`) slightly wider than target preset poses (e.g. `upper="0.015"` vs `closed="0.012"`). This prevents single-precision float conversion (`0.012000000104`) from triggering MoveIt's `CheckStartStateBounds` abort.

---

## 3. Communication Protocols

### PC (ROS 2) ⇄ Arduino Nano (Serial 115200 baud)
* **Stream Rate:** Throttled to ~22 Hz via dedicated worker thread in `eb15_serial_node.py` to prevent Arduino 64-byte serial RX buffer overflow.
* **Motion Command:**
  ```text
  $CMD,<q1>,<q2>,<q3>,<q_wrist>,<q_grip>\n
  ```
  *(Positions are in SI radians; gripper in meters stroke).*
* **Telemetry Message (Arduino -> PC at 20 Hz):**
  ```text
  $STATE,<meas_q1>,<meas_q2>,<meas_q3>,<sh_min>,<sh_max>,<el_min>,<el_max>,<cal_ok>\n
  ```
* **Priority Commands (Never dropped):**
  * `CAL\n` or `ALL\n` or `$HOME\n`: Triggers dual-joint homing calibration sequence.
  * `CAL2\n` / `CAL3\n`: Homing for Shoulder (J2) or Elbow (J3) individually.
  * `SET_ZERO\n`: Resets joint position accumulators to `0.0` at current pose in RAM (does not move motors).
  * `ENA\n` / `DIS\n`: Enable / disable TB6600 stepper drivers.
  * `STAT\n`: Dumps diagnostic table over serial.

### PC (ROS 2) ⇄ Unity Digital Twin (UDP 5005 / 5006)
* `unity_bridge_node` sends JSON joint telemetry to UDP `127.0.0.1:5005` at 50 Hz.
* Listens on UDP `5006` for external slider overrides.

---

## 4. Standard Operational Runbook

### Environment Setup
ROS 2 Jazzy requires CycloneDDS to avoid FastCDR deserialization issues:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Serial Port Access
```bash
sudo chmod 666 /dev/ttyUSB0
```

### Common Launch Commands
1. **Interactive Motion Planning with MoveIt 2 & Physical Arm:**
   ```bash
   ros2 launch eb15_moveit_config demo.launch.py serial_port:=/dev/ttyUSB0
   ```
2. **Hardware Mirroring & Diagnostics Only (no MoveIt):**
   ```bash
   ros2 launch eb15_driver hardware_display.launch.py serial_port:=/dev/ttyUSB0
   ```

### Calibration & Homing Commands
* **Run Full Arm Homing Calibration:**
  ```bash
  ros2 topic pub --once /eb15/calibrate std_msgs/msg/Empty "{}"
  ```
* **Zero Arm at Current Pose:**
  ```bash
  ros2 topic pub --once /eb15/set_zero std_msgs/msg/Empty "{}"
  ```
* **Raw Priority Commands to Arduino:**
  ```bash
  ros2 topic pub --once /eb15/send_cmd std_msgs/msg/String "{data: 'CAL2'}"   # Home Shoulder only
  ros2 topic pub --once /eb15/send_cmd std_msgs/msg/String "{data: 'CAL3'}"   # Home Elbow only
  ros2 topic pub --once /eb15/send_cmd std_msgs/msg/String "{data: 'STAT'}"   # Diagnostics dump
  ```

---

## 5. Known Hardware & Firmware Pitfalls

1. **I2C Bus Lockup on Arduino Nano:**
   * **Symptom:** Arm stops moving mid-trajectory; terminal reports `No Arduino telemetry for X.Xs`; subsequent serial commands receive no response.
   * **Root Cause:** Noise from stepper motor coils causes an I2C NACK on the AS5600/TCA9548A bus. Default AVR `Wire.h` hangs in an infinite loop.
   * **Solution:** `Wire.setWireTimeout(25000, true);` is configured in `eb15_arduino.ino` to automatically recover within 25ms.
2. **Limit Switch Noise Interlock:**
   * Limit switch pins (D2, D3, A1, A2) use internal pull-ups. Strong motor electrical transients can induce false LOW signals, causing `stepJointsTowardTarget()` to clamp target steps. Route switch cables away from high-current stepper phase wires.
3. **Build Command:**
   * Always compile with symlink-install after modifying Python or configuration files:
     ```bash
     colcon build --symlink-install
     ```
