#!/usr/bin/env python3
"""
EB-15 Serial Bridge Node with MoveIt 2 Trajectory Action Server
===============================================================
- Robust thread-safe serial communication with Arduino Nano
- Dedicated background worker for serial I/O (prevents concurrent access crashes)
- Implements FollowJointTrajectory Action Server for MoveIt 2 execution
- Implements GripperCommand Action Server for end-effector control
- Subscribes to /joint_states for direct manual jogging
- Publishes /joint_states or /eb15/measured_joint_states
"""

import queue
import threading
import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Empty, String
from control_msgs.action import FollowJointTrajectory, GripperCommand


class EB15SerialBridge(Node):
    def __init__(self):
        super().__init__('eb15_serial_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3', 'wrist_joint', 'gripper_left_joint'])
        self.declare_parameter('feedback_mode', False)

        self.port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        feedback_raw = self.get_parameter('feedback_mode').value
        if isinstance(feedback_raw, str):
            self.feedback_mode = feedback_raw.lower() in ('true', '1', 'yes')
        else:
            self.feedback_mode = bool(feedback_raw)

        self.get_logger().info(
            f'Initializing EB-15 Hardware Bridge on {self.port_name} @ {self.baud_rate} baud '
            f'(feedback_mode={self.feedback_mode})'
        )

        # Joint positions
        self.cmd_q1 = 0.0
        self.cmd_q2 = 0.0
        self.cmd_q3 = 0.0
        self.cmd_wrist = 0.0
        self.cmd_grip = 0.0

        self.meas_q1 = 0.0
        self.meas_q2 = 0.0
        self.meas_q3 = 0.0

        self.is_executing_trajectory = False

        # Thread-safe queue for outgoing serial messages
        self.out_queue = queue.Queue(maxsize=10)
        self.ser = None
        self.running = True

        # Callback group for concurrent action + timer execution
        self.cb_group = ReentrantCallbackGroup()

        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/eb15/measured_joint_states', 10)
        self.direct_joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.diag_pub = self.create_publisher(String, '/eb15/diagnostics', 10)

        # Subscribers
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/eb15/joint_commands',
            self.command_callback,
            10,
            callback_group=self.cb_group,
        )

        sub_topic = '/eb15/target_joint_states' if self.feedback_mode else '/joint_states'
        self.target_sub = self.create_subscription(
            JointState,
            sub_topic,
            self.target_joint_state_callback,
            10,
            callback_group=self.cb_group,
        )

        self.home_sub = self.create_subscription(Empty, '/eb15/home', self.home_callback, 10)
        self.zero_sub = self.create_subscription(Empty, '/eb15/set_zero', self.set_zero_callback, 10)
        self.cal_sub = self.create_subscription(Empty, '/eb15/calibrate', self.calibrate_callback, 10)
        self.raw_sub = self.create_subscription(String, '/eb15/send_cmd', self.raw_cmd_callback, 10)

        # MoveIt Action Servers
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_trajectory_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self._gripper_server = ActionServer(
            self,
            GripperCommand,
            '/gripper_controller/gripper_action',
            execute_callback=self.execute_gripper_callback,
            callback_group=self.cb_group,
        )

        # Heartbeat timer at 20 Hz (keeps RViz and MoveIt informed of robot pose)
        self.heartbeat_timer = self.create_timer(0.05, self.publish_heartbeat, callback_group=self.cb_group)

        # Start dedicated background thread for serial I/O
        self.serial_thread = threading.Thread(target=self._serial_worker, daemon=True)
        self.serial_thread.start()

    # ── Serial Worker (Single Thread for all Serial I/O) ─────────

    def _serial_worker(self):
        while self.running and rclpy.ok():
            # Connect if needed
            if self.ser is None or not self.ser.is_open:
                try:
                    self.ser = serial.Serial(self.port_name, self.baud_rate, timeout=0.05)
                    self.get_logger().info(f'Connected to Arduino Nano on {self.port_name}')
                    time.sleep(2.0)
                except serial.SerialException as e:
                    self.get_logger().warn(f'Waiting for {self.port_name}: {e}', throttle_duration_sec=3.0)
                    time.sleep(1.0)
                    continue

            try:
                # 1. Send any queued commands
                while not self.out_queue.empty():
                    cmd = self.out_queue.get_nowait()
                    self.ser.write(cmd.encode('utf-8'))

                # 2. Read incoming telemetry
                while self.ser.in_waiting > 0:
                    raw = self.ser.readline()
                    line = raw.decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue
                    if line.startswith('$STATE,'):
                        self._parse_state(line)
                    elif line.startswith('#'):
                        self.get_logger().info(f'Arduino: {line}')
                        diag = String()
                        diag.data = line
                        self.diag_pub.publish(diag)

                time.sleep(0.005)

            except serial.SerialException as e:
                self.get_logger().error(f'Serial communication error: {e}')
                try:
                    if self.ser:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None
                time.sleep(1.0)

    def _parse_state(self, line: str):
        parts = line.split(',')
        if len(parts) >= 4:
            try:
                self.meas_q1 = float(parts[1])
                self.meas_q2 = float(parts[2])
                self.meas_q3 = float(parts[3])
                meas_wrist = float(parts[4]) if len(parts) > 4 else self.cmd_wrist
                meas_grip = float(parts[5]) if len(parts) > 5 else self.cmd_grip

                # Always publish physical encoder feedback to /eb15/measured_joint_states
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = self.joint_names
                msg.position = [
                    self.meas_q1,
                    self.meas_q2,
                    self.meas_q3,
                    meas_wrist,
                    meas_grip,
                ]
                self.joint_pub.publish(msg)

                # If feedback_mode is active, also publish directly to /joint_states
                if self.feedback_mode:
                    self.direct_joint_pub.publish(msg)
            except ValueError:
                pass

    def queue_cmd(self, q1: float, q2: float, q3: float, q_wrist: float = 0.0, q_grip: float = 0.0):
        self.cmd_q1 = q1
        self.cmd_q2 = q2
        self.cmd_q3 = q3
        self.cmd_wrist = q_wrist
        self.cmd_grip = q_grip

        cmd_str = f'$CMD,{q1:.4f},{q2:.4f},{q3:.4f},{q_wrist:.4f},{q_grip:.4f}\n'
        try:
            # Drop older commands if buffer is full so we never lag behind
            if self.out_queue.full():
                try:
                    self.out_queue.get_nowait()
                except queue.Empty:
                    pass
            self.out_queue.put_nowait(cmd_str)
            self.get_logger().info(f'Commanded: {cmd_str.strip()}', throttle_duration_sec=0.5)
        except queue.Full:
            pass

    # ── State Publishing & Manual Jogging ────────────────────────

    def publish_heartbeat(self):
        if not self.feedback_mode and not self.is_executing_trajectory:
            self.publish_state(self.cmd_q1, self.cmd_q2, self.cmd_q3, self.cmd_wrist, self.cmd_grip)

    def publish_state(self, q1, q2, q3, q_wrist, q_grip):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [q1, q2, q3, q_wrist, q_grip]
        self.direct_joint_pub.publish(msg)

    def target_joint_state_callback(self, msg: JointState):
        if self.is_executing_trajectory:
            return  # MoveIt execution takes priority

        name_to_pos = dict(zip(msg.name, msg.position))
        q1 = name_to_pos.get('joint1', self.cmd_q1)
        q2 = name_to_pos.get('joint2', self.cmd_q2)
        q3 = name_to_pos.get('joint3', self.cmd_q3)
        q_wrist = name_to_pos.get('wrist_joint', self.cmd_wrist)
        q_grip = name_to_pos.get('gripper_left_joint', self.cmd_grip)
        self.queue_cmd(q1, q2, q3, q_wrist, q_grip)

    def command_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 5:
            self.queue_cmd(msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4])
        elif len(msg.data) >= 3:
            self.queue_cmd(msg.data[0], msg.data[1], msg.data[2])

    def home_callback(self, msg: Empty):
        try:
            self.out_queue.put_nowait('CAL\n')
            self.get_logger().info('Sent CAL (Home) command to Arduino')
        except queue.Full:
            pass

    def set_zero_callback(self, msg: Empty):
        try:
            self.out_queue.put_nowait('SET_ZERO\n')
            self.get_logger().info('Sent SET_ZERO command to Arduino')
        except queue.Full:
            pass

    def calibrate_callback(self, msg: Empty):
        try:
            self.out_queue.put_nowait('CAL\n')
            self.get_logger().info('Sent CAL (Full Arm Calibration) command to Arduino')
        except queue.Full:
            pass

    def raw_cmd_callback(self, msg: String):
        cmd = msg.data.strip().upper()
        try:
            self.out_queue.put_nowait(f'{cmd}\n')
            self.get_logger().info(f'Sent raw command to Arduino: {cmd}')
        except queue.Full:
            pass

    # ── MoveIt 2 FollowJointTrajectory Callbacks ─────────────────

    def goal_callback(self, goal_request):
        self.get_logger().info('MoveIt FollowJointTrajectory goal received -> ACCEPTED')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('MoveIt Trajectory Cancel requested')
        return CancelResponse.ACCEPT

    def execute_trajectory_callback(self, goal_handle):
        import numpy as np

        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names
        points = trajectory.points

        if not points:
            goal_handle.succeed()
            res = FollowJointTrajectory.Result()
            res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return res

        orig_times = [pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9 for pt in points]
        if orig_times[0] > 0.0:
            orig_times.insert(0, 0.0)
            points = [points[0]] + list(points)

        total_duration = orig_times[-1]
        if total_duration <= 0.0:
            goal_handle.succeed()
            res = FollowJointTrajectory.Result()
            res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return res

        # Generate dense 40 Hz sample grid (every 25 ms)
        sample_times = np.arange(0.0, total_duration, 0.025)
        if len(sample_times) == 0 or sample_times[-1] < total_duration:
            sample_times = np.append(sample_times, total_duration)

        # Smoothly interpolate each joint
        joint_series = {}
        for j_idx, j_name in enumerate(joint_names):
            y = [pt.positions[j_idx] for pt in points]
            joint_series[j_name] = np.interp(sample_times, orig_times, y)

        self.get_logger().info(
            f'Smoothing MoveIt trajectory: {len(points)} waypoints -> {len(sample_times)} continuous steps (40 Hz) over {total_duration:.2f}s'
        )

        self.is_executing_trajectory = True
        start_time = time.time()

        for t_step, cur_t in enumerate(sample_times):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.is_executing_trajectory = False
                self.get_logger().warn('Trajectory execution canceled')
                res = FollowJointTrajectory.Result()
                return res

            target_time = start_time + cur_t
            sleep_dt = target_time - time.time()
            if sleep_dt > 0:
                time.sleep(sleep_dt)

            q1 = float(joint_series['joint1'][t_step]) if 'joint1' in joint_series else self.cmd_q1
            q2 = float(joint_series['joint2'][t_step]) if 'joint2' in joint_series else self.cmd_q2
            q3 = float(joint_series['joint3'][t_step]) if 'joint3' in joint_series else self.cmd_q3
            q_wrist = float(joint_series['wrist_joint'][t_step]) if 'wrist_joint' in joint_series else self.cmd_wrist
            q_grip = self.cmd_grip

            self.queue_cmd(q1, q2, q3, q_wrist, q_grip)
            self.publish_state(q1, q2, q3, q_wrist, q_grip)

        self.is_executing_trajectory = False
        goal_handle.succeed()
        self.get_logger().info('MoveIt trajectory completed smoothly on physical EB-15 arm!')

        res = FollowJointTrajectory.Result()
        res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return res

    # ── Gripper Command Callbacks ────────────────────────────────

    def execute_gripper_callback(self, goal_handle):
        cmd = goal_handle.request.command
        position = cmd.position
        self.get_logger().info(f'Gripper command received: {position}')

        self.queue_cmd(self.cmd_q1, self.cmd_q2, self.cmd_q3, self.cmd_wrist, position)
        self.publish_state(self.cmd_q1, self.cmd_q2, self.cmd_q3, self.cmd_wrist, position)

        goal_handle.succeed()
        res = GripperCommand.Result()
        res.position = position
        res.reached_goal = True
        return res


def main(args=None):
    rclpy.init(args=args)
    node = EB15SerialBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
