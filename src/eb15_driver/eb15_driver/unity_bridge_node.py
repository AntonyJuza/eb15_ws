#!/usr/bin/env python3
"""
Unity ROS 2 Bridge Node for EB-15 Robotic Arm Digital Twin
==========================================================
Bridges ROS 2 topics with the Unity Digital Twin simulation over a lightweight,
high-performance UDP socket protocol.

Bi-directional Teleoperation & Physical Twin Architecture:
1. Physical Encoder Feedback (Real Arm -> Unity):
   - Subscribes to /eb15/measured_joint_states (hardware encoder feedback from Arduino Nano)
   - Streams measured joint positions to Unity on port 5005 (labeled with "is_encoder": true)
   - In Unity, the Solid Robot Arm Digital Twin mirrors these physical encoder values.

2. Ghost Target Preview & Commands (Unity -> Real Arm):
   - Receives target goal commands from Unity sliders/ghost preview on port 5006
   - Publishes to /eb15/joint_commands and /eb15/target_joint_states
   - Arduino Nano moves the physical motors, and encoders stream back physical movement!
"""

import json
import socket
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class UnityRosBridgeNode(Node):
    def __init__(self):
        super().__init__('unity_ros_bridge_node')

        # Declare parameters
        self.declare_parameter('unity_ip', '127.0.0.1')
        self.declare_parameter('unity_port', 5005)
        self.declare_parameter('ros_listen_port', 5006)
        self.declare_parameter('send_rate_hz', 50.0)

        self.unity_ip = self.get_parameter('unity_ip').get_parameter_value().string_value
        self.unity_port = self.get_parameter('unity_port').get_parameter_value().integer_value
        self.listen_port = self.get_parameter('ros_listen_port').get_parameter_value().integer_value
        self.send_rate = self.get_parameter('send_rate_hz').get_parameter_value().double_value

        self.get_logger().info(
            f'Starting EB-15 Unity Digital Twin Bridge -> Sending telemetry to {self.unity_ip}:{self.unity_port}, '
            f'Listening for commands on port {self.listen_port}'
        )

        # UDP Sockets
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock_recv.bind(('0.0.0.0', self.listen_port))
            self.sock_recv.settimeout(0.1)
        except Exception as e:
            self.get_logger().warn(f'Could not bind receive port {self.listen_port}: {e}')

        # Measured physical encoder state
        self.joint_positions = {
            'joint1': 0.0,
            'joint2': 0.0,
            'joint3': 0.0,
            'wrist_joint': 0.0,
            'gripper_left_joint': 0.0,
            'gripper_right_joint': 0.0,
        }
        self.is_real_encoder = False
        self.has_new_state = False
        self.lock = threading.Lock()

        # Primary Subscriber: Physical Hardware Encoder Telemetry
        self.measured_sub = self.create_subscription(
            JointState,
            '/eb15/measured_joint_states',
            self.measured_encoder_callback,
            10
        )

        # Secondary Subscriber: Generic joint states (e.g. MoveIt or RViz preview fallback)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.generic_joint_state_callback,
            10
        )

        # ROS 2 Publishers (Commands coming from Unity Ghost Preview / Sliders)
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/eb15/joint_commands', 10)
        self.target_pub = self.create_publisher(JointState, '/eb15/target_joint_states', 10)

        # Timer to send UDP packets to Unity at fixed rate
        timer_period = 1.0 / max(1.0, self.send_rate)
        self.timer = self.create_timer(timer_period, self.send_to_unity)

        # Background thread to receive incoming commands from Unity
        self.running = True
        self.recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.recv_thread.start()

    def measured_encoder_callback(self, msg: JointState):
        """Called when real Arduino encoder data arrives."""
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self.joint_positions:
                    self.joint_positions[name] = float(pos)
            self.is_real_encoder = True
            self.has_new_state = True

    def generic_joint_state_callback(self, msg: JointState):
        """Fallback when real hardware encoder is not active."""
        with self.lock:
            # Only use if real encoder feedback is not already established
            if not self.is_real_encoder:
                for name, pos in zip(msg.name, msg.position):
                    if name in self.joint_positions:
                        self.joint_positions[name] = float(pos)
                self.has_new_state = True

    def send_to_unity(self):
        with self.lock:
            data = dict(self.joint_positions)
            data['is_encoder'] = self.is_real_encoder

        try:
            payload = json.dumps(data).encode('utf-8')
            self.sock_send.sendto(payload, (self.unity_ip, self.unity_port))
        except Exception as e:
            self.get_logger().debug(f'Failed to send to Unity: {e}')

    def _receive_loop(self):
        """Listens for target commands sent from Unity sliders / ghost preview back to ROS."""
        while self.running:
            try:
                data, _ = self.sock_recv.recvfrom(2048)
                text = data.decode('utf-8').strip()
                if not text:
                    continue

                parsed = json.loads(text)
                if isinstance(parsed, dict):
                    # Publish as target JointState for ROS 2 nodes
                    js = JointState()
                    js.header.stamp = self.get_clock().now().to_msg()
                    names = []
                    positions = []
                    for k in ['joint1', 'joint2', 'joint3', 'wrist_joint', 'gripper_left_joint']:
                        if k in parsed:
                            names.append(k)
                            positions.append(float(parsed[k]))
                    js.name = names
                    js.position = positions
                    self.target_pub.publish(js)

                    # Also publish Float64MultiArray for eb15_serial_node queue_cmd
                    if 'joint1' in parsed and 'joint2' in parsed and 'joint3' in parsed:
                        arr = Float64MultiArray()
                        arr.data = [
                            float(parsed.get('joint1', 0.0)),
                            float(parsed.get('joint2', 0.0)),
                            float(parsed.get('joint3', 0.0)),
                            float(parsed.get('wrist_joint', 0.0)),
                            float(parsed.get('gripper_left_joint', 0.0)),
                        ]
                        self.cmd_pub.publish(arr)

            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().debug(f'Receive error: {e}')

    def destroy_node(self):
        self.running = False
        try:
            self.sock_send.close()
            self.sock_recv.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UnityRosBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
