#!/usr/bin/env python3
"""
Unity ROS 2 Bridge Node for EB-15 Robotic Arm Digital Twin
==========================================================
Bridges ROS 2 topics with the Unity Digital Twin simulation over a lightweight,
high-performance UDP/TCP socket.

Features:
- Subscribes to /joint_states and streams joint positions to Unity (radians -> Unity ArticulationBody)
- Subscribes to /eb15/measured_joint_states (physical encoder feedback)
- Receives teleoperation/IK commands from Unity and publishes to /eb15/joint_commands and /eb15/target_joint_states
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
            f'Starting EB-15 Unity Digital Twin Bridge -> Sending to {self.unity_ip}:{self.unity_port}, '
            f'Listening on port {self.listen_port}'
        )

        # UDP Sockets
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock_recv.bind(('0.0.0.0', self.listen_port))
            self.sock_recv.settimeout(0.1)
        except Exception as e:
            self.get_logger().warn(f'Could not bind receive port {self.listen_port}: {e}')

        # Current joint state cache
        self.joint_positions = {
            'joint1': 0.0,
            'joint2': 0.0,
            'joint3': 0.0,
            'wrist_joint': 0.0,
            'gripper_left_joint': 0.0,
            'gripper_right_joint': 0.0,
        }
        self.has_new_state = False
        self.lock = threading.Lock()

        # ROS 2 Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.measured_sub = self.create_subscription(
            JointState,
            '/eb15/measured_joint_states',
            self.joint_state_callback,
            10
        )

        # ROS 2 Publishers (Commands coming from Unity)
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/eb15/joint_commands', 10)
        self.target_pub = self.create_publisher(JointState, '/eb15/target_joint_states', 10)

        # Timer to send UDP packets to Unity at fixed rate
        timer_period = 1.0 / max(1.0, self.send_rate)
        self.timer = self.create_timer(timer_period, self.send_to_unity)

        # Background thread to receive incoming commands from Unity
        self.running = True
        self.recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.recv_thread.start()

    def joint_state_callback(self, msg: JointState):
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self.joint_positions:
                    self.joint_positions[name] = float(pos)
            self.has_new_state = True

    def send_to_unity(self):
        with self.lock:
            data = dict(self.joint_positions)

        try:
            payload = json.dumps(data).encode('utf-8')
            self.sock_send.sendto(payload, (self.unity_ip, self.unity_port))
        except Exception as e:
            self.get_logger().debug(f'Failed to send to Unity: {e}')

    def _receive_loop(self):
        """Listens for commands sent from Unity back to ROS."""
        while self.running:
            try:
                data, _ = self.sock_recv.recvfrom(2048)
                text = data.decode('utf-8').strip()
                if not text:
                    continue

                parsed = json.loads(text)
                # Check format
                if isinstance(parsed, dict):
                    # Publish as target JointState
                    js = JointState()
                    js.header.stamp = self.get_clock().now().to_msg()
                    names = []
                    positions = []
                    for k, v in parsed.items():
                        names.append(str(k))
                        positions.append(float(v))
                    js.name = names
                    js.position = positions
                    self.target_pub.publish(js)

                    # Also publish as Float64MultiArray if standard joint sequence
                    if 'joint1' in parsed and 'joint2' in parsed and 'joint3' in parsed:
                        arr = Float64MultiArray()
                        arr.data = [
                            parsed.get('joint1', 0.0),
                            parsed.get('joint2', 0.0),
                            parsed.get('joint3', 0.0),
                            parsed.get('wrist_joint', 0.0),
                            parsed.get('gripper_left_joint', 0.0),
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
