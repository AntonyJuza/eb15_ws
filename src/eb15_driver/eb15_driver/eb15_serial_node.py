#!/usr/bin/env python3
import time
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Empty, String


class EB15SerialBridge(Node):
    def __init__(self):
        super().__init__('eb15_serial_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3'])

        self.port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value

        self.get_logger().info(f'Initializing EB-15 Serial Bridge on {self.port_name} @ {self.baud_rate} baud')

        # Joint State Publisher
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Raw Telemetry & Diagnostic Publisher
        self.diag_pub = self.create_publisher(String, '/eb15/diagnostics', 10)

        # Joint Command Subscriber
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/eb15/joint_commands',
            self.command_callback,
            10
        )

        # Home trigger subscriber
        self.home_sub = self.create_subscription(
            Empty,
            '/eb15/home',
            self.home_callback,
            10
        )

        # Zero offset trigger subscriber
        self.zero_sub = self.create_subscription(
            Empty,
            '/eb15/set_zero',
            self.set_zero_callback,
            10
        )

        # Serial Connection
        self.ser = None
        self.connect_serial()

        # Timer for polling serial input
        self.poll_timer = self.create_timer(0.01, self.poll_serial) # 100 Hz poll

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port_name, self.baud_rate, timeout=0.05)
            self.get_logger().info(f'Successfully connected to Arduino on {self.port_name}')
            # Allow Arduino time to reset after DTR toggle
            time.sleep(2.0)
        except serial.SerialException as e:
            self.get_logger().warn(f'Could not open serial port {self.port_name}: {e}. Retrying in background...')
            self.ser = None

    def poll_serial(self):
        if self.ser is None or not self.ser.is_open:
            self.connect_serial()
            return

        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                if line.startswith('$STATE,'):
                    self.parse_and_publish_state(line)
                elif line.startswith('#'):
                    # Diagnostic / Log message from Arduino
                    self.get_logger().info(f'Arduino: {line}')
                    diag_msg = String()
                    diag_msg.data = line
                    self.diag_pub.publish(diag_msg)
        except serial.SerialException as e:
            self.get_logger().error(f'Serial communication error: {e}')
            self.ser.close()
            self.ser = None

    def parse_and_publish_state(self, line):
        # Format: $STATE,<j1>,<j2>,<j3>,<lim_min>,<lim_max>
        parts = line.split(',')
        if len(parts) >= 6:
            try:
                q1 = float(parts[1])
                q2 = float(parts[2])
                q3 = float(parts[3])
                lim_min = int(parts[4])
                lim_max = int(parts[5])

                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = self.joint_names
                positions = [q1, q2, q3]
                while len(positions) < len(self.joint_names):
                    positions.append(0.0)
                msg.position = positions
                self.joint_pub.publish(msg)

            except ValueError as e:
                self.get_logger().warn(f'Malformed telemetry packet: {line} ({e})')

    def command_callback(self, msg: Float64MultiArray):
        if self.ser and self.ser.is_open:
            if len(msg.data) >= 3:
                cmd_str = f'$CMD,{msg.data[0]:.4f},{msg.data[1]:.4f},{msg.data[2]:.4f}\n'
                self.ser.write(cmd_str.encode('utf-8'))
                self.get_logger().debug(f'Sent command: {cmd_str.strip()}')

    def home_callback(self, msg: Empty):
        if self.ser and self.ser.is_open:
            self.ser.write(b'$HOME\n')
            self.get_logger().info('Sent $HOME command to Arduino')

    def set_zero_callback(self, msg: Empty):
        if self.ser and self.ser.is_open:
            self.ser.write(b'$SET_ZERO\n')
            self.get_logger().info('Sent $SET_ZERO command to Arduino')


def main(args=None):
    rclpy.init(args=args)
    node = EB15SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
