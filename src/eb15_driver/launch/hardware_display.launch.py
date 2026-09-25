import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_eb15_desc = get_package_share_directory('eb15_description')
    pkg_eb15_driver = get_package_share_directory('eb15_driver')

    urdf_file = os.path.join(pkg_eb15_desc, 'urdf', 'eb15.urdf')
    rviz_config_file = os.path.join(pkg_eb15_desc, 'rviz', 'display.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port of Arduino Nano (e.g. /dev/ttyUSB0, /dev/ttyACM0)'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate'
    )

    unity_arg = DeclareLaunchArgument(
        'unity',
        default_value='true',
        description='Enable Unity Digital Twin UDP bridge'
    )

    unity_ip_arg = DeclareLaunchArgument(
        'unity_ip',
        default_value='127.0.0.1',
        description='IP address of Unity Digital Twin simulation machine'
    )

    unity_port_arg = DeclareLaunchArgument(
        'unity_port',
        default_value='5005',
        description='UDP port on which Unity is listening'
    )

    # CycloneDDS environment variable to avoid FastCDR issues on ROS 2 Jazzy
    set_rmw_env = SetEnvironmentVariable(
        name='RMW_IMPLEMENTATION',
        value='rmw_cyclonedds_cpp'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc}]
    )

    # EB-15 Hardware Serial Driver Node
    eb15_driver_node = Node(
        package='eb15_driver',
        executable='eb15_serial_node',
        name='eb15_serial_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'joint_names': ['joint1', 'joint2', 'joint3', 'wrist_joint', 'gripper_left_joint'],
        }]
    )

    # Unity Digital Twin Bridge Node
    unity_bridge_node = Node(
        package='eb15_driver',
        executable='unity_bridge_node',
        name='unity_bridge_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('unity')),
        parameters=[{
            'unity_ip': LaunchConfiguration('unity_ip'),
            'unity_port': LaunchConfiguration('unity_port'),
            'send_rate_hz': 50.0,
        }]
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        unity_arg,
        unity_ip_arg,
        unity_port_arg,
        set_rmw_env,
        robot_state_publisher_node,
        eb15_driver_node,
        unity_bridge_node,
        rviz_node
    ])
