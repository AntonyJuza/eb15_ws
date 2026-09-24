import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_eb15_desc = FindPackageShare('eb15_description')
    pkg_eb15_driver = FindPackageShare('eb15_driver')

    default_xacro_path = PathJoinSubstitution([pkg_eb15_desc, 'urdf', 'eb15.urdf.xacro'])
    default_rviz_path = PathJoinSubstitution([pkg_eb15_desc, 'rviz', 'display.rviz'])

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

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch joint_state_publisher_gui slider control panel'
    )

    # CycloneDDS environment variable to avoid FastCDR issues on ROS 2 Jazzy
    set_rmw_env = SetEnvironmentVariable(
        name='RMW_IMPLEMENTATION',
        value='rmw_cyclonedds_cpp'
    )

    robot_description = ParameterValue(
        Command(['xacro ', default_xacro_path]),
        value_type=str
    )

    # Robot State Publisher (publishes TF from real physical /joint_states)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}]
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

    # Joint State Publisher GUI (Slider controls remapped to target topic)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
        parameters=[{'robot_description': robot_description}],
        remappings=[
            ('joint_states', '/eb15/target_joint_states'),
        ]
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_path]
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        gui_arg,
        set_rmw_env,
        robot_state_publisher_node,
        eb15_driver_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
