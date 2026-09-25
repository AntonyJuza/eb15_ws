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

    feedback_arg = DeclareLaunchArgument(
        'feedback',
        default_value='false',
        description='If true, RViz shows only physical encoder feedback. If false, RViz moves immediately with GUI sliders.'
    )

    unity_arg = DeclareLaunchArgument(
        'unity',
        default_value='true',
        description='Launch Unity Digital Twin bridge node'
    )

    unity_ip_arg = DeclareLaunchArgument(
        'unity_ip',
        default_value='127.0.0.1',
        description='IP address of Unity workstation'
    )

    unity_port_arg = DeclareLaunchArgument(
        'unity_port',
        default_value='5005',
        description='UDP port of Unity Digital Twin'
    )

    feedback_conf = LaunchConfiguration('feedback')

    # CycloneDDS environment variable to avoid FastCDR issues on ROS 2 Jazzy
    set_rmw_env = SetEnvironmentVariable(
        name='RMW_IMPLEMENTATION',
        value='rmw_cyclonedds_cpp'
    )

    robot_description = ParameterValue(
        Command(['xacro ', default_xacro_path]),
        value_type=str
    )

    # Robot State Publisher (publishes TF from /joint_states)
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
            'feedback_mode': LaunchConfiguration('feedback'),
        }]
    )

    # Joint State Publisher GUI in Direct Mode (feedback:=false, default):
    # Publishes directly to /joint_states so RViz moves immediately when sliders are moved.
    joint_state_publisher_gui_direct = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(
            Command(['python3 -c "import sys; print(str(\'', LaunchConfiguration('gui'), '\').lower() == \'true\' and str(\'', feedback_conf, '\').lower() == \'false\')\"'])
        ),
        parameters=[{'robot_description': robot_description}],
    )

    # Joint State Publisher GUI in Closed-Loop Feedback Mode (feedback:=true):
    # Sliders publish to /eb15/target_joint_states, while /joint_states is driven by physical encoders.
    joint_state_publisher_gui_feedback = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(
            Command(['python3 -c "import sys; print(str(\'', LaunchConfiguration('gui'), '\').lower() == \'true\' and str(\'', feedback_conf, '\').lower() == \'true\')\"'])
        ),
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

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        gui_arg,
        feedback_arg,
        unity_arg,
        unity_ip_arg,
        unity_port_arg,
        set_rmw_env,
        robot_state_publisher_node,
        eb15_driver_node,
        joint_state_publisher_gui_direct,
        joint_state_publisher_gui_feedback,
        unity_bridge_node,
        rviz_node,
    ])
