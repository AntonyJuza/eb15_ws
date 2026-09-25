"""
MoveIt 2 Demo Launch for EB-15 5-DOF Arm
==========================================
Launches:
  - robot_state_publisher  (publishes /robot_description TF from URDF)
  - move_group              (MoveIt motion planning server)
  - rviz2                   (with MotionPlanning plugin and eb15 setup)
  - eb15_serial_node        (ActionServer for /arm_controller/follow_joint_trajectory)
  - joint_state_publisher_gui (optional manual sliders, disabled by default)
  - unity_bridge_node       (optional high-performance UDP bridge to Unity Digital Twin)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # ── Launch Arguments ───────────────────────────────────────
    use_hardware_arg = DeclareLaunchArgument(
        'use_hardware',
        default_value='true',
        description='Connect to physical EB-15 arm via serial driver'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Arduino Nano'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Launch joint_state_publisher_gui sliders window'
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

    # ── CycloneDDS RMW for reliable ROS 2 Jazzy message delivery ──
    set_rmw_env = SetEnvironmentVariable(
        name="RMW_IMPLEMENTATION",
        value="rmw_cyclonedds_cpp",
    )

    # ── Build MoveIt Configuration ─────────────────────────────
    pkg_moveit = FindPackageShare("eb15_moveit_config").find("eb15_moveit_config")
    rviz_config_file = os.path.join(pkg_moveit, "config", "moveit.rviz")

    moveit_config = (
        MoveItConfigsBuilder(robot_name="eb15", package_name="eb15_moveit_config")
        .robot_description(
            file_path=os.path.join(
                FindPackageShare("eb15_description").find("eb15_description"),
                "urdf",
                "eb15.urdf.xacro",
            )
        )
        .robot_description_semantic(file_path="srdf/eb15.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    # ── Robot State Publisher ──────────────────────────────────
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    # ── EB-15 Hardware Driver (FollowJointTrajectory Action Server) ─
    eb15_driver_node = Node(
        package="eb15_driver",
        executable="eb15_serial_node",
        name="eb15_serial_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_hardware")),
        parameters=[{
            "serial_port": LaunchConfiguration("serial_port"),
            "baud_rate": LaunchConfiguration("baud_rate"),
            "feedback_mode": False,
        }],
    )

    # ── Optional Joint State Publisher GUI ─────────────────────
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    # ── Unity Digital Twin Bridge Node ─────────────────────────
    unity_bridge_node = Node(
        package="eb15_driver",
        executable="unity_bridge_node",
        name="unity_bridge_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("unity")),
        parameters=[{
            "unity_ip": LaunchConfiguration("unity_ip"),
            "unity_port": LaunchConfiguration("unity_port"),
            "send_rate_hz": 50.0,
        }],
    )

    # ── MoveIt Move Group ──────────────────────────────────────
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )

    # ── RViz2 with MoveIt Config ───────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        use_hardware_arg,
        serial_port_arg,
        baud_rate_arg,
        gui_arg,
        unity_arg,
        unity_ip_arg,
        unity_port_arg,
        set_rmw_env,
        robot_state_publisher_node,
        eb15_driver_node,
        joint_state_publisher_gui_node,
        unity_bridge_node,
        move_group_node,
        rviz_node,
    ])
