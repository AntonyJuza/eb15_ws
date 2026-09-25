#!/usr/bin/env python3
"""
MoveIt 2 + Gazebo Harmonic Physics Simulation for EB-15 5-DOF Arm
================================================================
Launches:
  1. Gazebo Harmonic Simulation (physics, gravity, ground plane)
  2. ros2_control (joint_state_broadcaster, arm_controller, gripper_controller)
  3. MoveIt 2 move_group planning pipeline (OMPL)
  4. RViz2 with MotionPlanning plugin for interactive planning & execution
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_eb15_description = get_package_share_directory("eb15_description")
    pkg_eb15_moveit = get_package_share_directory("eb15_moveit_config")

    rviz_config_file = os.path.join(pkg_eb15_moveit, "config", "moveit.rviz")
    xacro_file = os.path.join(pkg_eb15_description, "urdf", "eb15.urdf.xacro")
    controllers_file = os.path.join(pkg_eb15_description, "config", "eb15_controllers.yaml")

    # Launch Arguments
    gui_arg = DeclareLaunchArgument(
        "gz_gui",
        default_value="true",
        description="Launch Gazebo graphical window"
    )

    # CycloneDDS for reliable ROS 2 Jazzy message delivery
    set_rmw_env = SetEnvironmentVariable(
        name="RMW_IMPLEMENTATION",
        value="rmw_cyclonedds_cpp"
    )

    # Build MoveIt Configuration for Simulation
    moveit_config = (
        MoveItConfigsBuilder(robot_name="eb15", package_name="eb15_moveit_config")
        .robot_description(
            file_path=xacro_file,
            mappings={
                "use_sim": "true",
                "controllers_yaml": controllers_file,
            }
        )
        .robot_description_semantic(file_path="config/eb15.srdf")
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

    # 1. Gazebo Simulation & Controllers Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_eb15_description, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "gui": LaunchConfiguration("gz_gui"),
            "use_sim_time": "true",
            "rviz": "false",  # We launch RViz with MoveIt below
        }.items()
    )

    # 2. MoveIt Move Group Node (with simulation time enabled)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    # 3. RViz2 with MoveIt MotionPlanning Plugin
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
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        gui_arg,
        set_rmw_env,
        gazebo_launch,
        move_group_node,
        rviz_node,
    ])
