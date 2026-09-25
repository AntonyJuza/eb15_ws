#!/usr/bin/env python3
"""
Gazebo Harmonic Physics Simulation Launch for EB-15 Robotic Arm
==============================================================
Launches:
  1. Gazebo Harmonic (gz sim) with EB-15 physics world (gravity, ground plane, lighting)
  2. robot_state_publisher with simulated URDF and ros2_control hardware plugin
  3. ros_gz_sim entity spawner to insert EB-15 model into Gazebo
  4. ros_gz_bridge for clock synchronization (/clock)
  5. ros2_control controller spawners:
     - joint_state_broadcaster
     - arm_controller (JointTrajectoryController)
     - gripper_controller (GripperActionController)
  6. Optional RViz2 visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_eb15_description = get_package_share_directory('eb15_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Default paths
    default_world_path = os.path.join(pkg_eb15_description, 'worlds', 'eb15_world.sdf')
    default_rviz_path = os.path.join(pkg_eb15_description, 'rviz', 'display.rviz')
    controllers_file = os.path.join(pkg_eb15_description, 'config', 'eb15_controllers.yaml')
    xacro_file = os.path.join(pkg_eb15_description, 'urdf', 'eb15.urdf.xacro')

    # Launch Arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Path to Gazebo SDF world file'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run Gazebo headless (no GUI)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 alongside Gazebo'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock from Gazebo'
    )

    # Configure Gazebo Resource Paths for Mesh Resolution
    # Allows package://eb15_description/meshes/... to be resolved inside Gazebo
    package_parent_path = os.path.dirname(pkg_eb15_description)
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = f"{package_parent_path}:{existing_resource_path}" if existing_resource_path else package_parent_path

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )

    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=gz_resource_path
    )

    # CycloneDDS RMW for ROS 2 Jazzy
    set_rmw_env = SetEnvironmentVariable(
        name='RMW_IMPLEMENTATION',
        value='rmw_cyclonedds_cpp'
    )

    # Generate robot_description using xacro with use_sim:=true
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        xacro_file,
        ' ',
        'use_sim:=true',
        ' ',
        f'controllers_yaml:={controllers_file}',
    ])
    robot_description = {'robot_description': robot_description_content}

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Gazebo Sim Launch:
    # -r = run immediately on start (not paused)
    # -v 3 = verbose level 3 for clean diagnostics
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 3 ', LaunchConfiguration('world')]
        }.items()
    )

    # Spawn EB-15 Robot Entity into Gazebo World
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'eb15',
            '-z', '0.0',
            '-allow_renaming', 'true'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Clock Bridge (Gazebo -> ROS 2)
    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Controller Spawners (spawned once robot is created in Gazebo)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Ensure controllers spawn sequentially after robot entity is spawned
    delay_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )

    delay_arm_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner]
        )
    )

    # Optional RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        world_arg,
        gui_arg,
        rviz_arg,
        use_sim_time_arg,
        set_gz_resource_path,
        set_ign_resource_path,
        set_rmw_env,
        robot_state_publisher_node,
        gz_sim_launch,
        spawn_robot_node,
        clock_bridge_node,
        delay_broadcaster_after_spawn,
        delay_arm_after_broadcaster,
        rviz_node,
    ])
