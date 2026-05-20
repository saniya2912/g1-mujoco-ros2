"""
RViz visualization bridge for G1 Edu U8 MuJoCo simulation.

Bridges /lowstate joint positions → /joint_states, then launches
robot_state_publisher + RViz so the G1 URDF model mirrors the simulation.

Usage:
  ros2 launch g1_sim g1_rviz.launch.py
  ros2 launch g1_sim g1_rviz.launch.py urdf_path:=/path/to/g1_29dof.urdf

Start the MuJoCo simulator first (separate terminal):
  DISPLAY=:1.0 env -u WAYLAND_DISPLAY python3 \\
    ~/Projects/g1-mujoco-ros2/unitree_mujoco/simulate_python/unitree_mujoco.py
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CYCLONEDDS_URI = (
    '<CycloneDDS><Domain><General><Interfaces>'
    '<NetworkInterface name="lo" priority="default" multicast="default"/>'
    '</Interfaces></General>'
    '<Discovery><MaxAutoParticipantIndex>50</MaxAutoParticipantIndex></Discovery>'
    '</Domain></CycloneDDS>'
)

DEFAULT_URDF = str(
    Path.home() / "Projects/g1-mujoco-ros2/unitree_ros/robots/g1_description/g1_29dof.urdf"
)

THIS_DIR = Path(__file__).parent.parent


def launch_nodes(context, *args, **kwargs):
    urdf_path = LaunchConfiguration("urdf_path").perform(context)
    rviz_config = str(THIS_DIR / "rviz" / "g1_default.rviz")

    with open(urdf_path) as f:
        robot_description = f.read()

    # URDF uses bare relative paths (e.g. meshes/pelvis.STL) with no package:// prefix.
    # Rewrite them to absolute file:// URIs so RViz can find the STL files.
    mesh_dir = str(Path(urdf_path).parent / "meshes")
    robot_description = robot_description.replace(
        'filename="meshes/',
        f'filename="file://{mesh_dir}/',
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="g1_sim",
            executable="joint_state_bridge.py",
            name="joint_state_bridge",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "1"),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        SetEnvironmentVariable("CYCLONEDDS_URI", CYCLONEDDS_URI),

        DeclareLaunchArgument(
            "urdf_path",
            default_value=DEFAULT_URDF,
            description="Path to the G1 29-DOF URDF file",
        ),

        OpaqueFunction(function=launch_nodes),
    ])
