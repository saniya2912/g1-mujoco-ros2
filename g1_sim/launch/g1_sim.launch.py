"""
Launch file for G1 MuJoCo simulation.

Sets DDS environment for loopback communication with unitree_mujoco
Python simulator, then starts the stand_g1 demo node.

Usage:
  ros2 launch g1_sim g1_sim.launch.py

Start the Python simulator first in a separate terminal:
  cd ~/Projects/g1-mujoco-ros2/unitree_mujoco/simulate_python
  python3 unitree_mujoco.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

CYCLONEDDS_URI = (
    '<CycloneDDS><Domain><General><Interfaces>'
    '<NetworkInterface name="lo" priority="default" multicast="default"/>'
    '</Interfaces></General></Domain></CycloneDDS>'
)


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '1'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', CYCLONEDDS_URI),

        Node(
            package='g1_sim',
            executable='stand_g1',
            name='g1_stand_demo',
            output='screen',
        ),
    ])
