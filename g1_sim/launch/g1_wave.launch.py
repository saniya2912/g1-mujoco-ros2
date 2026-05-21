"""
Launch file for the G1 arm-wave demo in MuJoCo.

Sets DDS environment for loopback communication with unitree_mujoco
Python simulator, then starts the wave_g1 demo node.

Usage:
  ros2 launch g1_sim g1_wave.launch.py

Start the Python simulator first in a separate terminal:
  cd ~/Projects/g1-mujoco-ros2/unitree_mujoco/simulate_python
  python3 unitree_mujoco.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

CYCLONEDDS_URI = (
    '<CycloneDDS><Domain><General><Interfaces>'
    '<NetworkInterface name="lo" priority="default" multicast="default"/>'
    '</Interfaces></General>'
    '<Discovery><MaxAutoParticipantIndex>50</MaxAutoParticipantIndex></Discovery>'
    '</Domain></CycloneDDS>'
)


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '1'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', CYCLONEDDS_URI),

        Node(
            package='g1_sim',
            executable='wave_g1',
            name='g1_arm_wave_demo',
            output='screen',
        ),
    ])
