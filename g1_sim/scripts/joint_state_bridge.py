#!/usr/bin/env python3
"""
Bridge node: /lowstate (unitree_hg/LowState) → /joint_states (sensor_msgs/JointState)

Reads the 29 motor positions from the MuJoCo simulator and republishes them
as a standard ROS2 JointState message so robot_state_publisher + RViz can
render the G1 robot model in real-time.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from unitree_hg.msg import LowState

# G1 29-DOF joint names in LowState motor_state order (indices 0-28)
G1_JOINT_NAMES = [
    "left_hip_pitch_joint",      # 0
    "left_hip_roll_joint",       # 1
    "left_hip_yaw_joint",        # 2
    "left_knee_joint",           # 3
    "left_ankle_pitch_joint",    # 4
    "left_ankle_roll_joint",     # 5
    "right_hip_pitch_joint",     # 6
    "right_hip_roll_joint",      # 7
    "right_hip_yaw_joint",       # 8
    "right_knee_joint",          # 9
    "right_ankle_pitch_joint",   # 10
    "right_ankle_roll_joint",    # 11
    "waist_yaw_joint",           # 12
    "waist_roll_joint",          # 13
    "waist_pitch_joint",         # 14
    "left_shoulder_pitch_joint", # 15
    "left_shoulder_roll_joint",  # 16
    "left_shoulder_yaw_joint",   # 17
    "left_elbow_joint",          # 18
    "left_wrist_roll_joint",     # 19
    "left_wrist_pitch_joint",    # 20
    "left_wrist_yaw_joint",      # 21
    "right_shoulder_pitch_joint",# 22
    "right_shoulder_roll_joint", # 23
    "right_shoulder_yaw_joint",  # 24
    "right_elbow_joint",         # 25
    "right_wrist_roll_joint",    # 26
    "right_wrist_pitch_joint",   # 27
    "right_wrist_yaw_joint",     # 28
]

# G1 pelvis height above ground when standing (metres)
G1_STANDING_HEIGHT = 0.787


class JointStateBridge(Node):
    def __init__(self):
        super().__init__("joint_state_bridge")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub_ = self.create_subscription(
            LowState, "/lowstate", self._lowstate_cb, qos
        )
        self.pub_ = self.create_publisher(JointState, "/joint_states", 10)

        # Publish a static TF world → base_link so RViz places the robot
        # at the correct standing height (pelvis reference frame)
        self.tf_broadcaster_ = StaticTransformBroadcaster(self)
        self._publish_base_tf()

        self.get_logger().info(
            "Joint state bridge ready — waiting for /lowstate from simulator."
        )

    def _publish_base_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "pelvis"   # G1 URDF root link
        t.transform.translation.z = G1_STANDING_HEIGHT
        t.transform.rotation.w = 1.0
        self.tf_broadcaster_.sendTransform(t)

    def _lowstate_cb(self, msg: LowState):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = G1_JOINT_NAMES
        js.position = [float(msg.motor_state[i].q) for i in range(len(G1_JOINT_NAMES))]
        js.velocity = [float(msg.motor_state[i].dq) for i in range(len(G1_JOINT_NAMES))]
        self.pub_.publish(js)


def main():
    rclpy.init()
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
