#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FakeJointStateNode(Node):

    def __init__(self):
        super().__init__('fake_joint_state_node')

        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        self.joint_names = [
            'base_joint',
            'arm_joint_1',
            'arm_joint_2',
            'arm_joint_3',
            'arm_joint_4',
            'finger_1_base_joint',
            'finger_1_middle_joint',
            'finger_1_tip_joint',
            'finger_2_base_joint',
            'finger_2_middle_joint',
            'finger_2_tip_joint',
            'finger_3_base_joint',
            'finger_3_middle_joint',
            'finger_3_tip_joint',
        ]

        self.positions = [
            -1.57079633,
            -1.50098316,
            -1.38055544,
            -1.59697627,
            -1.57079633,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
        ]

        self.timer = self.create_timer(0.02, self.publish_joint_states)
        self.get_logger().info('Fake joint state publisher started')

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeJointStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
