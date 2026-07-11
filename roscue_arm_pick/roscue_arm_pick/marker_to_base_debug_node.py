#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener


class MarkerToBaseDebugNode(Node):

    def __init__(self):
        super().__init__('marker_to_base_debug_node')

        self.target_frame = 'base_link'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PoseStamped,
            '/detected_marker_pose',
            self.marker_pose_callback,
            10
        )

        self.get_logger().info('Marker to base debug node started')
        self.get_logger().info('Listening: /detected_marker_pose')
        self.get_logger().info('Target frame: base_link')

    def marker_pose_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time()
            )

            pose_base = do_transform_pose(msg.pose, transform)

            p = pose_base.position
            q = pose_base.orientation

            self.get_logger().info(
                f'Marker in base_link | '
                f'x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f} | '
                f'qx={q.x:.3f}, qy={q.y:.3f}, qz={q.z:.3f}, qw={q.w:.3f}'
            )

        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')


def main():
    rclpy.init()
    node = MarkerToBaseDebugNode()

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
