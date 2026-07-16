#!/usr/bin/env python3

import json
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from vicpinky_interfaces.action import RunTask


def update_alignment_hold(aligned_since, now, hold_sec, is_aligned):
    """Track whether alignment stayed continuously valid long enough."""
    if not is_aligned:
        return None, False, 0.0

    if aligned_since is None:
        aligned_since = now

    held_sec = max(0.0, now - aligned_since)
    required_sec = max(0.0, hold_sec)
    return aligned_since, held_sec >= required_sec, held_sec


class DockAlignServer(Node):
    """Align the base to a fresh marker pose before completing the action."""

    def __init__(self):
        super().__init__('dock_align_server')

        self.declare_parameter('server_name', '/dock/align')
        self.declare_parameter('mock_mode', True)

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('offset_topic', '/tag/target_offset_x')
        self.declare_parameter('distance_topic', '/tag/target_distance')
        self.declare_parameter('marker_id_topic', '/tag/marker_id')

        self.declare_parameter('target_distance_m', 1.27)
        self.declare_parameter('target_lateral_m', 0.0)
        self.declare_parameter('aligned_hold_sec', 3.0)

        self.declare_parameter('x_tolerance_m', 0.07)
        self.declare_parameter('z_tolerance_m', 0.08)

        self.declare_parameter('linear_kp', 0.18)
        self.declare_parameter('angular_kp', 0.65)

        self.declare_parameter('max_linear_speed', 0.045)
        self.declare_parameter('max_angular_speed', 0.18)
        self.declare_parameter('min_linear_speed', 0.012)

        self.declare_parameter('search_angular_speed', -0.12)
        self.declare_parameter('marker_timeout_sec', 1.2)
        self.declare_parameter('align_timeout_sec', 90.0)

        self.server_name = self.get_parameter('server_name').value
        self.mock_mode = bool(self.get_parameter('mock_mode').value)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.offset_topic = self.get_parameter('offset_topic').value
        self.distance_topic = self.get_parameter('distance_topic').value
        self.marker_id_topic = self.get_parameter('marker_id_topic').value

        self.default_target_distance_m = float(self.get_parameter('target_distance_m').value)
        self.default_target_lateral_m = float(self.get_parameter('target_lateral_m').value)
        self.default_aligned_hold_sec = float(
            self.get_parameter('aligned_hold_sec').value
        )

        self.x_tolerance_m = float(self.get_parameter('x_tolerance_m').value)
        self.z_tolerance_m = float(self.get_parameter('z_tolerance_m').value)

        self.linear_kp = float(self.get_parameter('linear_kp').value)
        self.angular_kp = float(self.get_parameter('angular_kp').value)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)

        self.search_angular_speed = float(self.get_parameter('search_angular_speed').value)
        self.marker_timeout_sec = float(self.get_parameter('marker_timeout_sec').value)
        self.align_timeout_sec = float(self.get_parameter('align_timeout_sec').value)

        self.latest_offset_x = None
        self.latest_distance = None
        self.latest_marker_id = None
        self.latest_offset_time = 0.0
        self.latest_distance_time = 0.0
        self.latest_marker_id_time = 0.0

        self.cb_group = ReentrantCallbackGroup()

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.create_subscription(Float32, self.offset_topic, self.offset_callback, 10,
                                 callback_group=self.cb_group)
        self.create_subscription(Float32, self.distance_topic, self.distance_callback, 10,
                                 callback_group=self.cb_group)
        self.create_subscription(Int32, self.marker_id_topic, self.marker_id_callback, 10,
                                 callback_group=self.cb_group)

        self.action_server = ActionServer(
            self,
            RunTask,
            self.server_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info('Dock Align TVEC XZ Server Started.')
        self.get_logger().info(f'server_name: {self.server_name}')
        self.get_logger().info(f'mock_mode: {self.mock_mode}')
        self.get_logger().info(f'search_angular_speed: {self.search_angular_speed}')

    def offset_callback(self, msg):
        self.latest_offset_x = float(msg.data)
        self.latest_offset_time = time.monotonic()

    def distance_callback(self, msg):
        self.latest_distance = float(msg.data)
        self.latest_distance_time = time.monotonic()

    def marker_id_callback(self, msg):
        self.latest_marker_id = int(msg.data)
        self.latest_marker_id_time = time.monotonic()

    def goal_callback(self, goal_request):
        if goal_request.task_id not in ('dock_to_marker', 'align', 'tag_align'):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.stop_robot()
        return CancelResponse.ACCEPT

    def clamp(self, v, limit):
        return max(min(v, limit), -limit)

    def has_fresh_marker(self):
        now = time.monotonic()
        return (
            self.latest_offset_x is not None
            and self.latest_distance is not None
            and now - self.latest_offset_time <= self.marker_timeout_sec
            and now - self.latest_distance_time <= self.marker_timeout_sec
        )

    def has_fresh_marker_id(self, marker_id):
        return (
            self.latest_marker_id == marker_id
            and time.monotonic() - self.latest_marker_id_time
            <= self.marker_timeout_sec
        )

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        result = RunTask.Result()

        target_distance_m = self.default_target_distance_m
        target_lateral_m = self.default_target_lateral_m
        aligned_hold_sec = self.default_aligned_hold_sec

        try:
            if goal.extra_json:
                extra = json.loads(goal.extra_json)
                target_distance_m = float(extra.get('target_distance_m', target_distance_m))
                if 'target_distance_cm' in extra:
                    target_distance_m = float(extra['target_distance_cm']) / 100.0
                target_lateral_m = float(extra.get('target_lateral_m', target_lateral_m))
                aligned_hold_sec = max(
                    0.0,
                    float(extra.get('aligned_hold_sec', aligned_hold_sec)),
                )
        except Exception as e:
            result.success = False
            result.message = f'invalid extra_json: {e}'
            goal_handle.abort()
            return result

        if self.mock_mode:
            result.success = True
            result.message = 'mock aligned'
            goal_handle.succeed()
            return result

        start_time = time.monotonic()
        aligned_since = None

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                result.success = False
                result.message = 'canceled'
                goal_handle.canceled()
                return result

            if time.monotonic() - start_time > self.align_timeout_sec:
                self.stop_robot()
                result.success = False
                result.message = 'dock align timeout'
                goal_handle.abort()
                return result

            cmd = Twist()

            if not self.has_fresh_marker():
                aligned_since = None
                cmd.angular.z = self.search_angular_speed
                self.cmd_vel_pub.publish(cmd)
                self.publish_feedback(
                    goal_handle,
                    'SEARCH_MARKER',
                    0.10,
                    'searching marker to right',
                )
                time.sleep(0.05)
                continue

            if (
                goal.marker_id >= 0
                and not self.has_fresh_marker_id(goal.marker_id)
            ):
                aligned_since = None
                cmd.angular.z = self.search_angular_speed
                self.cmd_vel_pub.publish(cmd)
                self.publish_feedback(
                    goal_handle,
                    'SEARCH_MARKER',
                    0.15,
                    f'waiting marker {goal.marker_id}',
                )
                time.sleep(0.05)
                continue

            x_error = self.latest_offset_x - target_lateral_m
            z_error = self.latest_distance - target_distance_m

            x_ok = abs(x_error) <= self.x_tolerance_m
            z_ok = abs(z_error) <= self.z_tolerance_m
            is_aligned = x_ok and z_ok
            entering_hold = is_aligned and aligned_since is None
            aligned_since, hold_complete, held_sec = update_alignment_hold(
                aligned_since,
                time.monotonic(),
                aligned_hold_sec,
                is_aligned,
            )

            if is_aligned:
                self.stop_robot()

                if entering_hold and aligned_hold_sec > 0.0:
                    self.get_logger().info(
                        'Alignment entered tolerance; holding for '
                        f'{aligned_hold_sec:.1f} s'
                    )

                if not hold_complete:
                    hold_ratio = held_sec / max(aligned_hold_sec, 0.001)
                    detail = (
                        f'aligned for {held_sec:.1f}/'
                        f'{aligned_hold_sec:.1f} s; '
                        f'x={self.latest_offset_x:.3f}, '
                        f'z={self.latest_distance:.3f}'
                    )
                    self.publish_feedback(
                        goal_handle,
                        'ALIGN_STABLE_HOLD',
                        min(0.99, 0.90 + 0.09 * hold_ratio),
                        detail,
                    )
                    time.sleep(0.05)
                    continue

                result.success = True
                result.message = (
                    f'alignment stable for {aligned_hold_sec:.1f} s: '
                    f'x={self.latest_offset_x:.3f}, '
                    f'z={self.latest_distance:.3f}'
                )
                self.publish_feedback(goal_handle, 'ALIGNED', 1.0, result.message)
                goal_handle.succeed()
                return result

            # x_error > 0이면 마커가 오른쪽 → 오른쪽 회전: angular.z 음수
            cmd.angular.z = self.clamp(-self.angular_kp * x_error, self.max_angular_speed)

            # 너무 치우쳐 있으면 먼저 회전만
            if abs(x_error) > 0.18:
                cmd.linear.x = 0.0
                phase = 'X_ALIGN'
                progress = 0.45
            else:
                linear_cmd = self.linear_kp * z_error
                linear_cmd = self.clamp(linear_cmd, self.max_linear_speed)

                if abs(z_error) > self.z_tolerance_m and abs(linear_cmd) < self.min_linear_speed:
                    linear_cmd = self.min_linear_speed if z_error > 0 else -self.min_linear_speed

                cmd.linear.x = linear_cmd
                phase = 'DISTANCE_ALIGN'
                progress = 0.75

            detail = (
                f'x={self.latest_offset_x:.3f}, '
                f'z={self.latest_distance:.3f}, '
                f'x_err={x_error:.3f}, '
                f'z_err={z_error:.3f}, '
                f'cmd_v={cmd.linear.x:.3f}, '
                f'cmd_w={cmd.angular.z:.3f}'
            )

            self.cmd_vel_pub.publish(cmd)
            self.publish_feedback(goal_handle, phase, progress, detail)
            time.sleep(0.05)

        self.stop_robot()
        result.success = False
        result.message = 'rclpy shutdown'
        goal_handle.abort()
        return result

    def publish_feedback(self, goal_handle, phase, progress, detail):
        feedback = RunTask.Feedback()
        feedback.phase = phase
        feedback.progress = float(progress)
        feedback.detail = detail
        goal_handle.publish_feedback(feedback)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = DockAlignServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
