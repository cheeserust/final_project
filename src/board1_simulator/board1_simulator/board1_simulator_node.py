"""ROS 2 node that simulates STM32 Board1/2/3 on SocketCAN."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from arm_can_bridge.can_protocol import BOARD_ID_BOARD1
from arm_can_bridge.can_protocol import CAN_ID_BOARD2_POSITION_COMMAND
from arm_can_bridge.can_protocol import CAN_ID_BOARD3_SERVO_COMMAND
from arm_can_bridge.socketcan_transport import SocketCanTransport

from .model import Board1SimulatorModel
from .model import make_board2_simulator_model
from .model import make_board3_simulator_model


class Board1SimulatorNode(Node):
    """Publish 0x201/0x202/0x203 status and consume command frames."""

    def __init__(self) -> None:
        """Create the simulator model, transport, and timers."""
        super().__init__('board1_simulator')

        self.declare_parameter('can_interface', 'vcan0')
        self.declare_parameter('status_period_ms', 100)
        self.declare_parameter('feedback_period_ms', 20)
        self.declare_parameter('tick_period_ms', 10)
        self.declare_parameter('homing_duration_ms', 500)
        self.declare_parameter('queue_capacity', 32)

        self._can_interface = (
            self.get_parameter('can_interface')
            .get_parameter_value()
            .string_value
        )
        status_period_ms = (
            self.get_parameter('status_period_ms')
            .get_parameter_value()
            .integer_value
        )
        feedback_period_ms = (
            self.get_parameter('feedback_period_ms')
            .get_parameter_value()
            .integer_value
        )
        tick_period_ms = (
            self.get_parameter('tick_period_ms')
            .get_parameter_value()
            .integer_value
        )
        homing_duration_ms = (
            self.get_parameter('homing_duration_ms')
            .get_parameter_value()
            .integer_value
        )
        queue_capacity = (
            self.get_parameter('queue_capacity')
            .get_parameter_value()
            .integer_value
        )

        homing_duration_s = float(homing_duration_ms) / 1000.0
        self._models = [
            Board1SimulatorModel(
                queue_capacity=int(queue_capacity),
                homing_duration_s=homing_duration_s,
            ),
            make_board2_simulator_model(
                queue_capacity=int(queue_capacity),
                homing_duration_s=homing_duration_s,
            ),
            make_board3_simulator_model(
                queue_capacity=int(queue_capacity),
            ),
        ]
        self._last_tick_time = time.monotonic()

        self._transport = SocketCanTransport(
            interface_name=self._can_interface,
            receive_ids=(
                0x001,
                0x010,
                0x020,
                0x030,
                0x101,
                CAN_ID_BOARD2_POSITION_COMMAND,
                CAN_ID_BOARD3_SERVO_COMMAND,
            ),
            frame_callback=self._handle_frame,
            error_callback=self._handle_transport_error,
        )
        self._transport.open()

        self._tick_timer = self.create_timer(
            float(tick_period_ms) / 1000.0,
            self._tick,
        )
        self._status_timer = self.create_timer(
            float(status_period_ms) / 1000.0,
            self._send_status,
        )
        self._feedback_timer = self.create_timer(
            float(feedback_period_ms) / 1000.0,
            self._send_position_feedback,
        )

        self.get_logger().info(
            f'Board1/Board2/Board3 simulator started on '
            f'{self._can_interface}'
        )

    def _handle_frame(self, frame) -> None:
        self.get_logger().info(
            f'Received CAN frame: id={frame.can_id:#04x}, '
            f'data={frame.data.hex().upper()}'
        )

        for model in self._models:
            if model.handle_frame(frame):
                self._send_status(model)

    def _handle_transport_error(self, error: Exception) -> None:
        self.get_logger().error(f'SocketCAN transport error: {error}')

    def _tick(self) -> None:
        now = time.monotonic()
        delta_s = now - self._last_tick_time
        self._last_tick_time = now
        for model in self._models:
            model.tick(delta_s)

    def _send_status(self, model=None) -> None:
        if model is None:
            for status_model in self._models:
                self._send_status(status_model)
            return

        frame = model.build_status_frame()
        self._transport.send_frame(frame)

    def _send_position_feedback(self) -> None:
        for model in self._models:
            max_frames = 2 if model.board_id == BOARD_ID_BOARD1 else None
            for feedback_frame in model.build_position_feedback_frames(
                max_frames=max_frames,
            ):
                self._transport.send_frame(feedback_frame)

    def destroy_node(self) -> bool:
        """Close the SocketCAN transport before destroying the node."""
        self._transport.close()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the Board1 simulator node."""
    rclpy.init(args=args)
    node = Board1SimulatorNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
