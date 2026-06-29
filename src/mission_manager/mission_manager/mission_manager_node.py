import os

import rclpy
from rclpy.action import (
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from ament_index_python.packages import (
    get_package_share_directory,
)

from vicpinky_interfaces.action import ExecuteMission
from vicpinky_interfaces.msg import MissionStatus

from .mission_flow_loader import MissionFlowLoader
from .mission_state import (
    MissionContext,
    MissionRuntimeState,
    TaskExecutionResult,
)
from .task_executor import TaskExecutor


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.callback_group = ReentrantCallbackGroup()

        package_share = get_package_share_directory(
            'mission_manager'
        )

        self.declare_parameter(
            'mission_flow_file',
            os.path.join(
                package_share,
                'config',
                'mission_flow.yaml',
            ),
        )

        self.declare_parameter(
            'locations_file',
            os.path.join(
                package_share,
                'config',
                'locations.yaml',
            ),
        )

        self.declare_parameter(
            'action_servers_file',
            os.path.join(
                package_share,
                'config',
                'action_servers.yaml',
            ),
        )

        mission_flow_file = (
            self.get_parameter('mission_flow_file')
            .get_parameter_value()
            .string_value
        )

        locations_file = (
            self.get_parameter('locations_file')
            .get_parameter_value()
            .string_value
        )

        action_servers_file = (
            self.get_parameter('action_servers_file')
            .get_parameter_value()
            .string_value
        )

        self.flow_loader = MissionFlowLoader(
            mission_flow_file=mission_flow_file,
            locations_file=locations_file,
            action_servers_file=action_servers_file,
        )

        self.task_executor = TaskExecutor(
            node=self,
            callback_group=self.callback_group,
        )

        self.status_publisher = self.create_publisher(
            MissionStatus,
            '/mission/status',
            10,
        )

        self.runtime_state = MissionRuntimeState.IDLE
        self.mission_active = False

        self.execute_action_server = ActionServer(
            self,
            ExecuteMission,
            '/mission/execute',
            execute_callback=self.execute_mission_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info('Mission Manager started')
        self.get_logger().info(
            f'Mission flow: {mission_flow_file}'
        )
        self.get_logger().info(
            f'Locations: {locations_file}'
        )
        self.get_logger().info(
            f'Action servers: {action_servers_file}'
        )

    def goal_callback(self, goal_request):
        """
        동시에 두 미션이 실행되지 않도록 새 Goal 수락 여부를 판단한다.
        """
        if self.mission_active:
            self.get_logger().warning(
                f'Rejecting mission "{goal_request.mission_id}": '
                'another mission is already active'
            )
            return GoalResponse.REJECT

        self.mission_active = True

        self.get_logger().info(
            f'Accept mission: '
            f'id={goal_request.mission_id}, '
            f'pickup={goal_request.pickup_location}, '
            f'delivery={goal_request.delivery_location}, '
            f'floor={goal_request.target_floor}, '
            f'object={goal_request.object_label}'
        )

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        전체 미션 취소 요청을 허용한다.

        실제 하위 Action의 취소는 TaskExecutor가 전달한다.
        """
        self.get_logger().warning('Mission cancel requested')
        return CancelResponse.ACCEPT

    def publish_status(
        self,
        mission_id: str,
        state: str,
        active_task: str,
        progress: float,
        error: bool,
        message: str,
    ) -> None:
        status = MissionStatus()

        status.mission_id = mission_id
        status.state = state
        status.active_task = active_task
        status.progress = float(progress)
        status.error = bool(error)
        status.message = message
        status.stamp = self.get_clock().now().to_msg()

        self.status_publisher.publish(status)

    @staticmethod
    def publish_action_feedback(
        goal_handle,
        state: str,
        active_task: str,
        progress: float,
        detail: str,
    ) -> None:
        feedback = ExecuteMission.Feedback()

        feedback.current_state = state
        feedback.current_task = active_task
        feedback.progress = float(progress)
        feedback.detail = detail

        goal_handle.publish_feedback(feedback)

    def finish_canceled(
        self,
        goal_handle,
        context: MissionContext,
        progress: float,
        message: str,
    ):
        self.runtime_state = MissionRuntimeState.CANCELED
        goal_handle.canceled()

        result = ExecuteMission.Result()
        result.success = False
        result.final_state = MissionRuntimeState.CANCELED.value
        result.message = message

        self.publish_status(
            mission_id=context.mission_id,
            state=MissionRuntimeState.CANCELED.value,
            active_task='',
            progress=progress,
            error=True,
            message=message,
        )

        return result

    def execute_mission_callback(self, goal_handle):
        request = goal_handle.request

        context = MissionContext(
            mission_id=request.mission_id,
            pickup_location=request.pickup_location,
            delivery_location=request.delivery_location,
            target_floor=request.target_floor,
            object_label=request.object_label,
        )

        result = ExecuteMission.Result()
        self.runtime_state = MissionRuntimeState.RUNNING

        try:
            try:
                plan = self.flow_loader.build_plan(context)
            except Exception as exc:
                self.runtime_state = MissionRuntimeState.FAILED
                goal_handle.abort()

                result.success = False
                result.final_state = 'CONFIG_ERROR'
                result.message = f'Failed to build mission plan: {exc}'

                self.publish_status(
                    mission_id=context.mission_id,
                    state='CONFIG_ERROR',
                    active_task='',
                    progress=0.0,
                    error=True,
                    message=result.message,
                )

                self.get_logger().error(result.message)
                return result

            total_steps = len(plan)

            self.publish_status(
                mission_id=context.mission_id,
                state=MissionRuntimeState.RUNNING.value,
                active_task='',
                progress=0.0,
                error=False,
                message='Mission started',
            )

            self.publish_action_feedback(
                goal_handle=goal_handle,
                state=MissionRuntimeState.RUNNING.value,
                active_task='',
                progress=0.0,
                detail='Mission started',
            )

            for step_index, step in enumerate(plan):
                if goal_handle.is_cancel_requested:
                    return self.finish_canceled(
                        goal_handle=goal_handle,
                        context=context,
                        progress=step_index / total_steps,
                        message='Mission canceled by user',
                    )

                max_attempts = step.retry + 1
                step_succeeded = False

                last_result = TaskExecutionResult(
                    success=False,
                    message='Task was not executed',
                )

                self.get_logger().info(
                    f'===== Step {step_index + 1}/{total_steps}: '
                    f'{step.state} ====='
                )

                for attempt in range(1, max_attempts + 1):
                    base_progress = step_index / total_steps

                    attempt_message = (
                        f'Attempt {attempt}/{max_attempts}'
                    )

                    self.publish_status(
                        mission_id=context.mission_id,
                        state=step.state,
                        active_task=step.server,
                        progress=base_progress,
                        error=False,
                        message=attempt_message,
                    )

                    self.publish_action_feedback(
                        goal_handle=goal_handle,
                        state=step.state,
                        active_task=step.server,
                        progress=base_progress,
                        detail=attempt_message,
                    )

                    def child_feedback_callback(
                        child_step,
                        child_feedback,
                        current_index=step_index,
                    ):
                        child_progress = max(
                            0.0,
                            min(
                                1.0,
                                float(child_feedback.progress),
                            ),
                        )

                        overall_progress = (
                            current_index + child_progress
                        ) / total_steps

                        detail = (
                            f'phase={child_feedback.phase}, '
                            f'detail={child_feedback.detail}'
                        )

                        self.publish_status(
                            mission_id=context.mission_id,
                            state=child_step.state,
                            active_task=child_step.server,
                            progress=overall_progress,
                            error=False,
                            message=detail,
                        )

                        self.publish_action_feedback(
                            goal_handle=goal_handle,
                            state=child_step.state,
                            active_task=child_step.server,
                            progress=overall_progress,
                            detail=detail,
                        )

                    last_result = self.task_executor.execute(
                        step=step,
                        mission_goal_handle=goal_handle,
                        feedback_callback=child_feedback_callback,
                    )

                    if last_result.canceled:
                        return self.finish_canceled(
                            goal_handle=goal_handle,
                            context=context,
                            progress=step_index / total_steps,
                            message=last_result.message,
                        )

                    if last_result.success:
                        step_succeeded = True

                        self.get_logger().info(
                            f'Step succeeded: {step.state}, '
                            f'message="{last_result.message}"'
                        )
                        break

                    self.get_logger().warning(
                        f'Step failed: {step.state}, '
                        f'attempt={attempt}/{max_attempts}, '
                        f'message="{last_result.message}"'
                    )

                if not step_succeeded:
                    self.runtime_state = MissionRuntimeState.FAILED
                    goal_handle.abort()

                    result.success = False
                    result.final_state = step.state
                    result.message = (
                        f'Mission failed at {step.state}: '
                        f'{last_result.message}'
                    )

                    self.publish_status(
                        mission_id=context.mission_id,
                        state=step.state,
                        active_task=step.server,
                        progress=step_index / total_steps,
                        error=True,
                        message=result.message,
                    )

                    return result

            self.runtime_state = MissionRuntimeState.DONE
            goal_handle.succeed()

            result.success = True
            result.final_state = MissionRuntimeState.DONE.value
            result.message = 'Mission completed successfully'

            self.publish_status(
                mission_id=context.mission_id,
                state=MissionRuntimeState.DONE.value,
                active_task='',
                progress=1.0,
                error=False,
                message=result.message,
            )

            self.publish_action_feedback(
                goal_handle=goal_handle,
                state=MissionRuntimeState.DONE.value,
                active_task='',
                progress=1.0,
                detail=result.message,
            )

            return result

        except Exception as exc:
            self.runtime_state = MissionRuntimeState.FAILED

            self.get_logger().exception(
                f'Unexpected mission error: {exc}'
            )

            goal_handle.abort()

            result.success = False
            result.final_state = MissionRuntimeState.FAILED.value
            result.message = f'Unexpected mission error: {exc}'

            self.publish_status(
                mission_id=context.mission_id,
                state=MissionRuntimeState.FAILED.value,
                active_task='',
                progress=0.0,
                error=True,
                message=result.message,
            )

            return result

        finally:
            self.mission_active = False


def main(args=None):
    rclpy.init(args=args)

    node = MissionManager()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()