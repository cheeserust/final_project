from dataclasses import dataclass
from enum import Enum


class MissionRuntimeState(str, Enum):
    """전체 미션의 실행 상태."""

    IDLE = 'IDLE'
    RUNNING = 'RUNNING'
    DONE = 'DONE'
    FAILED = 'FAILED'
    CANCELED = 'CANCELED'


@dataclass(frozen=True)
class MissionContext:
    """
    사용자가 /mission/execute Goal로 전달한 미션 정보.

    mission_flow.yaml 안의 $pickup_location, $target_floor 같은
    변수들을 실제 값으로 바꿀 때 사용한다.
    """

    mission_id: str
    pickup_location: str
    delivery_location: str
    target_floor: int
    object_label: str


@dataclass(frozen=True)
class MissionStep:
    """
    YAML 설정 세 개를 합쳐 만든 실행 가능한 작업 한 단계.

    예:
        state='DOCK_TO_OUTER_PANEL'
        task_id='dock_to_marker'
        server='/dock/align'
        target_name='outer_panel'
        target_floor=1
        marker_id=11
        timeout_sec=20.0
        retry=2
    """

    state: str
    task_id: str
    server: str
    target_name: str
    target_floor: int
    marker_id: int
    timeout_sec: float
    retry: int
    extra_json: str = '{}'


@dataclass(frozen=True)
class TaskExecutionResult:
    """하위 Action 한 번을 실행한 결과."""

    success: bool
    message: str
    canceled: bool = False
    timed_out: bool = False
