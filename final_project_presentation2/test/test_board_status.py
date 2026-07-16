"""Tests for the standalone arm-board status parser."""

from final_project_presentation2.board_status import parse_arm_board_status


def test_parses_each_board_and_controller_admission_state():
    message = (
        'Status snapshot: arm_v3_state=IDLE, active_goal=None || '
        'arm[board1: state=IDLE, error=ERR_NONE, ready_mask=0x0F, '
        'moving_mask=0x00, fault=0x00, enabled=True, stale=False, '
        'age_ms=12.5, position_valid=True; '
        'board2: state=IDLE, error=ERR_NONE, ready_mask=0x01, '
        'fault=0x00, enabled=True, stale=False, age_ms=13, '
        'position_valid=True; accept_traj=True] || '
        'gripper[board3: state=ESTOP, error=ERR_ESTOP, ready=0x00, '
        'fault=0x01, enabled=False, stale=False, age_ms=8, '
        'position_valid=False; accept_traj=False]'
    )

    parsed = parse_arm_board_status(message)

    assert parsed['arm_v3_state'] == 'IDLE'
    assert parsed['active_goal'] is None
    assert len(parsed['boards']) == 3
    assert parsed['controllers'][0]['accept_traj'] is True
    assert parsed['controllers'][1]['accept_traj'] is False
    assert parsed['boards'][0]['fields']['ready_mask'] == 0x0F
    assert parsed['boards'][0]['fields']['enabled'] is True
    assert parsed['boards'][2]['fields']['state'] == 'ESTOP'


def test_no_status_board_is_preserved_as_a_note():
    parsed = parse_arm_board_status(
        'Status snapshot: arm_v3_state=IDLE, active_goal=None || '
        'arm[board1=no status, stale=True; accept_traj=False]'
    )

    assert parsed['boards'][0]['board_id'] == 1
    assert parsed['boards'][0]['notes'] == ['no status']
    assert parsed['boards'][0]['fields']['stale'] is True
