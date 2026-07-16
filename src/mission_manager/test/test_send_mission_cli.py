"""Tests for mission CLI arm task selection."""

from mission_manager.send_mission import build_argument_parser, main


def test_list_locations_does_not_require_arm_task(capsys):
    assert main(['--list-locations']) == 0
    assert capsys.readouterr().out


def test_default_goal_matches_calibrated_final_scenario():
    args = build_argument_parser().parse_args([])

    assert args.pickup_location == '402'
    assert args.delivery_location == 'object_place'
    assert args.object_label == 'object_1'
    assert args.arm_task_name == 'deliver_object_1_from_tray'
