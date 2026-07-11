"""Tests for mission CLI arm task selection."""

from mission_manager.send_mission import main
import pytest


def test_list_locations_does_not_require_arm_task(capsys):
    assert main(['--list-locations']) == 0
    assert capsys.readouterr().out


def test_sending_mission_requires_concrete_arm_task():
    with pytest.raises(SystemExit) as exc_info:
        main([])

    assert exc_info.value.code == 2
