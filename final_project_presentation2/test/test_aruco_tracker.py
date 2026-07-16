"""Tests for the standalone front/rear ArUco tracker."""

import cv2

from final_project_presentation2.aruco_tracker import ArucoTracker
from final_project_presentation2.aruco_tracker import CameraCalibration

import numpy as np

import pytest


def marker_frame(marker_id=7):
    """Render one legacy/new-API compatible marker on a white canvas."""
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    marker = np.zeros((200, 200), dtype=np.uint8)
    if hasattr(aruco, 'generateImageMarker'):
        marker = aruco.generateImageMarker(dictionary, marker_id, 200)
    else:
        aruco.drawMarker(dictionary, marker_id, 200, marker, 1)
    frame = np.full((400, 400), 255, dtype=np.uint8)
    frame[100:300, 100:300] = marker
    return frame


def calibration(focal_length):
    """Create a simple zero-distortion calibration."""
    return CameraCalibration(
        camera_matrix=[
            [focal_length, 0.0, 200.0],
            [0.0, focal_length, 200.0],
            [0.0, 0.0, 1.0],
        ],
        distortion_coefficients=[0.0, 0.0, 0.0, 0.0, 0.0],
    )


def test_detect_returns_pixel_observation_without_calibration():
    """Allow UI detection but explicitly mark metric pose unavailable."""
    tracker = ArucoTracker()
    observations = tracker.detect(marker_frame(), 'front', 1.0)

    assert len(observations) == 1
    result = observations[0]
    assert result.marker_id == 7
    assert result.camera_name == 'front'
    assert result.center_x_px == pytest.approx(199.5)
    assert not result.pose_valid
    assert result.distance_m is None


def test_front_and_rear_keep_independent_metric_calibrations():
    """Use each camera's own intrinsics when estimating the same marker."""
    tracker = ArucoTracker(
        cameras={'front': calibration(500.0), 'rear': calibration(800.0)}
    )
    front = tracker.detect_target(marker_frame(), 'front', 2.0, 7)
    rear = tracker.detect_target(marker_frame(), 'rear', 2.0, 7)

    assert front is not None and front.pose_valid
    assert rear is not None and rear.pose_valid
    assert front.distance_m > 0.0
    assert rear.distance_m > front.distance_m
    assert abs(front.yaw_rad) < 0.05
    assert abs(rear.yaw_rad) < 0.05


def test_from_unified_config_and_runtime_camera_info_update():
    """Load configured cameras and support a later CameraInfo replacement."""
    config = {
        'aruco': {'dictionary': 'DICT_4X4_50', 'marker_size_m': 0.08},
        'cameras': {
            'front': {
                'camera_matrix': calibration(400.0).camera_matrix.tolist(),
                'distortion_coefficients': [0.0] * 5,
                'flip_horizontal': False,
                'flip_vertical': False,
            },
            'rear': {
                'camera_matrix': None,
                'distortion_coefficients': None,
            },
        },
    }
    tracker = ArucoTracker.from_config(config)
    assert tracker.camera_names == ('front',)

    tracker.set_camera_calibration('rear', calibration(600.0))
    assert tracker.camera_names == ('front', 'rear')
    assert tracker.detect_target(marker_frame(), 'rear', 3.0, 7).pose_valid


def test_configurable_dictionary_and_input_validation():
    """Reject misspelled dictionaries and corrupt image/calibration inputs."""
    with pytest.raises(ValueError, match='unknown ArUco dictionary'):
        ArucoTracker(dictionary_name='DICT_DOES_NOT_EXIST')
    invalid_matrix = np.eye(3)
    invalid_matrix[0, 0] = 0.0
    with pytest.raises(ValueError, match='focal'):
        CameraCalibration(invalid_matrix, [0.0] * 5)
    tracker = ArucoTracker()
    with pytest.raises(ValueError, match='uint8'):
        tracker.detect(marker_frame().astype(np.float32), 'front', 1.0)


def test_malformed_candidate_does_not_discard_valid_marker(monkeypatch):
    """Keep usable detections when one candidate in the frame is corrupt."""
    tracker = ArucoTracker()
    valid = np.asarray(
        [[[100.0, 100.0], [299.0, 100.0], [299.0, 299.0], [100.0, 299.0]]],
        dtype=np.float32,
    )
    corrupt = np.full((1, 4, 2), np.nan, dtype=np.float32)

    monkeypatch.setattr(
        tracker,
        '_detect_markers',
        lambda gray: ([corrupt, valid], np.asarray([[8], [7]]), []),
    )

    observations = tracker.detect(marker_frame(), 'front', 1.0)

    assert [item.marker_id for item in observations] == [7]
