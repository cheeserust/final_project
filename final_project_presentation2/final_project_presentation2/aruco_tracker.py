"""Independent OpenCV ArUco detection for the front and rear cameras."""

from dataclasses import dataclass
import math
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import cv2

import numpy as np


def _finite(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be finite') from exc
    if not math.isfinite(result):
        raise ValueError(f'{name} must be finite')
    return result


def _positive(value: Any, name: str) -> float:
    result = _finite(value, name)
    if result <= 0.0:
        raise ValueError(f'{name} must be greater than zero')
    return result


def _boolean(value: Any, name: str) -> bool:
    if isinstance(value, bool):
        return value
    raise ValueError(f'{name} must be a boolean')


def _matrix3x3(value: Any, name: str) -> np.ndarray:
    try:
        matrix = np.asarray(value, dtype=np.float64).reshape(3, 3)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must contain a 3x3 matrix') from exc
    if not np.all(np.isfinite(matrix)):
        raise ValueError(f'{name} must contain only finite values')
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
        raise ValueError(f'{name} focal lengths must be positive')
    return matrix.copy()


def _distortion(value: Any, name: str) -> np.ndarray:
    try:
        coefficients = np.asarray(value, dtype=np.float64).reshape(-1, 1)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be a numeric sequence') from exc
    if coefficients.size < 4:
        raise ValueError(f'{name} must contain at least four coefficients')
    if not np.all(np.isfinite(coefficients)):
        raise ValueError(f'{name} must contain only finite values')
    return coefficients.copy()


@dataclass(frozen=True)
class CameraCalibration:
    """Intrinsic calibration and image orientation for one camera."""

    camera_matrix: Any
    distortion_coefficients: Any
    flip_horizontal: bool = False
    flip_vertical: bool = False

    def __post_init__(self) -> None:
        """Validate and copy intrinsic arrays and flip flags."""
        object.__setattr__(
            self,
            'camera_matrix',
            _matrix3x3(self.camera_matrix, 'camera_matrix'),
        )
        object.__setattr__(
            self,
            'distortion_coefficients',
            _distortion(
                self.distortion_coefficients, 'distortion_coefficients'
            ),
        )
        object.__setattr__(
            self,
            'flip_horizontal',
            _boolean(self.flip_horizontal, 'flip_horizontal'),
        )
        object.__setattr__(
            self,
            'flip_vertical',
            _boolean(self.flip_vertical, 'flip_vertical'),
        )

    @classmethod
    def from_mapping(cls, config: Mapping[str, Any]) -> 'CameraCalibration':
        """Build a calibration from the unified JSON camera mapping."""
        if not isinstance(config, Mapping):
            raise ValueError('camera configuration must be a mapping')
        matrix = config.get('camera_matrix')
        distortion = config.get(
            'distortion_coefficients', config.get('distortion')
        )
        return cls(
            camera_matrix=matrix,
            distortion_coefficients=distortion,
            flip_horizontal=config.get('flip_horizontal', False),
            flip_vertical=config.get('flip_vertical', False),
        )


@dataclass(frozen=True)
class MarkerObservation:
    """Pixel and metric pose data for one detected marker."""

    marker_id: int
    camera_name: str
    timestamp_sec: float
    corners_px: Tuple[Tuple[float, float], ...]
    center_x_px: float
    center_y_px: float
    pixel_width: float
    pose_valid: bool
    distance_m: Optional[float] = None
    lateral_m: Optional[float] = None
    vertical_m: Optional[float] = None
    yaw_rad: Optional[float] = None
    translation_m: Optional[Tuple[float, float, float]] = None
    rotation_vector: Optional[Tuple[float, float, float]] = None

    def __post_init__(self) -> None:
        """Validate pixel geometry and optional metric pose."""
        if self.marker_id < 0:
            raise ValueError('marker_id must be nonnegative')
        if not self.camera_name:
            raise ValueError('camera_name must not be empty')
        _finite(self.timestamp_sec, 'timestamp_sec')
        _finite(self.center_x_px, 'center_x_px')
        _finite(self.center_y_px, 'center_y_px')
        _positive(self.pixel_width, 'pixel_width')
        if len(self.corners_px) != 4:
            raise ValueError('corners_px must contain four points')
        for point in self.corners_px:
            if len(point) != 2:
                raise ValueError('each corner must contain x and y')
            _finite(point[0], 'corner_x')
            _finite(point[1], 'corner_y')
        if self.pose_valid:
            for name in ('distance_m', 'lateral_m', 'vertical_m', 'yaw_rad'):
                if getattr(self, name) is None:
                    raise ValueError(f'{name} is required for a valid pose')
                _finite(getattr(self, name), name)
            if self.distance_m <= 0.0:
                raise ValueError('distance_m must be greater than zero')


class ArucoTracker:
    """
    Detect configured ArUco markers with camera-specific calibration.

    The implementation supports both the newer ``ArucoDetector`` object API
    and the older module-level ``detectMarkers`` API shipped by ROS distros.
    """

    def __init__(
        self,
        dictionary_name: str = 'DICT_4X4_50',
        marker_size_m: float = 0.10,
        cameras: Optional[Mapping[str, Any]] = None,
        marker_sizes_m: Optional[Mapping[int, float]] = None,
    ) -> None:
        """Create a detector and validate dictionary, sizes, and cameras."""
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('OpenCV was built without the aruco module')
        self.dictionary_name = str(dictionary_name).strip()
        if not self.dictionary_name:
            raise ValueError('dictionary_name must not be empty')
        dictionary_id = getattr(cv2.aruco, self.dictionary_name, None)
        if dictionary_id is None:
            raise ValueError(
                f'unknown ArUco dictionary: {self.dictionary_name}'
            )
        self._dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.marker_size_m = _positive(marker_size_m, 'marker_size_m')
        self._marker_sizes: Dict[int, float] = {}
        if marker_sizes_m is not None:
            if not isinstance(marker_sizes_m, Mapping):
                raise ValueError('marker_sizes_m must be a mapping')
            for raw_id, raw_size in marker_sizes_m.items():
                if isinstance(raw_id, bool):
                    raise ValueError('marker size keys must be integer IDs')
                marker_id = int(raw_id)
                if marker_id < 0 or str(marker_id) != str(raw_id).strip():
                    raise ValueError('marker size keys must be integer IDs')
                self._marker_sizes[marker_id] = _positive(
                    raw_size, f'marker_sizes_m[{marker_id}]'
                )

        self._cameras: Dict[str, CameraCalibration] = {}
        if cameras is not None:
            if not isinstance(cameras, Mapping):
                raise ValueError('cameras must be a mapping')
            for name, calibration in cameras.items():
                self.set_camera_calibration(name, calibration)
        self._parameters = self._make_detector_parameters()
        self._detector = self._make_detector()

    @classmethod
    def from_config(cls, config: Mapping[str, Any]) -> 'ArucoTracker':
        """Construct a tracker directly from the unified config mapping."""
        if not isinstance(config, Mapping):
            raise ValueError('config must be a mapping')
        aruco_config = config.get('aruco', {})
        if not isinstance(aruco_config, Mapping):
            raise ValueError('aruco must be a mapping')
        cameras_config = config.get('cameras', {})
        if not isinstance(cameras_config, Mapping):
            raise ValueError('cameras must be a mapping')

        calibrations: Dict[str, CameraCalibration] = {}
        for camera_name in ('front', 'rear'):
            camera = cameras_config.get(camera_name)
            if not isinstance(camera, Mapping):
                continue
            matrix = camera.get('camera_matrix')
            distortion = camera.get(
                'distortion_coefficients', camera.get('distortion')
            )
            if matrix is None or distortion is None:
                continue
            calibrations[camera_name] = CameraCalibration.from_mapping(camera)
        return cls(
            dictionary_name=aruco_config.get(
                'dictionary', aruco_config.get(
                    'dictionary_name', 'DICT_4X4_50'
                )
            ),
            marker_size_m=aruco_config.get('marker_size_m', 0.10),
            cameras=calibrations,
            marker_sizes_m=aruco_config.get('marker_sizes_m'),
        )

    @staticmethod
    def _make_detector_parameters() -> Any:
        # OpenCV 4.6 exposes ``DetectorParameters`` but its legacy module-level
        # detector expects the object returned by ``_create``.  Prefer the API
        # generation that matches the available detector entry point.
        constructor = getattr(cv2.aruco, 'DetectorParameters', None)
        object_detector = getattr(cv2.aruco, 'ArucoDetector', None)
        if object_detector is not None and constructor is not None:
            try:
                return constructor()
            except TypeError:
                pass
        factory = getattr(cv2.aruco, 'DetectorParameters_create', None)
        if factory is not None:
            return factory()
        if constructor is not None:
            return constructor()
        raise RuntimeError('OpenCV ArUco detector parameters unavailable')

    def _make_detector(self) -> Any:
        constructor = getattr(cv2.aruco, 'ArucoDetector', None)
        if constructor is None:
            return None
        try:
            return constructor(self._dictionary, self._parameters)
        except (AttributeError, TypeError):
            return None

    @property
    def camera_names(self) -> Tuple[str, ...]:
        """Return camera names that currently have metric calibration."""
        return tuple(sorted(self._cameras))

    def set_camera_calibration(
        self, camera_name: str, calibration: Any
    ) -> None:
        """
        Add or replace one camera's calibration.

        This supports CameraInfo arriving after the tracker has started.
        """
        name = str(camera_name).strip()
        if not name:
            raise ValueError('camera_name must not be empty')
        if isinstance(calibration, CameraCalibration):
            checked = calibration
        elif isinstance(calibration, Mapping):
            checked = CameraCalibration.from_mapping(calibration)
        else:
            raise TypeError(
                'calibration must be CameraCalibration or a mapping'
            )
        self._cameras[name] = checked

    def remove_camera_calibration(self, camera_name: str) -> None:
        """Remove a camera calibration without affecting pixel detection."""
        self._cameras.pop(str(camera_name), None)

    def _detect_markers(
        self, gray: np.ndarray
    ) -> Tuple[Sequence[Any], Any, Sequence[Any]]:
        if self._detector is not None:
            return self._detector.detectMarkers(gray)
        return cv2.aruco.detectMarkers(
            gray, self._dictionary, parameters=self._parameters
        )

    @staticmethod
    def _flip(frame: np.ndarray, calibration: Optional[CameraCalibration]):
        if calibration is None:
            return frame
        if calibration.flip_horizontal and calibration.flip_vertical:
            return cv2.flip(frame, -1)
        if calibration.flip_horizontal:
            return cv2.flip(frame, 1)
        if calibration.flip_vertical:
            return cv2.flip(frame, 0)
        return frame

    @staticmethod
    def _gray(frame: np.ndarray) -> np.ndarray:
        if frame.ndim == 2:
            return frame
        if frame.ndim != 3:
            raise ValueError('frame must be a grayscale, BGR, or BGRA image')
        if frame.shape[2] == 3:
            return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if frame.shape[2] == 4:
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)
        raise ValueError('frame must be a grayscale, BGR, or BGRA image')

    def _marker_size(self, marker_id: int) -> float:
        return self._marker_sizes.get(marker_id, self.marker_size_m)

    @staticmethod
    def _object_points(size_m: float) -> np.ndarray:
        half = size_m / 2.0
        return np.asarray(
            [
                (-half, half, 0.0),
                (half, half, 0.0),
                (half, -half, 0.0),
                (-half, -half, 0.0),
            ],
            dtype=np.float32,
        )

    def _solve_pose(
        self,
        corners: np.ndarray,
        marker_size_m: float,
        calibration: CameraCalibration,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        estimate = getattr(cv2.aruco, 'estimatePoseSingleMarkers', None)
        if estimate is not None:
            try:
                rvecs, tvecs, _ = estimate(
                    corners.reshape(1, 4, 2),
                    marker_size_m,
                    calibration.camera_matrix,
                    calibration.distortion_coefficients,
                )
                return (
                    np.asarray(rvecs, dtype=np.float64).reshape(-1, 3)[0],
                    np.asarray(tvecs, dtype=np.float64).reshape(-1, 3)[0],
                )
            except (cv2.error, TypeError, ValueError):
                pass

        flags = getattr(cv2, 'SOLVEPNP_IPPE_SQUARE', cv2.SOLVEPNP_ITERATIVE)
        try:
            success, rvec, tvec = cv2.solvePnP(
                self._object_points(marker_size_m),
                corners.reshape(4, 2).astype(np.float32),
                calibration.camera_matrix,
                calibration.distortion_coefficients,
                flags=flags,
            )
        except (cv2.error, TypeError, ValueError):
            return None, None
        if not success:
            return None, None
        return rvec.reshape(3), tvec.reshape(3)

    @staticmethod
    def _marker_yaw(rvec: np.ndarray) -> float:
        rotation, _ = cv2.Rodrigues(rvec.reshape(3, 1))
        normal = rotation[:, 2].astype(np.float64)
        # OpenCV's marker convention can point the plane normal either toward
        # or away from the camera depending on pose API/version.  Normalize it
        # toward the camera so a front-facing marker has yaw zero.
        if normal[2] > 0.0:
            normal *= -1.0
        return math.atan2(float(normal[0]), float(-normal[2]))

    @staticmethod
    def _pixel_width(corners: np.ndarray) -> float:
        lengths = []
        for index in range(4):
            delta = corners[(index + 1) % 4] - corners[index]
            lengths.append(float(np.linalg.norm(delta)))
        return sum(lengths) / len(lengths)

    def detect(
        self,
        frame: np.ndarray,
        camera_name: str,
        timestamp_sec: float,
    ) -> Tuple[MarkerObservation, ...]:
        """
        Detect all markers in one frame, sorted by marker ID.

        Pixel-only observations are still returned when no calibration exists;
        their ``pose_valid`` field is false so motion control cannot use them.
        """
        name = str(camera_name).strip()
        if not name:
            raise ValueError('camera_name must not be empty')
        timestamp = _finite(timestamp_sec, 'timestamp_sec')
        image = np.asarray(frame)
        if image.size == 0:
            raise ValueError('frame must not be empty')
        if image.dtype != np.uint8:
            raise ValueError('frame must use uint8 pixels')
        calibration = self._cameras.get(name)
        processed = self._flip(image, calibration)
        gray = np.ascontiguousarray(self._gray(processed))
        corners, ids, _ = self._detect_markers(gray)
        if ids is None or len(ids) == 0:
            return ()

        observations = []
        flat_ids = np.asarray(ids).reshape(-1)
        for raw_id, raw_corners in zip(flat_ids, corners):
            # A partially corrupted image can yield one malformed candidate.
            # Skip only that candidate instead of losing valid markers from
            # the same frame or poisoning following frames.
            try:
                marker_id = int(raw_id)
                if marker_id < 0:
                    continue
                points = np.asarray(
                    raw_corners, dtype=np.float64
                ).reshape(4, 2)
                if not np.all(np.isfinite(points)):
                    continue
                pixel_width = self._pixel_width(points)
                if not math.isfinite(pixel_width) or pixel_width <= 0.0:
                    continue
                immutable_corners = tuple(
                    (float(point[0]), float(point[1])) for point in points
                )
                center = np.mean(points, axis=0)
                pose = None
                if calibration is not None:
                    rvec, tvec = self._solve_pose(
                        points,
                        self._marker_size(marker_id),
                        calibration,
                    )
                    if rvec is not None and tvec is not None:
                        values = np.concatenate((rvec, tvec))
                        if np.all(np.isfinite(values)) and tvec[2] > 0.0:
                            pose = (rvec, tvec)

                if pose is None:
                    observation = MarkerObservation(
                        marker_id=marker_id,
                        camera_name=name,
                        timestamp_sec=timestamp,
                        corners_px=immutable_corners,
                        center_x_px=float(center[0]),
                        center_y_px=float(center[1]),
                        pixel_width=pixel_width,
                        pose_valid=False,
                    )
                else:
                    rvec, tvec = pose
                    observation = MarkerObservation(
                        marker_id=marker_id,
                        camera_name=name,
                        timestamp_sec=timestamp,
                        corners_px=immutable_corners,
                        center_x_px=float(center[0]),
                        center_y_px=float(center[1]),
                        pixel_width=pixel_width,
                        pose_valid=True,
                        distance_m=float(tvec[2]),
                        lateral_m=float(tvec[0]),
                        vertical_m=float(tvec[1]),
                        yaw_rad=self._marker_yaw(rvec),
                        translation_m=tuple(float(item) for item in tvec),
                        rotation_vector=tuple(float(item) for item in rvec),
                    )
                observations.append(observation)
            except (cv2.error, TypeError, ValueError, OverflowError):
                continue
        observations.sort(key=lambda item: item.marker_id)
        return tuple(observations)

    def detect_target(
        self,
        frame: np.ndarray,
        camera_name: str,
        timestamp_sec: float,
        marker_id: int,
    ) -> Optional[MarkerObservation]:
        """Return one requested marker observation, if present."""
        requested = int(marker_id)
        for observation in self.detect(frame, camera_name, timestamp_sec):
            if observation.marker_id == requested:
                return observation
        return None
