#!/usr/bin/env python3
# 엘리베이터 승/하차: ArUco solvePnP 기반 visual servoing
# RunTask Action Server 2개 제공:
#   /elevator/board : 문열림 대기(BOARD 마커) → 전진 탑승 → BOARDING_STOP_CM 정지
#   /elevator/exit  : target_floor 랜딩 마커 확인 → 후진 하차(EXIT_STOP_CM) → 좌 90도 회전
# 층 인식: 탑승 완료 후 후방 카메라가 랜딩 마커(4/5)를 안정 검출하면 /tag/floor_id 발행
#          → floor_check_server(/floor/check, mock_mode:=false)가 이를 소비

import math
import time
import json
import cv2
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from vicpinky_interfaces.action import RunTask

# ── 카메라 캘리브레이션 ─────────────────────────────────────
# 로지텍 (전방)
FRONT_CAMERA_MATRIX = np.array([
    [708.85065781,   0.0,        308.289349  ],
    [  0.0,        707.92630029, 244.0512732 ],
    [  0.0,          0.0,          1.0       ]
], dtype=np.float64)
FRONT_DIST_COEFFS = np.array(
    [0.02414867, 0.89713946, 0.00248749, -0.01085416, -2.32266594]
)

# Pleomax (후방)
REAR_CAMERA_MATRIX = np.array([
    [859.90806761,   0.0,        252.83359168],
    [  0.0,        858.64794316, 248.03325075],
    [  0.0,          0.0,          1.0       ]
], dtype=np.float64)
REAR_DIST_COEFFS = np.array(
    [0.13899504, -0.77334006, 0.00611556, -0.01769451, 2.75664301]
)

# ── 설정값 (현장에 맞게 조정) ───────────────────────────────
BOARD_MARKER_ID   = 10           # 탑승용: 캐빈 안쪽 마커
FRONT_IMAGE_TOPIC = "/front_camera/image_raw"
REAR_IMAGE_TOPIC  = "/rear_camera/image_raw"
FLOOR_IDS         = [4, 5]       # 랜딩 마커 ID = 층 번호

MARKER_LENGTH     = 0.1          # 마커 한 변 길이(m)
BOARDING_STOP_CM  = 50.0         # 승차시 BOARD 마커까지 목표 거리 (cm)
EXIT_STOP_CM      = 60.0        # 하차시 목표 거리 (cm)
STOP_TOLERANCE_CM = 1.5          # 허용 오차

ROTATE_ANGULAR_SPEED = 0.4
ROTATE_ANGLE_DEG     = 90.0
ROTATE_DURATION_SEC  = math.radians(ROTATE_ANGLE_DEG) / ROTATE_ANGULAR_SPEED

MAX_LINEAR        = 0.2          # m/s
MIN_LINEAR        = 0.1          # m/s
KP_LINEAR         = 0.01
KP_ANGULAR        = 0.5

DEBOUNCE_FRAMES   = 15           # N프레임 연속 보여야 "진짜 보임"
LOST_FRAMES       = 15           # N프레임 연속 안 보여야 "사라짐"
CONTROL_HZ        = 10.0
# ────────────────────────────────────────────────────────────

half = MARKER_LENGTH / 2.0
OBJ_POINTS = np.array([
    [-half,  half, 0],
    [ half,  half, 0],
    [ half, -half, 0],
    [-half, -half, 0]
], dtype=np.float64)


class ElevatorServers(Node):
    def __init__(self):
        super().__init__("elevator_servers")

        self.declare_parameter('board_server_name', '/elevator/board')
        self.declare_parameter('exit_server_name', '/elevator/exit')
        self.declare_parameter('board_wait_timeout_sec', 180.0)
        self.declare_parameter('exit_wait_timeout_sec', 180.0)
        self.declare_parameter('servo_timeout_sec', 90.0)
        self.declare_parameter(
            'boarding_target_distance_cm', float(BOARDING_STOP_CM))
        self.declare_parameter('camera_stale_timeout_sec', 0.75)

        self.board_server_name = self.get_parameter('board_server_name').value
        self.exit_server_name = self.get_parameter('exit_server_name').value
        self.board_wait_timeout_sec = float(self.get_parameter('board_wait_timeout_sec').value)
        self.exit_wait_timeout_sec = float(self.get_parameter('exit_wait_timeout_sec').value)
        self.servo_timeout_sec = float(self.get_parameter('servo_timeout_sec').value)
        self.boarding_target_distance_cm = float(
            self.get_parameter('boarding_target_distance_cm').value)
        self.camera_stale_timeout_sec = float(
            self.get_parameter('camera_stale_timeout_sec').value)

        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(
                self.aruco_dict, cv2.aruco.DetectorParameters())
            self.use_new_aruco_api = True
        else:
            # Ubuntu 22.04/OpenCV 4.6 does not provide ArucoDetector.
            self.detector_parameters = cv2.aruco.DetectorParameters_create()
            self.use_new_aruco_api = False

        self.cb_group = ReentrantCallbackGroup()

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.floor_pub = self.create_publisher(Int32, "/tag/floor_id", 10)

        # 디바운스 카운터 / 최신 pose 캐시 {id: (tz_cm, tx_m, monotonic_time)}
        self.front_seen = {}
        self.rear_seen = {}
        self.front_pose = {}
        self.rear_pose = {}
        self.frame_idx = {"front": 0, "rear": 0}

        # 카메라 게이팅: 탑승 완료~하차 완료 사이에만 후방 활성 (기존 FRONT/REAR_STATES와 동일 의미)
        self.rear_enabled = False
        # 동시 goal 방지
        self.busy = False

        self.create_subscription(
            Image, FRONT_IMAGE_TOPIC,
            lambda m: self.image_cb(m, "front", self.front_seen, self.front_pose,
                                    FRONT_CAMERA_MATRIX, FRONT_DIST_COEFFS),
            qos_profile_sensor_data, callback_group=self.cb_group)
        self.create_subscription(
            Image, REAR_IMAGE_TOPIC,
            lambda m: self.image_cb(m, "rear", self.rear_seen, self.rear_pose,
                                    REAR_CAMERA_MATRIX, REAR_DIST_COEFFS),
            qos_profile_sensor_data, callback_group=self.cb_group)

        self.board_server = ActionServer(
            self, RunTask, self.board_server_name,
            execute_callback=self.execute_board,
            goal_callback=self.board_goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group)
        self.exit_server = ActionServer(
            self, RunTask, self.exit_server_name,
            execute_callback=self.execute_exit,
            goal_callback=self.exit_goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group)

        self.get_logger().info(
            f'Elevator Servers Started: {self.board_server_name}, {self.exit_server_name}')

    # ── 이미지 콜백: 검출 + pose 계산 ─────────────────────────
    def image_cb(self, msg, cam, seen_dict, pose_dict, camera_matrix, dist_coeffs):
        if cam == "front" and self.rear_enabled:
            return
        if cam == "rear" and not self.rear_enabled:
            return
        self.frame_idx[cam] += 1
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(
                f'{cam} bridge fail: {e}', throttle_duration_sec=5.0)
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.use_new_aruco_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.detector_parameters)
        now_ids = {int(i) for i in ids.flatten()} if ids is not None else set()

        for mid in set(seen_dict) | now_ids:
            seen_dict[mid] = (min(seen_dict.get(mid, 0) + 1, DEBOUNCE_FRAMES)
                              if mid in now_ids else max(seen_dict.get(mid, 0) - 1, 0))

        # 후방: 랜딩 마커 안정 검출 시 층 발행 → floor_check_server 소비
        if cam == "rear":
            for floor in FLOOR_IDS:
                if self.stable_seen(seen_dict, floor):
                    self.floor_pub.publish(Int32(data=floor))
                    break

        if ids is None:
            return

        for i, marker_id in enumerate(ids.flatten()):
            img_points = corners[i][0].astype(np.float64)
            ok, rvec, tvec = cv2.solvePnP(
                OBJ_POINTS, img_points, camera_matrix, dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                tz_cm = float(tvec[2][0]) * 100.0
                tx_m = float(tvec[0][0])
                pose_dict[int(marker_id)] = (tz_cm, tx_m, time.monotonic())

    def stable_seen(self, seen_dict, marker_id) -> bool:
        return seen_dict.get(marker_id, 0) >= DEBOUNCE_FRAMES

    def get_fresh_pose(self, cam, pose_dict, marker_id):
        """카메라 프레임이 멈춰도 시간 기준으로 만료된 pose를 거부한다."""
        entry = pose_dict.get(marker_id)
        if entry is None:
            return None
        tz_cm, tx_m, seen_time = entry
        if time.monotonic() - seen_time > self.camera_stale_timeout_sec:
            return None
        return tz_cm, tx_m

    # ── 서보잉: 목표거리까지 P control ────────────────────────
    def servo_toward(self, tz_cm, tx_m, direction, target_cm=BOARDING_STOP_CM):
        """Drive forward (+1) or backward (-1); return True on arrival."""
        error_cm = tz_cm - target_cm
        if abs(error_cm) <= STOP_TOLERANCE_CM:
            self.stop()
            return True
        linear = KP_LINEAR * abs(error_cm)
        linear = max(MIN_LINEAR, min(MAX_LINEAR, linear))
        angular = -KP_ANGULAR * tx_m
        error_direction = 1.0 if error_cm > 0.0 else -1.0
        self.drive(linear=direction * error_direction * linear, angular=angular)
        return False

    def board_target_cm(self, goal):
        """Read per-goal distance while accepting both cm and legacy metre keys."""
        target_cm = self.boarding_target_distance_cm
        if goal.extra_json:
            extra = json.loads(goal.extra_json)
            if 'target_distance_cm' in extra:
                target_cm = float(extra['target_distance_cm'])
            elif 'target_distance_m' in extra:
                target_cm = 100.0 * float(extra['target_distance_m'])
        if not 5.0 <= target_cm <= 300.0:
            raise ValueError('target distance must be in [5, 300] cm')
        return target_cm

    @staticmethod
    def exit_target_cm(goal):
        """Read the landing-marker-only exit distance from the action goal."""
        target_cm = float(EXIT_STOP_CM)
        if goal.extra_json:
            extra = json.loads(goal.extra_json)
            if 'exit_target_distance_cm' in extra:
                target_cm = float(extra['exit_target_distance_cm'])
        if not 5.0 <= target_cm <= 300.0:
            raise ValueError('exit target distance must be in [5, 300] cm')
        return target_cm

    # ── goal / cancel 콜백 ────────────────────────────────────
    def board_goal_cb(self, goal_request):
        if self.busy:
            return GoalResponse.REJECT
        if goal_request.task_id not in ('board_elevator', 'enter_elevator'):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def exit_goal_cb(self, goal_request):
        if self.busy:
            return GoalResponse.REJECT
        if goal_request.task_id not in ('exit_elevator', 'exit'):
            return GoalResponse.REJECT
        if goal_request.target_floor not in FLOOR_IDS:
            self.get_logger().warn(f'unknown target_floor: {goal_request.target_floor}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.stop()
        return CancelResponse.ACCEPT

    def feedback(self, goal_handle, phase, progress, detail):
        fb = RunTask.Feedback()
        fb.phase = phase
        fb.progress = float(progress)
        fb.detail = detail
        goal_handle.publish_feedback(fb)

    def finish(self, goal_handle, result, ok, msg):
        self.stop()
        self.busy = False
        result.success = ok
        result.message = msg
        if ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        self.get_logger().info(msg)
        return result

    # ── /elevator/board ──────────────────────────────────────
    def execute_board(self, goal_handle):
        self.busy = True
        result = RunTask.Result()
        dt = 1.0 / CONTROL_HZ
        try:
            target_distance_cm = self.board_target_cm(goal_handle.request)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            return self.finish(
                goal_handle, result, False, f'board: invalid extra_json: {exc}')

        # 전방 카메라 활성 보장 + 캐시 초기화
        self.rear_enabled = False
        self.front_seen.clear()
        self.front_pose.clear()

        # phase 1: 문 열림 대기 (BOARD 마커 안정 검출)
        start = time.time()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.busy = False
                result.success = False
                result.message = 'board canceled'
                goal_handle.canceled()
                return result
            if time.time() - start > self.board_wait_timeout_sec:
                return self.finish(goal_handle, result, False, 'board: door-open wait timeout')
            if (self.stable_seen(self.front_seen, BOARD_MARKER_ID)
                    and self.get_fresh_pose(
                        "front", self.front_pose, BOARD_MARKER_ID) is not None):
                self.get_logger().info("문 열림 감지(BOARD 마커) → 서보잉 탑승 시작")
                break
            self.feedback(goal_handle, 'WAIT_DOOR', 0.1, 'waiting board marker')
            time.sleep(dt)

        # phase 2: 전진 탑승 서보잉
        start = time.time()
        init_err = None
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.busy = False
                result.success = False
                result.message = 'board canceled'
                goal_handle.canceled()
                return result
            if time.time() - start > self.servo_timeout_sec:
                return self.finish(goal_handle, result, False, 'board: servo timeout')

            pose = self.get_fresh_pose("front", self.front_pose, BOARD_MARKER_ID)
            if pose is None:
                self.get_logger().warn(
                    "BOARD 마커/카메라 놓침 → 정지(안전)",
                    throttle_duration_sec=2.0)
                self.stop()
                self.feedback(goal_handle, 'BOARDING', 0.5, 'marker lost, holding')
                time.sleep(dt)
                continue
            tz_cm, tx_m = pose
            err = abs(tz_cm - target_distance_cm)
            init_err = init_err or max(err, 1.0)
            self.feedback(goal_handle, 'BOARDING',
                          min(0.2 + 0.8 * (1 - err / init_err), 0.99),
                          f'dist {tz_cm:.1f}cm')
            if self.servo_toward(
                    tz_cm, tx_m, direction=+1,
                    target_cm=target_distance_cm):
                # 탑승 완료 → 후방 카메라로 전환
                self.rear_seen.clear()
                self.rear_pose.clear()
                self.rear_enabled = True
                return self.finish(goal_handle, result, True,
                                   f'boarded, stopped at {tz_cm:.1f}cm')
            time.sleep(dt)

        self.busy = False
        result.success = False
        result.message = 'board: rclpy shutdown'
        goal_handle.abort()
        return result

    # ── /elevator/exit ───────────────────────────────────────
    def execute_exit(self, goal_handle):
        self.busy = True
        result = RunTask.Result()
        goal = goal_handle.request
        target_floor = int(goal.target_floor)
        dt = 1.0 / CONTROL_HZ

        try:
            target_distance_cm = self.exit_target_cm(goal)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            return self.finish(
                goal_handle, result, False,
                f'exit: invalid extra_json: {exc}')

        self.rear_enabled = True  # 단독 테스트 시에도 후방 활성 보장

        # phase 1: 도착층 랜딩 마커 안정 검출 대기 (보통 /floor/check 통과 후라 즉시 통과)
        start = time.time()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.busy = False
                result.success = False
                result.message = 'exit canceled'
                goal_handle.canceled()
                return result
            if time.time() - start > self.exit_wait_timeout_sec:
                return self.finish(goal_handle, result, False, 'exit: floor marker wait timeout')
            if (self.stable_seen(self.rear_seen, target_floor)
                    and self.get_fresh_pose(
                        "rear", self.rear_pose, target_floor) is not None):
                self.get_logger().info(f"도착 문 열림 감지 → {target_floor}층, 후진 하차 시작")
                break
            self.feedback(goal_handle, 'WAIT_FLOOR_MARKER', 0.1,
                          f'waiting floor {target_floor} marker')
            time.sleep(dt)

        # phase 2: 후진 하차 서보잉
        start = time.time()
        init_err = None
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.busy = False
                result.success = False
                result.message = 'exit canceled'
                goal_handle.canceled()
                return result
            if time.time() - start > self.servo_timeout_sec:
                return self.finish(goal_handle, result, False, 'exit: servo timeout')

            pose = self.get_fresh_pose("rear", self.rear_pose, target_floor)
            if pose is None:
                self.get_logger().warn(
                    "층 마커/카메라 놓침 → 정지(안전)",
                    throttle_duration_sec=2.0)
                self.stop()
                self.feedback(goal_handle, 'EXITING', 0.5, 'marker lost, holding')
                time.sleep(dt)
                continue
            tz_cm, tx_m = pose
            err = abs(tz_cm - target_distance_cm)
            init_err = init_err or max(err, 1.0)
            self.feedback(goal_handle, 'EXITING',
                          min(0.2 + 0.6 * (1 - err / init_err), 0.85),
                          f'dist {tz_cm:.1f}cm')
            if self.servo_toward(
                    tz_cm, tx_m, direction=-1,
                    target_cm=target_distance_cm):
                self.get_logger().info(f"하차 완료 ({tz_cm:.1f}cm) → 좌 90도 회전")
                break
            time.sleep(dt)

        # phase 3: 좌 90도 회전 (시간 기반, REP103 +angular.z = 좌회전)
        self.stop_firm()
        rot_start = time.time()
        while rclpy.ok() and time.time() - rot_start < ROTATE_DURATION_SEC:
            if goal_handle.is_cancel_requested:
                self.busy = False
                result.success = False
                result.message = 'exit canceled'
                goal_handle.canceled()
                return result
            self.drive(angular=ROTATE_ANGULAR_SPEED)
            self.feedback(goal_handle, 'ROTATING',
                          0.85 + 0.14 * (time.time() - rot_start) / ROTATE_DURATION_SEC,
                          'rotating left 90deg')
            time.sleep(dt)
        self.stop_firm()

        # 완료 → 전방 카메라로 복귀 (다음 탑승 대비)
        self.rear_enabled = False
        return self.finish(goal_handle, result, True,
                           f'exited at floor {target_floor}, rotated 90deg left')

    # ── 구동 ─────────────────────────────────────────────────
    def drive(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def stop_firm(self):
        for _ in range(8):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = ElevatorServers()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
