#!/usr/bin/env python3
# 엘리베이터 MVP: servoing 없이 cmd_vel + ArUco 검출만으로 동작 확인용
# 상태흐름(FSM): 탑승마커 대기 → 전진탑승 → 운행대기 → floor-id마커 검출 → 후진하차 + 맵로드

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav2_msgs.srv import LoadMap
from cv_bridge import CvBridge
from typing import Optional
from std_msgs.msg import Int32

import cv2
import numpy as np

# 카메라 캘리브레이션 
# 로지텍
FRONT_CAMERA_MATRIX = np.array([
    [708.85065781,   0.0,        308.289349  ],
    [  0.0,        707.92630029, 244.0512732 ],
    [  0.0,          0.0,          1.0       ]
], dtype=np.float64)
FRONT_DIST_COEFFS = np.array(
    [0.02414867, 0.89713946, 0.00248749, -0.01085416, -2.32266594]
)

# Pleomax
REAR_CAMERA_MATRIX = np.array([
    [859.90806761,   0.0,        252.83359168],
    [  0.0,        858.64794316, 248.03325075],
    [  0.0,          0.0,          1.0       ]
], dtype=np.float64)
REAR_DIST_COEFFS = np.array(
    [0.13899504, -0.77334006, 0.00611556, -0.01769451, 2.75664301]
)


# ── 설정값 (현장에 맞게 조정) ───────────────────────────────
BOARD_MARKER_ID   = 10          # 탑승용: 캐빈 안쪽 입구 위에 붙인 마커
FRONT_IMAGE_TOPIC = "/front_camera/image_raw"   # 캐빈 안쪽 BOARD 마커용
REAR_IMAGE_TOPIC  = "/rear_camera/image_raw"    # 복도 floor-id 마커용
FLOOR_IDS = [4, 5]


MARKER_LENGTH     = 0.1          # 마커 한 변 길이(m)
TARGET_STOP_CM    = 50.0         # 마커까지 목표 거리(cm)
STOP_TOLERANCE_CM = 3.0          # 허용 오차

MAX_LINEAR        = 0.3          # m/s
MIN_LINEAR        = 0.1          # m/s
KP_LINEAR         = 0.01         # 거리 제어 비례 상수
KP_ANGULAR        = 0.5          # 각도 제어 비례 상수

DEBOUNCE_FRAMES   = 30            # 마커가 N프레임 연속 보여야 "진짜 보임"으로 인정 (오검출 방지)
LOST_FRAMES       = 30            # 마커가 N프레임 연속 안 보여야 "사라짐"으로 인정 (오검출 방지)
CONTROL_HZ        = 10.0
LOST_TZ_FINISH_CM = 80.0          # 이 거리안에서 놓치면 탑승완료로 간주
# ────────────────────────────────────────────────────────────

# FSM 상태 정의
WAIT_BOARD = "WAIT_BOARD"   # 탑승 마커 대기 (= 출발층 문 열림 대기)
BOARDING   = "BOARDING"     # 전진 탑승 중
RIDING     = "RIDING"       # 운행 중, 도착층 floor-id 마커 대기
EXITING    = "EXITING"      # 후진(또는 전진) 하차 중
DONE       = "DONE"
FRONT_STATES = (WAIT_BOARD, BOARDING)
REAR_STATES  = (RIDING, EXITING)

half = MARKER_LENGTH / 2.0
OBJ_POINTS = np.array([
    [-half,  half, 0],
    [ half,  half, 0],
    [ half, -half, 0],
    [-half, -half, 0]
], dtype=np.float64)

class ElevatorMVP(Node):
    def __init__(self):
        super().__init__("elevator_mvp")
        self.bridge = CvBridge()

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.floor_pub = self.create_publisher(Int32, "/floor/arrived", 10)

        # 디바운스(문 열림 판정용)
        self.front_seen = {}
        self.rear_seen  = {}

        # 최신 pose 캐시: {marker_id: (tz_cm, tx_m, frame_idx)}
        self.front_pose = {}
        self.rear_pose  = {}
        self.frame_idx = {"front": 0, "rear": 0}

        self.create_subscription(Image, FRONT_IMAGE_TOPIC,
            lambda m: self.image_cb(m, "front", self.front_seen, self.front_pose,
                                    FRONT_CAMERA_MATRIX, FRONT_DIST_COEFFS),
            qos_profile_sensor_data)
        self.create_subscription(Image, REAR_IMAGE_TOPIC,
            lambda m: self.image_cb(m, "rear", self.rear_seen, self.rear_pose,
                                    REAR_CAMERA_MATRIX, REAR_DIST_COEFFS),
            qos_profile_sensor_data)

        self.state = WAIT_BOARD
        
        self.target_floor: Optional[int] = None


        self.create_timer(1.0 / CONTROL_HZ, self.control_loop)
        self.get_logger().info("ElevatorMVP 시작. 상태=WAIT_BOARD")

    # ── 이미지 콜백: 검출 + pose 계산 ─────────────────────────
    def image_cb(self, msg, cam, seen_dict, pose_dict, camera_matrix, dist_coeffs):
        if cam == "front" and self.state not in FRONT_STATES:
            return
        if cam == "rear"  and self.state not in REAR_STATES:
            return
        self.frame_idx[cam] += 1
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        now_ids = {int(i) for i in ids.flatten()} if ids is not None else set()

        for mid in set(seen_dict) | now_ids:
            seen_dict[mid] = min(seen_dict.get(mid, 0) + 1, DEBOUNCE_FRAMES) if mid in now_ids else max(seen_dict.get(mid, 0) - 1, 0)


        self.get_logger().info(f'detected ids: {now_ids}')

        if ids is None:
            return

        for i, marker_id in enumerate(ids.flatten()):
            img_points = corners[i][0].astype(np.float64)
            ok, rvec, tvec = cv2.solvePnP(
                OBJ_POINTS, img_points, camera_matrix, dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if ok:
                tz_cm = float(tvec[2][0]) * 100.0
                tx_m  = float(tvec[0][0])
                pose_dict[int(marker_id)] = (tz_cm, tx_m, self.frame_idx[cam])

    def stable_seen(self, seen_dict, marker_id) -> bool:
        return seen_dict.get(marker_id, 0) >= DEBOUNCE_FRAMES

    def get_fresh_pose(self, cam, pose_dict, marker_id):
        """최근 프레임에서 검출된 pose만 유효로 인정 (LOST 판단용)"""
        entry = pose_dict.get(marker_id)
        if entry is None:
            return None
        tz_cm, tx_m, seen_frame = entry
        if self.frame_idx[cam] - seen_frame > LOST_FRAMES:
            return None
        return tz_cm, tx_m
    

    # ── 서보잉 제어: 목표거리까지 P control ───────────────────
    def servo_toward(self, tz_cm, tx_m, direction):
        """direction: +1 전진(탑승), -1 후진(하차)"""
        error_cm = tz_cm - TARGET_STOP_CM
        if abs(error_cm) <= STOP_TOLERANCE_CM:
            self.stop()
            return True  # 도착

        linear = KP_LINEAR * abs(error_cm)
        linear = max(MIN_LINEAR, min(MAX_LINEAR, linear))
        angular = -KP_ANGULAR * tx_m  # 마커가 오른쪽에 있으면(tx>0) 왼쪽으로 회전 보정

        self.drive(linear=direction * linear, angular=angular)
        return False

    # ── 제어 루프: FSM 전이 ─────────────────────────────────
    def control_loop(self):
        if self.state == WAIT_BOARD:
            if self.stable_seen(self.front_seen, BOARD_MARKER_ID):
                self.get_logger().info("문 열림 감지(앞: BOARD 마커) → 서보잉 탑승 시작")
                self.state = BOARDING

        elif self.state == BOARDING:
            pose = self.get_fresh_pose("front", self.front_pose, BOARD_MARKER_ID)
            if pose is None:
                self.get_logger().warn("BOARD 마커 놓침 → 정지(안전)")
                self.stop()
                return
            tz_cm, tx_m = pose
            arrived = self.servo_toward(tz_cm, tx_m, direction=+1)
            if arrived:
                self.rear_seen.clear()
                self.rear_pose.clear()
                self.state = RIDING
                self.get_logger().info("탑승 완료 → 운행 대기(RIDING)")

        elif self.state == RIDING:
            for floor in FLOOR_IDS:
                if self.stable_seen(self.rear_seen, floor):
                    self.target_floor = floor
                    self.get_logger().info(f"도착 문 열림 감지 → {floor}층")
                    self.state = EXITING
                    break

        elif self.state == EXITING:
            if self.target_floor is None:
                return
            pose = self.get_fresh_pose("rear", self.rear_pose, self.target_floor)
            if pose is None:
                self.get_logger().warn("층 마커 놓침 → 정지(안전)")
                self.stop()
                return
            tz_cm, tx_m = pose
            arrived = self.servo_toward(tz_cm, tx_m, direction=-1)
            if arrived:
                self.floor_pub.publish(Int32(data=self.target_floor))
                self.state = DONE
                self.get_logger().info(f"하차 완료 → /floor/arrived={self.target_floor} → DONE")

        elif self.state == DONE:
            self.stop()

    def drive(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = ElevatorMVP()
    try:
        rclpy.spin(node)
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