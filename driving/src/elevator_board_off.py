#!/usr/bin/env python3
# 엘리베이터 MVP: servoing 없이 cmd_vel + ArUco 검출만으로 동작 확인용
# 상태흐름(FSM): 탑승마커 대기 → 전진탑승 →(180°회전 = 보류)→ 운행대기 → floor-id마커 검출 → 후진하차 + 맵로드

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav2_msgs.srv import LoadMap
from cv_bridge import CvBridge
import cv2
import math

# ── 설정값 (현장에 맞게 조정) ───────────────────────────────
BOARD_MARKER_ID   = 10          # 탑승용: 캐빈 안쪽 입구 위에 붙인 마커
FRONT_IMAGE_TOPIC = "/front_camera/image_raw"   # 캐빈 안쪽 BOARD 마커용
REAR_IMAGE_TOPIC  = "/rear_camera/image_raw"    # 복도 floor-id 마커용
FLOOR_MAP = {                    # floor-id 마커 → 로드할 맵 (floor_markers.yaml에서 가져오는 게 정석)
    4: "maps/floor4.yaml",
    5: "maps/floor5.yaml",
}
DRIVE_SPEED       = 0.10         # m/s, 안전하게 느리게
TURN_SPEED        = 0.5          # rad/s
BOARD_DRIVE_SEC   = 3.0          # 전진 탑승 시간(거리 = 속도×시간). 캐빈 깊이에 맞게
EXIT_DRIVE_SEC    = 3.0          # 하차 주행 시간

#turning 관련 => 일단 보류
#TURN_AFTER_BOARD  = False        # (A)안 쓰려면 True. False면 후진하차.

DEBOUNCE_FRAMES   = 5            # 마커가 N프레임 연속 보여야 "진짜 보임"으로 인정 (오검출 방지)
IMAGE_TOPIC       = "/image_raw" # 카메라 노드(v4l2_camera 등) 토픽명에 맞게
CONTROL_HZ        = 10.0
# ────────────────────────────────────────────────────────────

# FSM 상태 정의
WAIT_BOARD = "WAIT_BOARD"   # 탑승 마커 대기 (= 출발층 문 열림 대기)
BOARDING   = "BOARDING"     # 전진 탑승 중

#turning 관련 => 일단 보류
#TURNING    = "TURNING"      # (옵션) 180° 회전 중

RIDING     = "RIDING"       # 운행 중, 도착층 floor-id 마커 대기
EXITING    = "EXITING"      # 후진(또는 전진) 하차 중
DONE       = "DONE"


class ElevatorMVP(Node):
    def __init__(self):
        super().__init__("elevator_mvp")
        self.bridge = CvBridge()

        # ArUco 검출기 (OpenCV 4.7+ 신 API). pose 계산 안 함 → "마커 보이는가"만 판단.
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

        # I/O
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Image, IMAGE_TOPIC, self.image_cb, qos_profile_sensor_data)
        self.map_cli = self.create_client(LoadMap, "/map_server/load_map")

        # 상태 변수
        self.state = WAIT_BOARD

        # 아래 두 줄은 카메라 하나용 --> 일단 보류 (삭제 예정) 
        #self.visible_ids = set()      # 현재 프레임에서 보이는 마커 ID들
        #self.seen_count = {}          # 마커별 연속 검출 카운트(디바운스)


        self.phase_start = None       # 현재 동작(주행/회전) 시작 시각
        self.target_floor = None      # 검출된 도착층

        # 제어 루프 (이미지 콜백과 분리: 검출은 콜백, 판단/주행은 타이머)
        self.create_timer(1.0 / CONTROL_HZ, self.control_loop)
        self.get_logger().info("ElevatorMVP 시작. 상태=WAIT_BOARD")

    # ── 이미지 콜백: 검출만 담당 ─────────────────────────────
    def image_cb(self, msg, seen_dict):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, ids, _ = self.detector.detectMarkers(gray)
        now = {int(i) for i in ids.flatten()} if ids is not None else set()
        # 해당 카메라의 디바운스 카운트만 갱신
        for mid in set(seen_dict) | now:
            seen_dict[mid] = seen_dict.get(mid, 0) + 1 if mid in now else 0

    def stable_seen(self, seen_dict, marker_id) -> bool:
        return seen_dict.get(marker_id, 0) >= DEBOUNCE_FRAMES

    # ── 제어 루프: FSM 전이 ─────────────────────────────────
    def control_loop(self):
        t = self.get_clock().now().nanoseconds * 1e-9  # 현재 시각(초)

        if self.state == WAIT_BOARD:
            if self.stable_seen(self.front_seen, BOARD_MARKER_ID):   # 앞 카메라
                self.get_logger().info("문 열림 감지(앞: BOARD 마커) → 전진 탑승")
                self.phase_start = t
                self.state = BOARDING
    
        elif self.state == BOARDING:
            if t - self.phase_start < BOARD_DRIVE_SEC:
                self.drive(linear=DRIVE_SPEED) # 전진
            else:
                self.stop()
                self.rear_seen.clear() # 후방 카메라 디바운스 카운트 초기화
                self.state = RIDING    # 탄 상태 운행 대기
                self.get_logger().info("탑승 완료 → 운행 대기(RIDING)")
        
        
            """
            회전관련 = 일단 보류
            elif self.state == TURNING:
                turn_sec = math.pi / TURN_SPEED              # 180° 도는 데 걸리는 시간
                if t - self.phase_start < turn_sec:
                    self.drive(angular=TURN_SPEED)
                else:
                    self.stop()
                    self.state = RIDING
                    self.get_logger().info("회전 완료 → 운행 대기(RIDING)")
            """
        
        
        elif self.state == RIDING:
            # 층 이동 후 문이 열리면 바깥 floor-id 마커가 보인다.
            for floor in FLOOR_MAP:
                marker_id = floor                        # 여기선 floor-id 마커 ID = 층번호로 가정
                if self.stable_seen(self.rear_seen, marker_id):
                    self.target_floor = floor
                    self.get_logger().info(f"도착 문 열림 감지 → {floor}층")
                    self.phase_start = t
                    self.state = EXITING
                    break

        elif self.state == EXITING:
            # 후진으로 하차
            # 회전 관련 코드는 보류
            # exit_speed = DRIVE_SPEED if TURN_AFTER_BOARD else -DRIVE_SPEED
            
            if t - self.phase_start < EXIT_DRIVE_SEC:
                self.drive(linear=exit_speed)
            else:
                self.stop()
                self.load_map(FLOOR_MAP[self.target_floor])  # 하차 직후 해당 층 맵 로드
                # TODO(다음 단계): 저장된 출구 pose를 /initialpose로 publish → AMCL seed
                self.state = DONE
                self.get_logger().info("하차 완료 + 맵 로드 요청 → DONE")

        elif self.state == DONE:
            self.stop()

    # ── 헬퍼 ────────────────────────────────────────────────
    def drive(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def stop(self):
        self.cmd_pub.publish(Twist())  # 전부 0

    def load_map(self, map_url):
        if not self.map_cli.service_is_ready():
            self.get_logger().warn("/map_server/load_map 서비스 없음")
            return
        req = LoadMap.Request()
        req.map_url = map_url                          # 맵 yaml 경로
        future = self.map_cli.call_async(req)
        future.add_done_callback(self._on_map_loaded)

    def _on_map_loaded(self, future):
        res = future.result()
        ok = (res is not None and res.result == LoadMap.Response.RESULT_SUCCESS)
        self.get_logger().info(f"맵 로드 결과: {'성공' if ok else '실패'}")


def main():
    rclpy.init()
    node = ElevatorMVP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()