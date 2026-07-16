# Raspberry Pi / Pinky (`pinky`)

모터, 기존 팀 LiDAR bringup, Nav2, 전·후방 V4L2 camera와 주행 task server를
묶는 ROS 2 workspace다.

```bash
./install_dependencies.sh
./build.sh
./run.sh
```

기본 장치는 다음과 같다.

- motor: `/dev/motor`
- front camera: `/dev/video0`
- rear camera: `/dev/video2`

다르면 환경변수 또는 launch 인자로 바꾼다.

```bash
FRONT_VIDEO_DEVICE=/dev/video2 REAR_VIDEO_DEVICE=/dev/video4 ./run.sh
```

`vicpinky_final_bringup`은 원본 `vicpinky_bringup/bringup.launch.xml`을 인자 변경
없이 포함한다. 따라서 LiDAR driver 소스, udev, serial port, frame, filter와 Nav2
scan 설정은 Raspberry Pi에 이미 구성된 팀 설정을 그대로 사용한다. 이 ZIP에는
별도 `sllidar_ros2` 복사본이 없다.

기존 LiDAR package가 별도 Pinky workspace에 있으면 그 환경을 먼저 source한다.

```bash
source ~/vicpinky_ws/install/setup.bash
./run.sh
```

`run.sh`는 `sllidar_ros2`가 ROS 환경에서 보이는지만 확인하며 설정값은 변경하지
않는다.

운영 launch는 Raspberry의 옛 `nav_go_to_server`를 실행하지 않는다. `/nav/go_to`는
중앙 PC의 adapter 하나만 소유한다. 지도 좌표 원본은
`src/vicpinky_task_servers/config/nav_points.yaml`이다.
