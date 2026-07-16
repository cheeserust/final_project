# 중앙 PC (`pj`)

USB2CAN-FD/STM 3개 보드, wrist RealSense, MoveIt2, Mission Manager, GUI가
동작하는 ROS 2 workspace다. `/nav/go_to` adapter도 여기에서만 실행하며
Raspberry Pi에는 중복 서버를 띄우지 않는다.

```bash
./install_dependencies.sh
./build.sh
./run.sh
```

기본값은 `can0`, hardware 실행, GUI `0.0.0.0:8080`이다. 안전한 launch 해석이나
RViz 확인만 할 때는 다음처럼 motion goal을 거절하는 plan-only 모드를 쓴다.

Arm bridge는 frame 간 `7 ms`, 일반 point 최소 `40 ms`를 사용한다. Homing은
Board1 J1을 firmware 명령 가능 범위인 `-85°`까지 자동으로 이동한 뒤 성공하며,
이 단계가 실패하면 mission의 `ARM_READY_AT_PICKUP`으로 진행하지 않는다.

```bash
EXECUTION_MODE=plan_only ./run.sh
```

전체 mock 시나리오는 하드웨어 없이 다음으로 확인할 수 있다.

```bash
./test_mock.sh
```

`aruco_markers/`의 ID 50~55는 검출용 원본이다. 출력 규칙은 그 폴더의
README를 따른다.
