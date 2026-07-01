import sys

from .send_mission import main as send_mission_main


def main():
    """
    기본 데모 미션을 실행한다.

    실제로는 send_mission.py의 main 함수에
    기본 인자들을 넣어서 호출한다.
    """
    demo_args = [
        '--mission-id', 'floor4_to_floor5_demo',
        '--pickup-location', 'room_402',
        '--delivery-location', 'room_501',
        '--target-floor', '5',
        '--object', 'box',
    ]

    return send_mission_main(demo_args)


if __name__ == '__main__':
    sys.exit(main())
