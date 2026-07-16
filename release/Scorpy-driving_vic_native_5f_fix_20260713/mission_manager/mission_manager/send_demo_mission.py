import sys

from .send_mission import main as send_mission_main


def main():
    """
    기본 데모 미션을 실행한다.

    실제로는 send_mission.py의 main 함수에
    기본 인자들을 넣어서 호출한다.
    """
    demo_args = [
        '--mission-id', 'final_4f_5f_demo',
        '--pickup-location', 'object',
        '--delivery-location', 'object_place',
        '--target-floor', '5',
        '--object', 'box',
    ]

    return send_mission_main(demo_args)


if __name__ == '__main__':
    sys.exit(main())
