#ifndef __GRIPPER_MAPPING_H__
#define __GRIPPER_MAPPING_H__

#include "gripper_shared.h" // CAN 통신부와 공통된 헤더파일 참조

// 1. 방향(Direction) 보정 배열
// 로봇 조립 후 반대로 도는 손가락이 있다면 해당 위치의 1을 -1로 변경
static const int GRIPPER_MOTOR_DIRECTION[GRIPPER_MOTOR_COUNT] = {
    1, 1, 1, // 인덱스 0~2: finger_1 (tip, middle, base)
    1, 1, 1, // 인덱스 3~5: finger_2
    1, 1, 1  // 인덱스 6~8: finger_3
};

/**
 * @brief  [수신용] 서버의 0.01도 단위 각도를 모터의 0~1023 스텝으로 변환
 */
static inline int Convert_Angle_To_Step(int server_motor_id, int target_angle_0_01_deg) 
{
    // 보호: 잘못된 서버 ID가 들어오면 안전하게 팀원이 정한 영점(Home) 반환
    if (server_motor_id < 0 || server_motor_id > 8) return 511;

    // 300도 = 1024스텝 -> 1도 = 약 3.4133스텝 (0.01도 단위 연산)
    float step_diff = (target_angle_0_01_deg * 1024.0f) / 30000.0f;
    
    // ★ 팀원이 설정한 영점(Home Goal) + 질문자님의 방향(Direction) 및 각도 계산 결합!
    int final_step = GRIPPER_SERVO_HOME_GOAL[server_motor_id]
                     + (int)(step_diff * GRIPPER_MOTOR_DIRECTION[server_motor_id]);
                     
    // 안전장치: 하드웨어 한계치(0~1023) 클램핑
    if (final_step < 20) final_step = 20;
    if (final_step > 1000) final_step = 1000;
    
    return final_step;
}

/**
 * @brief  [송신용] 모터의 0~1023 스텝 값을 서버 보고용 0.01도 단위 각도로 역변환
 */
static inline int Convert_Step_To_Angle(int server_motor_id, int current_step) 
{
    if (server_motor_id < 0 || server_motor_id > 8) return 0;

    // 설정한 영점을 기준으로 현재 스텝과의 차이를 구함
    int step_diff = current_step - GRIPPER_SERVO_HOME_GOAL[server_motor_id];
    step_diff *= GRIPPER_MOTOR_DIRECTION[server_motor_id];
    
    // 스텝 당 0.293도를 0.01도 스케일로 역계산
    float angle_0_01_deg = (step_diff * 30000.0f) / 1024.0f;
    
    return (int)angle_0_01_deg;
}

// #ifndef, #define, #endif 세트는 파일이 몇 번 불려가든 컴파일할 때는 딱 한 번만 깔끔하게 포함되도록 보장해 주는 안전장치
#endif /* __GRIPPER_MAPPING_H__ */
