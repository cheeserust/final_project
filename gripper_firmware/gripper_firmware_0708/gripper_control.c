#include "gripper_control.h"
#include "gripper_shared.h"      // 공유 칠판(g_state, g_cmd)
#include "gripper_mapping.h"     // 각도 변환
#include "feetech.h"             // 모터 통신
#include "device_driver.h"
#include <stdlib.h>              // abs()
#include <stdio.h>

// ---------------------------------------------------------
// [상태 머신 및 안전 변수 선언]
// ---------------------------------------------------------
// 각 모터가 어느 방향으로 막혔는지 기억하는 배열 (0: 안막힘, 1: +방향, -1: -방향)
static volatile int blocked_direction[GRIPPER_MOTOR_COUNT] = {0};

// 논블로킹 제어를 위한 모터별 상태 정의
typedef enum { 
    MOTOR_NORMAL,         // 평상시 (명령 수행 및 모니터링)
    MOTOR_BACKOFF_WAIT,   // 부하 감지됨! 3ms 대기 중
    MOTOR_BACKOFF_DONE    // 3ms 후 후퇴 명령 전송 완료
} MotorState_t;

static MotorState_t m_state[GRIPPER_MOTOR_COUNT] = {MOTOR_NORMAL, };
static uint32_t m_wait_start_time[GRIPPER_MOTOR_COUNT] = {0, };

// 외부(main.c)에 있는 1ms 타이머 변수
extern volatile uint32_t g_ms_tick; 

// ★ 인터럽트로부터 안전하게 보호될 전용 수첩 (복사본)
static GripperCommand current_cmd; 


// ---------------------------------------------------------
// [그리퍼 제어 핵심 태스크] - 무한 루프에서 호출됨
// ---------------------------------------------------------
void Gripper_Control_Task(void)
{
    // 1. ESTOP(비상정지) 또는 Disable 상태면 모터 제어 무시
    if (g_state.state == STATE_ESTOP || g_state.state == STATE_DISABLED || g_state.enabled == 0) {
        return; 
    }
    
    // 2. 새로운 명령 수신 시 동작 (CAN 팀원이 깃발을 들었을 때)
    if (g_cmd.is_new_cmd == 1) {
        
        // ★ [안전 구역] 인터럽트 끄고 공용 칠판(g_cmd)에서 전용 수첩(current_cmd)으로 복사
        __disable_irq();
        current_cmd = g_cmd; 
        g_cmd.is_new_cmd = 0; // 깃발 내리기
        __enable_irq();
        
        // 에러/Fault 상태가 아닐 때만 이동 시작
        if (g_state.fault == 0) {
            g_state.state = STATE_MOVING; 
            
            // ★ 이제부터는 g_cmd 대신 current_cmd를 사용합니다!
            uint16_t real_time_ms = GRIPPER_DURATION_5MS_TO_MS(current_cmd.duration_5ms);

            uint8_t  sync_ids[GRIPPER_MOTOR_COUNT];
            int16_t  sync_positions[GRIPPER_MOTOR_COUNT];
            uint16_t sync_times[GRIPPER_MOTOR_COUNT];

            for (int i = 0; i < GRIPPER_MOTOR_COUNT; i++) {
                uint8_t real_id = GRIPPER_MOTOR_ID_TO_SERVO_ID[i];
                
                // ★ current_cmd 사용
                int target_step = Convert_Angle_To_Step(i, current_cmd.target_pos_001deg[i]);
                int current_step = g_state.current_pos[i]; 

                // ★ [수정된 방어 로직] 이미 부하를 받아 멈춘 방향으로 계속 밀어붙이는 명령인지 확인
                int is_pushing_further = 0;

                // +방향으로 막혔는데, 현재 위치보다 더 큰(또는 같은) 값을 요구할 때
                if (blocked_direction[i] == 1 && target_step >= current_step) is_pushing_further = 1;
                // -방향으로 막혔는데, 현재 위치보다 더 작은(또는 같은) 값을 요구할 때
                if (blocked_direction[i] == -1 && target_step <= current_step) is_pushing_further = 1;

                if (is_pushing_further == 1) {
                    // 이미 부하를 받아 멈춘 방향으로 또 가라고 하면, 목표를 현재 위치로 묶어두고 상태(백오프) 유지!
                    target_step = current_step; 
                    // 주의: m_state[i]와 blocked_direction[i]를 리셋하지 않음!
                } 
                else {
                    // 막히지 않았거나, 반대 방향(물건을 놓기 위해 입을 벌림)으로 가는 명령이면 정상적으로 상태 리셋!
                    m_state[i] = MOTOR_NORMAL;
                    blocked_direction[i] = 0;
                }
                
                // 배열에 데이터 기록
                sync_ids[i] = real_id;
                sync_positions[i] = (int16_t)target_step;
                sync_times[i] = real_time_ms;
            }

            // Sync Write로 한 번에 송신 
            Feetech_Sync_Write_Pos_Time(GRIPPER_MOTOR_COUNT, sync_ids, sync_positions, sync_times);
        }
    }

    // 3. 실시간 모니터링 & 상태 머신 적용 (논블로킹 로직)
    int all_arrived = 1;
    
    for (int i = 0; i < GRIPPER_MOTOR_COUNT; i++) {
        uint8_t real_id = GRIPPER_MOTOR_ID_TO_SERVO_ID[i];
        int pos = 0, load = 0;

        switch(m_state[i]) {
            
            // [상태 1] 정상 모니터링
            case MOTOR_NORMAL:
                if (Feetech_Read_Pos_Load(real_id, &pos, &load) == 1) {
                    // 쓰레기값 필터링
                    if (pos == 0 && load == 0) continue; 
                    
                    g_state.current_pos[i] = pos;
                    g_state.current_pos_001deg[i] = (int16_t)Convert_Step_To_Angle(i, pos);

                    // 방어 1: 기구적 한계 이탈 (치명적 오류)
                    if (pos < 20 || pos > 1000) {
                        if (g_state.fault == 0) { 
                            printf(">> [치명적 오류] 모터 %d 기구 한계 이탈! 전체 정지!\n", real_id);
                            for(int j = 0; j < GRIPPER_MOTOR_COUNT; j++) {
                                int stop_pos = g_state.current_pos[j];
                                if (stop_pos >= 20 && stop_pos <= 1000) {
                                    Feetech_Set_Position_Time(GRIPPER_MOTOR_ID_TO_SERVO_ID[j], stop_pos, 0);
                                }
                            }
                            g_state.state = STATE_ERROR;
                            g_state.error_code = ERR_ANGLE_RANGE; 
                            g_state.fault = 1;
                            g_state.ready = 0;
                            g_state.fault_motor_id = i; 
                        }
                        return; // 완전히 뻗어버림
                    }

                    // 방어 2: 부하(Load) 감지 백오프 로직 (★ current_cmd 사용)
                    if (current_cmd.load_control_enabled == 1 && load > current_cmd.load_threshold[i]) {
                        int target_step = Convert_Angle_To_Step(i, current_cmd.target_pos_001deg[i]);

                        // 막힌 방향 기록
                        if (target_step > pos) blocked_direction[i] = 1;
                        else if (target_step < pos) blocked_direction[i] = -1;

                        // 3ms 대기를 위해 상태 머신 전환 (기다리지 않고 바로 넘어감!)
                        m_wait_start_time[i] = g_ms_tick; 
                        m_state[i] = MOTOR_BACKOFF_WAIT; 
                        
                        all_arrived = 0; 
                    }
                }
                break;

            // [상태 2] 3ms 대기 중 (CPU 멈춤 없이 시간만 체크)
            case MOTOR_BACKOFF_WAIT:
                all_arrived = 0; // 아직 처리 중이므로 도착 안 함
                
                // 3ms가 경과했는지 논블로킹으로 확인
                if ((g_ms_tick - m_wait_start_time[i]) >= 3) {
                    
                    int safe_pos = g_state.current_pos[i];
                    if (blocked_direction[i] == 1) safe_pos -= 5;
                    else if (blocked_direction[i] == -1) safe_pos += 5;

                    if (safe_pos < 20) safe_pos = 20;
                    if (safe_pos > 1000) safe_pos = 1000;

                    // 50ms 동안 부드럽게 뒤로 후퇴 (충격 완화)
                    Feetech_Set_Position_Time(real_id, safe_pos, 50); 
                    
                    m_state[i] = MOTOR_BACKOFF_DONE;
                }
                break;

            // [상태 3] 후퇴 완료 후 대기
            case MOTOR_BACKOFF_DONE:
                // 더 이상 명령을 내리지 않고 유지
                break;
        }

        // [상태 기록 로직] - 현재 모터 상태 판별
        uint8_t current_status = 0; 
        
        if (g_state.fault == 1 && g_state.fault_motor_id == i) {
            current_status = 3; // ERROR
        }
        else if (blocked_direction[i] != 0) {
            current_status = 2; // CONTACT_HOLD (부하 정지)
        }
        else if (g_state.state == STATE_MOVING) {
            // ★ current_cmd 사용
            int target_step = Convert_Angle_To_Step(i, current_cmd.target_pos_001deg[i]);
            if (abs(pos - target_step) > 10 && m_state[i] == MOTOR_NORMAL) { 
                current_status = 1; // MOVING
                all_arrived = 0;
            }
        }
        g_state.motor_status[i] = current_status;
    }

    // 4. 모든 모터 도달 확인 시 IDLE 복귀
    if (g_state.state == STATE_MOVING && all_arrived == 1 && g_state.fault == 0) {

        // ★ [여기 전체 수정] 부하를 받아 락이 걸린 모터가 하나라도 있는지 검사
        int is_holding = 0;
        for (int j = 0; j < GRIPPER_MOTOR_COUNT; j++) {
            if (blocked_direction[j] != 0) {
                is_holding = 1;
                break;
            }
        }
        // 락이 걸려있으면 파지 완료(CONTACT_HOLD), 허공이면 IDLE
        if (is_holding == 1) {
            g_state.state = STATE_CONTACT_HOLD;
        } else {
            g_state.state = STATE_IDLE;
        }
    }
}


// ---------------------------------------------------------
// [테스트 전용 함수] CAN 통신이 온 것처럼 가짜 명령을 주입
// ---------------------------------------------------------
void Inject_Mock_Command(int step) 
{
    if (step == 0) {
        // [테스트 1] 그리퍼 열기 (0도)
        printf("\n[TEST] 그리퍼 열기 명령 주입 (0.00도)\n");
        for (int i = 0; i < GRIPPER_MOTOR_COUNT; i++) {
            g_cmd.target_pos_001deg[i] = 0; // 0.00도
        }
    } 
    else if (step == 1) {
        // [테스트 2] 그리퍼 닫기 (20도)
        printf("\n[TEST] 그리퍼 닫기 명령 주입 (-20.00도)\n");
        for (int i = 0; i < GRIPPER_MOTOR_COUNT; i++) {
            // 손가락이 안쪽으로 닫히는 방향의 0.01도 스케일 값 (예: 45도 = 4500)
            g_cmd.target_pos_001deg[i] = -2000;
        }
    }
    else if (step == 2) {
        // [테스트 3] 그리퍼 개별제어 접기
        printf("\n[TEST] 그리퍼 모으기 1\n");
        g_cmd.target_pos_001deg[0] = 0;
        g_cmd.target_pos_001deg[1] = -7000;
        g_cmd.target_pos_001deg[2] = -3000;
        g_cmd.target_pos_001deg[3] = 0;
        g_cmd.target_pos_001deg[4] = -7000;
        g_cmd.target_pos_001deg[5] = -3000;
        g_cmd.target_pos_001deg[6] = 0;
        g_cmd.target_pos_001deg[7] = -7000;
        g_cmd.target_pos_001deg[8] = -3000;
    }
    else if (step == 3) {
        // [테스트 4] 그리퍼 개별제어 접기
        printf("\n[TEST] 그리퍼 모으기 2\n");
        g_cmd.target_pos_001deg[0] = 0;
        g_cmd.target_pos_001deg[1] = -9000;
        g_cmd.target_pos_001deg[2] = -4000;
        g_cmd.target_pos_001deg[3] = 0;
        g_cmd.target_pos_001deg[4] = -9000;
        g_cmd.target_pos_001deg[5] = -4000;
        g_cmd.target_pos_001deg[6] = 0;
        g_cmd.target_pos_001deg[7] = -9000;
        g_cmd.target_pos_001deg[8] = -4000;
    }

    else if (step == 4) {
        // [테스트 4] 그리퍼 개별제어 접기
        printf("\n[TEST] 그리퍼 모으기 2\n");
        g_cmd.target_pos_001deg[0] = 0;
        g_cmd.target_pos_001deg[1] = -12000;
        g_cmd.target_pos_001deg[2] = 2500;
        g_cmd.target_pos_001deg[3] = 0;
        g_cmd.target_pos_001deg[4] = -12000;
        g_cmd.target_pos_001deg[5] = 2500;
        g_cmd.target_pos_001deg[6] = 0;
        g_cmd.target_pos_001deg[7] = -12000;
        g_cmd.target_pos_001deg[8] = 2500;
    }

    else if (step == 5) {
        // [테스트 4] 그리퍼 개별제어 접기
        printf("\n[TEST] 그리퍼 모으기 2\n");
        g_cmd.target_pos_001deg[0] = 0;
        g_cmd.target_pos_001deg[1] = -12500;
        g_cmd.target_pos_001deg[2] = 2000;
        g_cmd.target_pos_001deg[3] = 0;
        g_cmd.target_pos_001deg[4] = -12500;
        g_cmd.target_pos_001deg[5] = 2000;
        g_cmd.target_pos_001deg[6] = 0;
        g_cmd.target_pos_001deg[7] = -12500;
        g_cmd.target_pos_001deg[8] = 2000;
    }

    else if (step == 6) {
        // [테스트 4] 그리퍼 개별제어 접기
        printf("\n[TEST] 그리퍼 모으기 2\n");
        g_cmd.target_pos_001deg[0] = 0;
        g_cmd.target_pos_001deg[1] = -13000;
        g_cmd.target_pos_001deg[2] = 2000;
        g_cmd.target_pos_001deg[3] = 0;
        g_cmd.target_pos_001deg[4] = -13000;
        g_cmd.target_pos_001deg[5] = 2000;
        g_cmd.target_pos_001deg[6] = 0;
        g_cmd.target_pos_001deg[7] = -13000;
        g_cmd.target_pos_001deg[8] = 2000;
    }
    
    // 동작 시간: 100 * 5ms = 500ms 동안 부드럽게 이동
    g_cmd.duration_5ms = 100; 
    // 중요 테스트 모드일 때 load_threshold가 0이 되지 않도록 넉넉한 값(500)으로 초기화
    for (int i = 0; i < GRIPPER_MOTOR_COUNT; i++) {
        g_cmd.load_threshold[i] = GRIPPER_DEFAULT_LOAD_THRESHOLD; 
    }
    g_cmd.load_control_enabled = 1;
    // ★ CAN 통신이 수신 완료된 것처럼 깃발을 강제로 올림!
    g_cmd.is_new_cmd = 1;
}