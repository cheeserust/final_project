#ifndef __FEETECH_H__
#define __FEETECH_H__

// 1. 통신 프로토콜 및 레지스터 주소 매크로
#define SCS_START             0xFF
#define INST_READ             0x02
#define INST_WRITE            0x03
#define INST_SYNC_WRITE       0x83  // 추가: Sync Write 명령어
#define SCS_GOAL_POS_ADDR     0x2A  // 목표 위치 레지스터 시작 주소
#define SCS_PRESENT_POS_ADDR  0x38  // 현재 위치 레지스터 시작 주소

#include <stdint.h> // uint8_t, int16_t 등 자료형 사용을 위해 추가

// 2. 외부에서 사용할 함수 선언
void Feetech_Set_Position_Time(unsigned char id, unsigned short position, unsigned short time);
int Feetech_Read_Pos_Load(unsigned char id, int *p_pos, int *p_load);

// 추가: 9개 모터 동시 제어용 Sync Write 함수
void Feetech_Sync_Write_Pos_Time(uint8_t num_motors, uint8_t* ids, int16_t* positions, uint16_t* times);
// #ifndef, #define, #endif 세트는 파일이 몇 번 불려가든 컴파일할 때는 딱 한 번만 깔끔하게 포함되도록 보장해 주는 안전장치
#endif /* __FEETECH_H__ */