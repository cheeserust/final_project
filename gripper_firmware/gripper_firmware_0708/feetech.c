#include "feetech.h"        // 내 헤더 파일 포함
#include "device_driver.h"  // Uart1_Send_Binary 등 UART 제어 함수 사용을 위해 포함

// 체크섬 계산 함수 (이 파일 안에서만 쓸 거라면 static을 붙여서 숨기는 게 좋음)
// 체크섬 계산: ~(ID + Length + Instruction + Address + Params의 합)
static unsigned char Calculate_Checksum(unsigned char id, unsigned char length, unsigned char inst, unsigned char addr, unsigned char* params, int param_len) 
{
    int sum = id + length + inst + addr;
    for (int i = 0; i < param_len; i++) 
    {
        sum += params[i];
    }
    return (unsigned char)(~sum);
}

void Feetech_Set_Position_Time(unsigned char id, unsigned short position, unsigned short time) 
{
    unsigned char params[4];
    unsigned char length = 7;
    
    // 비트 연산 확인: position 511(0x01FF)이면 -> params[0]=0xFF, params[1]=0x01
    // params[0] = (unsigned char)(position & 0xFF);         
    // params[1] = (unsigned char)((position >> 8) & 0xFF);

	params[0] = (unsigned char)((position >> 8) & 0xFF); // 상위 바이트 먼저
	params[1] = (unsigned char)(position & 0xFF);        // 하위 바이트 나중에
	params[2] = (unsigned char)((time >> 8) & 0xFF);
	params[3] = (unsigned char)(time & 0xFF);             

    // 체크섬 계산 시 매개변수 확인
    unsigned char checksum = Calculate_Checksum(id, length, INST_WRITE, SCS_GOAL_POS_ADDR, params, 4);

    Uart1_Set_Tx_Mode(); // 추가 전송모드 귀를 닫고 입을 열기

    Uart1_Send_Binary(SCS_START);
    Uart1_Send_Binary(SCS_START);
    Uart1_Send_Binary(id);
    Uart1_Send_Binary(length);
    Uart1_Send_Binary(INST_WRITE);
    Uart1_Send_Binary(SCS_GOAL_POS_ADDR);
    
    // 파라미터 4개 순서대로 전송
    Uart1_Send_Binary(params[0]);
    Uart1_Send_Binary(params[1]);
    Uart1_Send_Binary(params[2]);
    Uart1_Send_Binary(params[3]);
    
    Uart1_Send_Binary(checksum);

    Uart1_Set_Rx_Mode(); // 말이 끝났음을 확인(TC)하고 즉시 귀를 연다
}

/**
 * @brief  SCS0009 모터의 현재 위치와 부하(Load)를 동시에 읽어오는 함수
 * @param  id: 모터 ID 번호
 * @param  p_pos: 위치값을 받아올 변수의 주소 (포인터)
 * @param  p_load: 부하값을 받아올 변수의 주소 (포인터)
 * @retval 통신 성공 시 1, 실패 시 -1 반환
 */
int Feetech_Read_Pos_Load(unsigned char id, int *p_pos, int *p_load) 
{
    unsigned char length = 4; // Inst(1) + Addr(1) + Read_Length(1) + Checksum(1)
    unsigned char read_bytes = 6; // 위치(2) + 속도(2) + 부하(2) = 총 6바이트
    
    // 체크섬 계산: ~(ID + Length + Inst + Addr + 읽을 바이트 수)
    unsigned char checksum = (unsigned char)(~(id + length + INST_READ + SCS_PRESENT_POS_ADDR + read_bytes));

    Uart1_Set_Tx_Mode(); // 추가 전송모드 귀를 닫고 입을 열기

    // 1. 모터에 읽기 명령 패킷 전송
    Uart1_Send_Binary(SCS_START);
    Uart1_Send_Binary(SCS_START);
    Uart1_Send_Binary(id);
    Uart1_Send_Binary(length);
    Uart1_Send_Binary(INST_READ);
    Uart1_Send_Binary(SCS_PRESENT_POS_ADDR); // 56번지부터
    Uart1_Send_Binary(read_bytes);           // 6바이트 요청
    Uart1_Send_Binary(checksum);

    // 2. 에코 현상 제거
    // Uart1_Flush_Rx(); // 에코 현상 제거 원위치

    Uart1_Set_Rx_Mode(); // [추가] 말이 끝났음을 확인(TC)하고 즉시 귀를 연다

    // 3. 응답 패킷 수신
    // 응답 패킷 구조: Start(2) + ID(1) + Length(1) + Error(1) + Data(6) + Checksum(1) = 총 12바이트
    unsigned char rx_buf[12];

    // 통신 지연 유발하므로 제거
    // __disable_irq(); // [추가] UART 수신 중 CAN 인터럽트 개입 차단 (Overrun 방지)

    for (int i = 0; i < 12; i++) 
    {
        // 첫 바이트는 모터가 응답할 때까지 넉넉히 기다리고(40000)
        // 그 다음 바이트부터는 연속해서 들어오므로 타임아웃을 줄여(3000) 블로킹 방지
        int timeout_val = (i == 0) ? 40000 : 3000;

        int incoming_byte = Uart1_Get_Char_Timeout(timeout_val);
        if (incoming_byte == -1) 
        {
            // __enable_irq(); // disable을 제거했으므로 여기도 제거
            return -1; // 타임아웃 발생
        }
        rx_buf[i] = (unsigned char)incoming_byte;
    }
    // __enable_irq(); // [추가] 12바이트 수신이 무사히 끝나면 인터럽트 복구 // 통신 지연 유발하므로 제거
    // 4. 데이터 파싱 및 복원
    if (rx_buf[0] == SCS_START && rx_buf[1] == SCS_START && rx_buf[2] == id) 
    {
        unsigned char error = rx_buf[4];
        if (error == 0) 
        {
            // 수신 데이터 순서: [5][6]=위치, [7][8]=속도, [9][10]=부하 (빅엔디안)
            unsigned char pos_H = rx_buf[5]; 
            unsigned char pos_L = rx_buf[6];
            
            // 속도 데이터(7, 8번 인덱스)는 현재 필요 없으니 건너뜀
            
            unsigned char load_H = rx_buf[9];
            unsigned char load_L = rx_buf[10];
            
            // 10비트 데이터 복원
            int current_pos = (pos_H << 8) | pos_L;
            int raw_load = (load_H << 8) | load_L;
            
            // 부하(Load) 값 마스킹 처리
            // Feetech 모터의 Load 값은 최상위 비트(Bit 10)가 방향을 의미함
            // '모터가 얼마나 힘을 쓰고 있는지' 크기만 필요하므로 하위 10비트만 추출 (0x03FF = 1023)
            int current_load = raw_load & 0x03FF; 
            
            // 포인터를 통해 메인 함수로 값 전달
            *p_pos = current_pos;
            *p_load = current_load;
            
            return 1; // 성공
        }
    }
    
    return -1; // 패킷 오류 또는 에러 감지
}

void Feetech_Sync_Write_Pos_Time(uint8_t num_motors, uint8_t* ids, int16_t* positions, uint16_t* times)
{
    uint8_t data_len = 4; // 모터 1개당 데이터 길이: Pos(2) + Time(2) = 4바이트
    
    // 패킷 길이 계산: (Data_Length + 1(ID)) * 모터 개수 + 4(명령어 길이 등)
    uint8_t pkt_len = (data_len + 1) * num_motors + 4; 
    
    // 체크섬 초기 누적값 계산 (ID(0xFE) + Length + Inst + Addr + DataLen)
    int sum = 0xFE + pkt_len + INST_SYNC_WRITE + SCS_GOAL_POS_ADDR + data_len;

    // 여기에 추가: 전송 모드로 전환 (귀 닫고 입 열기)
    Uart1_Set_Tx_Mode();

    // 1. 헤더 및 명령어 전송
    Uart1_Send_Binary(SCS_START);           // 0xFF
    Uart1_Send_Binary(SCS_START);           // 0xFF
    Uart1_Send_Binary(0xFE);                // Broadcast ID (Sync Write는 무조건 0xFE)
    Uart1_Send_Binary(pkt_len);             // Packet Length
    Uart1_Send_Binary(INST_SYNC_WRITE);     // Instruction (0x83)
    Uart1_Send_Binary(SCS_GOAL_POS_ADDR);   // Start Address (0x2A)[cite: 5]
    Uart1_Send_Binary(data_len);            // Data Length per Servo (4)

    // 2. 모터별 ID 및 파라미터(Pos, Time) 전송
    for (int i = 0; i < num_motors; i++) {
        uint8_t id = ids[i];
        
        // ★ SCS0009 방식 적용: 상위 바이트(High) 먼저, 하위 바이트(Low) 나중에[cite: 6]
        uint8_t pos_H = (uint8_t)((positions[i] >> 8) & 0xFF);
        uint8_t pos_L = (uint8_t)(positions[i] & 0xFF);
        uint8_t time_H = (uint8_t)((times[i] >> 8) & 0xFF);
        uint8_t time_L = (uint8_t)(times[i] & 0xFF);

        Uart1_Send_Binary(id);
        Uart1_Send_Binary(pos_H);
        Uart1_Send_Binary(pos_L);
        Uart1_Send_Binary(time_H);
        Uart1_Send_Binary(time_L);

        // 체크섬 계산을 위해 파라미터 누적 합산
        sum += id + pos_H + pos_L + time_H + time_L;
    }

    // 3. 체크섬 전송
    Uart1_Send_Binary((unsigned char)(~sum & 0xFF));

    // 여기에 추가: 송신 완료 후 수신 모드로 복귀 (입 닫고 귀 열기)
    Uart1_Set_Rx_Mode();
}