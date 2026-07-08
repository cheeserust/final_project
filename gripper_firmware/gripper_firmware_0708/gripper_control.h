#ifndef __GRIPPER_CONTROL_H__
#define __GRIPPER_CONTROL_H__

// 메인 루프에서 지속적으로 호출될 핵심 태스크
void Gripper_Control_Task(void);
// CAN통신 없이 테스트 코드 함수
void Inject_Mock_Command(int step);

#endif /* __GRIPPER_CONTROL_H__ */