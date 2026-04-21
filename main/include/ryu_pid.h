#pragma once
/*
 * PID 제어 관련 함수와 변수 선언
 * PID 상수는 paramtable_ryu.h의 파라미터 테이블에서 불러오도록 설계되어 있습니다.
 * PID 계산 함수는 run_pid()로, Yaw 각도 랩어라운드 대응이 포함되어 있습니다.
 * 고도 PID는 기압계 데이터가 업데이트될 때마다 계산하도록 main.cpp에서 호출됩니다.
*/
#include "ryu_common_std.h"
#include "ryu_paramtable.h"  // drone_pid_t 및 전역 PID 변수를 사용하기 위해 포함
#include "ryu_config.h" // PID 관련 매크로 정의 (예: PID_MAX_OUT 등)

namespace PID{
extern void reset_pid(drone_pid_t *p) ;
extern float run_pid_angle(drone_pid_t *p, float tar, float cur, float dt, bool is_yaw);
extern float run_pid_rate(drone_pid_t *p, float target_rate, float current_rate, float dt);
extern void sync_pid_from_params();
}