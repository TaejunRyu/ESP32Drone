#include "ryu_pid.h"
#include <algorithm>
namespace PID
{
// PID 리셋: 적분값과 이전 오차를 초기화
void reset_pid(drone_pid_t* p) {
    p->integral = 0.0f;
    p->err_prev = 0.0f;
}
float run_pid_angle(drone_pid_t *p, float tar, float cur, float dt, bool is_yaw){
    if (dt <= 0.0f) return 0.0f;

    float error = tar - cur;

    // 1. 오차 계산 직후에 정규화 수행 (이래야 P, I, D 모든 항에 올바른 오차가 적용됨)
    if (is_yaw) {
        if (error > 180.0f) error -= 360.0f;
        else if (error < -180.0f) error += 360.0f;
    }

    // [Safety Check] 센서 에러(Hold Mode) 발생 시 처리
    if (g_sys.error_hold_mode || g_sys.manual_hold_mode) {
         p->err_prev = error; // 복구 시 D항 튀는 것 방지 (동기화)
        const float p_out = p->kp * error;
        const float i_out = p->ki * p->integral; // 기존 누적값만 사용 (업데이트 X)
        return (p_out + i_out); 
    }

    //---- 정상동작----
    const float p_out = p->kp * error;

    // 2. 정규화된 error를 기반으로 누적 오차 계산
    p->integral = std::clamp(p->integral + (error * dt), -50.0f, 50.0f);
    const float i_out = p->ki * p->integral;

    // 3. 정규화된 error를 기반으로 변화량 계산
    const float d_out = p->kd * (error - p->err_prev) / dt;
    p->err_prev = error;

    return (p_out + i_out + d_out);
}

// --- PID 계산 rate
float run_pid_rate(drone_pid_t *p, float target_rate, float current_rate, float dt) {
    if (dt <= 0.0f) return 0.0f;
    float error = target_rate - current_rate;

   // [Safety Check] I2C 에러 및 복구 중 처리
    if (g_sys.error_hold_mode || g_sys.manual_hold_mode) {
        p->prev_rate = current_rate; // 센서 복구 시 D항 폭주 방지 (동기화)
        float p_out = p->kp * error;
        float i_out = std::clamp(p->ki * p->integral, -150.0f, 150.0f); // 기존 I값 유지
        return p_out + i_out;
    }

    //-----정상동작 -----
    // P 항
    float p_out = p->kp * error;

    // I 항 (Anti-Windup 적용)
    p->integral += error * dt;
    // 출력 기준으로 I항 제한 (예: 모터 출력의 최대 15%까지만 담당)
    float i_out = std::clamp(p->ki * p->integral, -150.0f, 150.0f); 

    // D 항 (Measurement Derivative: 목표값 변화가 아닌 실제 센서 변화 기반)
    // 오차 변화량 대신 '현재 각속도 변화'를 쓰면 스틱을 급격히 움직일 때 튀는 현상이 줄어듭니다.
    float d_out = p->kd * (p->prev_rate - current_rate) / dt;
    p->prev_rate = current_rate;

    return p_out + i_out + d_out;
}

// PID 파라미터 동기화 구현 (일반 함수)
void sync_pid_from_params() {
    // Roll / Pitch / Yaw 각 항의 비례, 적분, 미분 계수
    pid_roll_angle.kp   = PARAM::values.MC_ROLL_P;
    pid_roll_angle.ki   = PARAM::values.MC_ROLLRATE_I; // rate I를 재활용
    pid_roll_angle.kd   = PARAM::values.MC_ROLLRATE_D;

    pid_pitch_angle.kp  = PARAM::values.MC_PITCH_P;
    pid_pitch_angle.ki  = PARAM::values.MC_PITCHRATE_I;
    pid_pitch_angle.kd  = PARAM::values.MC_PITCHRATE_D;

    pid_yaw_angle.kp    = PARAM::values.MC_YAW_P;
    pid_yaw_angle.ki    = PARAM::values.MC_YAWRATE_I;
    pid_yaw_angle.kd    = 0.0f; // 별도 D 없음

    // 고도용 파라미터가 있다면 여기에 추가 (MPC_* 항목을 예시로 사용)
    pid_alt_pos.kp      = PARAM::values.MPC_Z_P;
    pid_alt_pos.ki      = PARAM::values.MPC_Z_VEL_I_ACC;
    pid_alt_pos.kd      = PARAM::values.MPC_Z_VEL_D_ACC;

    // pid_alt_rate.kp
    // pid_alt_rate.kd
    // pid_alt_rate.ki
}
}