#include "ryu_pid.h"

#include <algorithm>
#include "ryu_config.h"
#include "ryu_ParamTable.h"  // drone_pid_t 및 전역 PID 변수를 사용하기 위해 포함

namespace Controller
{

const char* PID::TAG = "PID";


PID::PID(){
    ESP_LOGI(TAG,"Initializing PID Controller...");
    // PID 구조체 초기화 (기본값 0)
    // 제어기 명칭	       역할	    P (Proportional)	                I (Integral)	D (Derivative)	비고
    // Alt Position     (Outer)	    목표 고도 유지	1.0 ~ 2.0	        0.0	            0.0	단위:       (m) -> (m/s) 변환
    // Alt Rate         (Inner)	    상승/하강 속도 제어	50.0 ~ 100.0	 10.0 ~ 20.0	 0.05	        단위: (m/s) -> PWM 변량
    pid_alt_pos     = { .kp = 1.0f,  .ki = 0.0f,   .kd = 0.0f,  .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f };
    pid_alt_rate    = { .kp = 50.0f, .ki = 10.0f,  .kd = 0.5f,  .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f };

    // 1. 각도 제어용 (Outer Loop) - P값 위주
    // PID 구조체 초기값 (추천 가이드)
    // 제어기 명칭	        역할	        P (Proportional)	I (Integral)	D (Derivative)	비고
    // Angle (Outer)	    각도 유지	    4.5	                0.0	            0.0	            오직 P값만 사용해도
    pid_roll_angle  = { .kp = 4.5f, .ki = 0.0f,  .kd = 0.0f,  .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f };
    pid_pitch_angle = { .kp = 4.5f, .ki = 0.0f,  .kd = 0.0f,  .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f };
    pid_yaw_angle   = { .kp = 3.0f, .ki = 0.0f,  .kd = 0.0f,  .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f }; // Yaw는 조금 낮게

    // 2. 각속도 제어용 (Inner Loop) - 실제 기체 반응 결정
    // 제어기 명칭	        역할	        P (Proportional)	I (Integral)	D (Derivative)	비고
    // Rate (Inner)	        진동/회전 제어	0.15	            0.1	            0.003	        가장 정밀하게 튜닝 필요
    // Yaw Rate	회전        속도 제어	    0.20	            0.05	        0.0	            값은 거의 사용 안 함

    pid_roll_rate   = { .kp = 0.15f, .ki = 0.1f,  .kd = 0.003f,  .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f };
    pid_pitch_rate  = { .kp = 0.15f, .ki = 0.1f,  .kd = 0.003f,  .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f };
    pid_yaw_rate    = { .kp = 0.25f, .ki = 0.05f, .kd = 0.0f  ,  .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f };
}

PID::~PID(){}

esp_err_t PID::initialize()
{
    if(_initialized) return ESP_OK;
    //
    _initialized = true;
    ESP_LOGI(TAG,"Initialized successfully.");
    return ESP_OK;
}

void PID::reset_pid(drone_pid_t *p)
{
    p->integral = 0.0f;
    p->err_prev = 0.0f;
}

float PID::run_pid_angle(drone_pid_t *p, float tar, float cur, float dt, bool is_yaw)
{
     if (dt <= 0.0f) return 0.0f;

    float error = tar - cur;

    // 1. 오차 계산 직후에 정규화 수행 (이래야 P, I, D 모든 항에 올바른 오차가 적용됨)
    if (is_yaw) {
        while (error > 180.0f) error -= 360.0f;
        while (error < -180.0f) error += 360.0f;
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


float PID::run_pid_rate(drone_pid_t *p, float target_rate, float current_rate, float dt)
{
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

void PID::sync_pid_from_params()
{
    auto& p_mgr = Service::ParamMgr::get_instance();
    auto& values = p_mgr.get_values();

    // Roll / Pitch / Yaw 각 항의 비례, 적분, 미분 계수
    pid_roll_angle.kp   = values.MC_ROLL_P;
    pid_roll_angle.ki   = values.MC_ROLLRATE_I; // rate I를 재활용
    pid_roll_angle.kd   = values.MC_ROLLRATE_D;

    pid_pitch_angle.kp  = values.MC_PITCH_P;
    pid_pitch_angle.ki  = values.MC_PITCHRATE_I;
    pid_pitch_angle.kd  = values.MC_PITCHRATE_D;

    pid_yaw_angle.kp    = values.MC_YAW_P;
    pid_yaw_angle.ki    = values.MC_YAWRATE_I;
    pid_yaw_angle.kd    = 0.0f; // 별도 D 없음

    // 고도용 파라미터가 있다면 여기에 추가 (MPC_* 항목을 예시로 사용)
    pid_alt_pos.kp      = values.MPC_Z_P;
    pid_alt_pos.ki      = values.MPC_Z_VEL_I_ACC;
    pid_alt_pos.kd      = values.MPC_Z_VEL_D_ACC;

    pid_alt_rate.kp     = values.MPC_Z_VEL_P;
    pid_alt_rate.ki     = values.MPC_Z_VEL_I;
    pid_alt_rate.kd     = values.MPC_Z_VEL_D;
}


} 