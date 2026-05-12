#include "ryu_pid.h"

#include <algorithm>
#include "ryu_config.h"
#include "ryu_ParamTable.h"  // drone_pid_t 및 전역 PID 변수를 사용하기 위해 포함

namespace Controller
{

esp_err_t PID::initialize()
{
    if(_initialized) return ESP_OK;

    // PID 구조체 초기화 (기본값 0)
    // 제어기 명칭	       역할	    
    // Alt Position     (Outer)	    목표 고도 유지	
    // Alt Rate         (Inner)	    상승/하강 속도 제어	
    // pid_alt_pos     = { .kp = 1.0f, .ki = 0.0f,   .kd = 0.0f,   .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f, .d_out_filt =0.0f};
    // pid_alt_rate    = { .kp = 50.0f,.ki = 15.0f,  .kd = 0.5f,   .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f, .d_out_filt =0.0f};


    // Outer Loop: 고도 제어 (P위주)
    pid_alt_pos = { 
        .kp = 1.2f,        // 1.0 ~ 1.5 사이 유지 (반응이 느리면 소폭 상승)
        .ki = 0.0f, .kd = 0.0f, .integral = 0.0f, .err_prev = 0.0f, .prev_rate = 0.0f, .d_out_filt = 0.0f
    };

    // Inner Loop: 수직 속도 제어 (400Hz 환경 맞춤형)
    pid_alt_rate = { 
        .kp = 60.0f,       // 기체가 고도를 못 버티고 흐르면 80까지 서서히 상승
        .ki = 25.0f,       // 호버링 스로틀 오프셋 누적을 위해 기존보다 상향
        .kd = 1.5f,        // 400Hz 분모 대응을 위해 기존 0.5에서 상향 조정
        .integral = 0.0f, .err_prev = 0.0f, .prev_rate = 0.0f, 
        .d_out_filt = 0.0f 
    };



    // 1. 각도 제어용 (Outer Loop) - P값 위주
    // PID 구조체 초기값 (추천 가이드)
    // 제어기 명칭	        역할	        
    // Angle (Outer)	   각도 유지	    
    pid_roll_deg    = { .kp = 3.8f, .ki = 0.0f,  .kd = 0.0f,    .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f, .d_out_filt =0.0f};
    pid_pitch_deg   = { .kp = 3.8f, .ki = 0.0f,  .kd = 0.0f,    .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f, .d_out_filt =0.0f};
    pid_yaw_deg     = { .kp = 2.5f, .ki = 0.0f,  .kd = 0.0f,    .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f, .d_out_filt =0.0f};

    // 2. 각속도 제어용 (Inner Loop) - 실제 기체 반응 결정
    // 제어기 명칭	        역할	        
    // Rate (Inner)	       진동/회전 제어	
    // Yaw Rate	회전       속도 제어	    
    pid_roll_rate   = { .kp = 0.15f,.ki = 0.12f, .kd = 0.005f,  .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f, .d_out_filt =0.0f};
    pid_pitch_rate  = { .kp = 0.15f,.ki = 0.12f, .kd = 0.005f,  .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f, .d_out_filt =0.0f};
    pid_yaw_rate    = { .kp = 0.25f,.ki = 0.08f, .kd = 0.0f,    .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f, .d_out_filt =0.0f};


    _initialized = true;
    ESP_LOGI(TAG,"Initialized successfully.");
    return ESP_OK;
}

void PID::reset_pid_iterm(drone_pid_t *p) {
    p->integral = 0.0f;
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
        error = fmodf(error + 180.0f, 360.0f);
        if (error < 0) error += 360.0f;
        error -= 180.0f;
    }

    // [Safety Check] 센서 에러(Hold Mode) 발생 시 처리
    if (ENV::g_sys.hold_mode != ENV::flight_hold_mode::MODE_NORMAL ) {
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
    if (ENV::g_sys.hold_mode != ENV::flight_hold_mode::MODE_NORMAL) {
        p->prev_rate = current_rate; // 센서 복구 시 D항 폭주 방지 (동기화)
        float p_out = p->kp * error;
        float i_out = std::clamp(p->ki * p->integral, -150.0f, 150.0f); // 기존 I값 유지
        return p_out + i_out;
    }

    //-----정상동작 -----
    // P 항
    float p_out = p->kp * error;

    // I 항 (Anti-Windup 적용)
    // p->integral += error * dt;
    // // 출력 기준으로 I항 제한 (예: 모터 출력의 최대 15%까지만 담당)
    // //float i_out = std::clamp(p->ki * p->integral, -150.0f, 150.0f); 
    // float i_out = std::clamp(p->ki * p->integral, -60.0f, 60.0f); 
 
    p->integral += error * dt;
    // i_out을 계산하기 전, integral 자체를 미리 제한 (I항의 영향력을 각속도 단위에서 제어)
    p->integral = std::clamp(p->integral, -200.0f, 200.0f); 
    float i_out = p->ki * p->integral;

    // D 항 (Measurement Derivative: 목표값 변화가 아닌 실제 센서 변화 기반)
    // 오차 변화량 대신 '현재 각속도 변화'를 쓰면 스틱을 급격히 움직일 때 튀는 현상이 줄어듭니다.
    // float d_out = p->kd * (p->prev_rate - current_rate) / dt;
    // p->prev_rate = current_rate;

    float raw_d_out = p->kd * (p->prev_rate - current_rate) / dt;
    // 간단한 LPF 예시 (alpha는 0.1~0.3 정도, 낮을수록 부드러움)
    p->d_out_filt = p->d_out_filt * (1.0f - _alpha) + raw_d_out * _alpha;
    float d_out = p->d_out_filt;    

    return p_out + i_out + d_out;
}

void PID::sync_pid_from_params()
{
    auto& p_mgr = Service::ParamMgr::get_instance();
    auto& values = p_mgr.get_values();

    // Roll / Pitch / Yaw 각 항의 비례, 적분, 미분 계수
    pid_roll_deg.kp   = values.MC_ROLL_P;
    pid_roll_deg.ki   = values.MC_ROLLRATE_I; // rate I를 재활용
    pid_roll_deg.kd   = values.MC_ROLLRATE_D;

    pid_pitch_deg.kp  = values.MC_PITCH_P;
    pid_pitch_deg.ki  = values.MC_PITCHRATE_I;
    pid_pitch_deg.kd  = values.MC_PITCHRATE_D;

    pid_yaw_deg.kp    = values.MC_YAW_P;
    pid_yaw_deg.ki    = values.MC_YAWRATE_I;
    pid_yaw_deg.kd    = 0.0f; // 별도 D 없음

    // 고도용 파라미터가 있다면 여기에 추가 (MPC_* 항목을 예시로 사용)
    pid_alt_pos.kp      = values.MPC_Z_P;
    pid_alt_pos.ki      = values.MPC_Z_VEL_I_ACC;
    pid_alt_pos.kd      = values.MPC_Z_VEL_D_ACC;

    pid_alt_rate.kp     = values.MPC_Z_VEL_P;
    pid_alt_rate.ki     = values.MPC_Z_VEL_I;
    pid_alt_rate.kd     = values.MPC_Z_VEL_D;
}


} 