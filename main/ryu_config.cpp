#include "ryu_config.h"


// I2C 및 디바이스 핸들
i2c_master_bus_handle_t i2c_handle      = NULL;

// error발생시 대신 데이터를 읽어올 센서을 둔다 main=>[0](ado = vcc) / sub=>[1](ado = gnd)
i2c_master_dev_handle_t imu_handle[2]   = {0};   
i2c_master_dev_handle_t mag_handle[2]   = {0};
//i2c_master_dev_handle_t baro_handle[2]  = {0}; 


sys_t   g_sys = {
             .flight_mode       = MODE_STABILIZED, 
             .system_status     = MAV_STATE_STANDBY,  // 전원 초기 상태 (system_status = 3 = STANDBY)
             .system_health     = 0x00011111b,        // ERR::SYS_HEALTH_ALL_OK 
             .is_armed          = false, 
             .manual_hold_mode  = false,
             .error_hold_mode   = false,
             .gps_ready         = false,
             .payload_dropped   = false,
             .battery_voltage   = 0.0f,
             .loop_count        = 0,
};   


attitude_data_t g_attitude = {};

// qgc에서 지도상의 위치에서 home지정해준 위치.
qgc_home_pos_t qgc_home_pos = {};

// PID 구조체 초기화 (기본값 0)
// 제어기 명칭	       역할	    P (Proportional)	                I (Integral)	D (Derivative)	비고
// Alt Position     (Outer)	    목표 고도 유지	1.0 ~ 2.0	        0.0	            0.0	단위:       (m) -> (m/s) 변환
// Alt Rate         (Inner)	    상승/하강 속도 제어	50.0 ~ 100.0	 10.0 ~ 20.0	 0.05	        단위: (m/s) -> PWM 변량
drone_pid_t pid_alt_pos     = { .kp = 1.0f,  .ki = 0.0f,   .kd = 0.0f,  .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f };
drone_pid_t pid_alt_rate    = { .kp = 50.0f, .ki = 10.0f,  .kd = 0.5f,  .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f };

// 1. 각도 제어용 (Outer Loop) - P값 위주
// PID 구조체 초기값 (추천 가이드)
// 제어기 명칭	        역할	        P (Proportional)	I (Integral)	D (Derivative)	비고
// Angle (Outer)	    각도 유지	    4.5	                0.0	            0.0	            오직 P값만 사용해도
drone_pid_t pid_roll_angle  = { .kp = 4.5f, .ki = 0.0f,  .kd = 0.0f,  .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f };
drone_pid_t pid_pitch_angle = { .kp = 4.5f, .ki = 0.0f,  .kd = 0.0f,  .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f };
drone_pid_t pid_yaw_angle   = { .kp = 3.0f, .ki = 0.0f,  .kd = 0.0f,  .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f }; // Yaw는 조금 낮게

// 2. 각속도 제어용 (Inner Loop) - 실제 기체 반응 결정
// 제어기 명칭	        역할	        P (Proportional)	I (Integral)	D (Derivative)	비고
// Rate (Inner)	        진동/회전 제어	0.15	            0.1	            0.003	        가장 정밀하게 튜닝 필요
// Yaw Rate	회전        속도 제어	    0.20	            0.05	        0.0	            값은 거의 사용 안 함

drone_pid_t pid_roll_rate   = { .kp = 0.15f, .ki = 0.1f,  .kd = 0.003f,  .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f };
drone_pid_t pid_pitch_rate  = { .kp = 0.15f, .ki = 0.1f,  .kd = 0.003f,  .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f };
drone_pid_t pid_yaw_rate    = { .kp = 0.25f, .ki = 0.05f, .kd = 0.0f  ,  .integral =0.0f,    .err_prev=0.0f, .prev_rate=0.0f };

qgc_roll_pid_t qgc_roll_pid = {};

// GPS 데이터 초기화
gps_data_t g_gps = {};

// RC 송수신기 데이터 초기화
rc_data_t g_rc = {};

// 고도 데이터 초기화
altitude_data_t g_altitude = {};

// PID 디버그 데이터 초기화
pid_debug_t g_roll_pid = {};
pid_debug_t g_pitch_pid = {};
pid_debug_t g_yaw_pid = {};

// 배터리 데이터 초기화
battery_data_t g_battery = {};

// qgc에 보내는 heartbeat 정보
heartbeat_t g_heartbeat={
            .base_mode =   MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | //MAV_MODE_FLAG_TEST_ENABLED    |    // 테스트 모드 (실제 비행에서는 사용 안 함)
                            MAV_MODE_FLAG_STABILIZE_ENABLED  |  // 자세 제어 활성화
                            MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, // 원격제어 활성화
            .custom_mode = 0x00070000 // PX4 STABILIZE 모드: 0x00070000 (Main Mode 7) + 0x00000000 (Sub Mode 0)
};
        
// flight task에서 imu sensor의 값을 보관
imu_data_t      g_imu = {};

// flight task에서 imu sensor의 offset 값을 보관 ( error발생시 offset 측정을 할수 없기 때문에 처음 시동시 두쪽모두 측정한다.)
imu_offset_t    g_imu_offset[] ={};

// 기압계의 데이터를 보관
baro_t g_baro ={};

