#include "ryu_config.h"
#include <c_library_v2/common/mavlink.h>

sys_t g_sys = {
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

qgc_roll_pid_t qgc_roll_pid = {};

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

