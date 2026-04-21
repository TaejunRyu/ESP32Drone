#pragma once

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "common/mavlink.h"
#include "voltage_ryu.h"

// 기체 시동 상태 정의
typedef enum {
    ARM_STATE_DISARMED = 0,
    ARM_STATE_PREARM_CHECK,
    ARM_STATE_ARMED,
    ARM_STATE_FAILSAFE
} arming_state_t;


typedef enum {
    MODE_MANUAL = 0,      // 수동
    MODE_STABILIZED,      // Stabilized
    MODE_ACRO,            // Acro
    MODE_ALTCTL,          // 고도 (Altitude) 
    MODE_OFFBOARD,        // 오프보드
    MODE_POSCTL,          // 위치 (Position)
    MODE_LOITER,          // 대기 (Hold)
    MODE_MISSION,         // 미션 (Auto)
    MODE_RTL,             // 복귀 (Return)
    MODE_PRECISION_LAND   // 정밀 착륙
} flight_mode_t;

typedef struct {
    flight_mode_t current_mode;
    bool has_gps;          // 위치, 대기, 미션, 복귀 등에 필수
    bool has_baro;         // 고도 모드에 필수
    bool has_offboard;     // 오프보드 통신 연결 여부
    float battery_percent; // 배터리 체크용
} vehicle_status_t;


vehicle_status_t drone = {MODE_MANUAL, false, true, false, 35.0f};


// 매개변수 구조체 (C 버전)
typedef struct {
    char name[16];
    float value;
    uint8_t type;
} parameter_t;

// 전역 변수 예시 (실제로는 정렬된 배열이 좋음)
parameter_t global_params[] = {
    {"BAT1_LOW_THR", 0.40f, MAV_PARAM_TYPE_INT32},
    {"SYS_AUTOSTART", 4001.0f, MAV_PARAM_TYPE_INT32}
};

#define PARAM_COUNT (sizeof(global_params) / sizeof(parameter_t))

// 매개변수 찾기 함수
float get_param_value(const char* name) {
    for (int i = 0; i < PARAM_COUNT; i++) {
        if (strcmp(global_params[i].name, name) == 0) {
            return global_params[i].value;
        }
    }
    return 0.0f; // 기본값
}

static arming_state_t current_state = ARM_STATE_DISARMED;

// 3. 메인 루프 (FreeRTOS Task 등에서 호출)
void flight_control_update() {
    switch (current_state) {
        case ARM_STATE_PREARM_CHECK:
            if (get_battery_voltage()) {
                printf("[OK] 모든 점검 통과! 시동 완료.\n");
                current_state = ARM_STATE_ARMED;
                // 실제 모터 가동 함수 호출
            } else {
                current_state = ARM_STATE_FAILSAFE;
            }
            break;

        case ARM_STATE_ARMED:
            // 비행 중 배터리 감시 로직 추가 가능
            break;

        case ARM_STATE_FAILSAFE:
            // 오류 알림 후 대기 상태로 복귀
            current_state = ARM_STATE_DISARMED;
            break;

        default:
            break;
    }
}
